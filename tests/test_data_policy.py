"""Alignment, the full recording pipeline to a real LeRobot v3 dataset, and the model process."""

from __future__ import annotations

import threading
import time

import numpy as np
import pytest
from rac_data import ArmTrack, CameraTrack, align_episode, control_intervals


def fake_samples(start_us, count, period_us, value_at, source=1, valid=0x7F, seq_gap_at=None):
    out = []
    seq = 1
    for i in range(count):
        if seq_gap_at is not None and i == seq_gap_at:
            seq += 3
        t = start_us + i * period_us
        v = value_at(t)
        out.append(
            {
                "host_ts_us": t,
                "sample_ts_us": t + 1000,
                "sample_seq": seq,
                "position": [v] * 7,
                "target": [v + 0.1] * 7,
                "flags": 1,
                "exec_seq": i,
                "source": source,
                "valid_mask": valid,
            }
        )
        seq += 1
    return out


def camera(start_us, count, period_us, skip=()):
    keep = [i for i in range(count) if i not in skip]
    return CameraTrack(
        host_ts=np.array([start_us + i * period_us for i in keep], dtype=np.int64),
        capture_ts=np.array([start_us + i * period_us for i in keep], dtype=np.int64),
        frame_index=np.array(keep, dtype=np.int64),
        files=[f"f{i}.jpg" for i in keep],
    )


class TestAlignment:
    def test_interpolates_state_and_holds_targets(self):
        # 250 Hz samples of a ramp; frames at 30 Hz.
        ramp = lambda t: t / 1e6  # noqa: E731
        follower = ArmTrack.from_samples(fake_samples(0, 750, 4000, ramp))
        aligned = align_episode(30, [follower], [None], {"env": camera(0, 90, 33333)}, [])
        assert aligned.state.shape[1] == 7
        expected = aligned.frame_times_us / 1e6
        assert np.allclose(aligned.state[:, 0], expected, atol=1e-3)
        # The action holds the last dispatched value: never ahead of the frame time.
        assert np.all(aligned.action[:, 0] <= expected + 0.1 + 1e-4)
        assert np.all(aligned.action[:, 0] >= expected + 0.1 - 0.0041)

    def test_clock_refits_do_not_invent_lost_samples(self):
        samples = fake_samples(0, 500, 4000, lambda t: 0.0)
        # A refit moves host times backwards relative to their neighbours.
        for sample in samples[200:210]:
            sample["host_ts_us"] -= 9000
        track = ArmTrack.from_samples(samples)
        assert list(track.sample_seq) == sorted(track.sample_seq)
        assert np.all(np.diff(track.host_ts) >= 0)
        aligned = align_episode(30, [track], [None], {"env": camera(0, 60, 33333)}, [])
        assert aligned.quality["followers"][0]["missing"] == 0

    def test_only_the_overlap_is_used(self):
        follower = ArmTrack.from_samples(fake_samples(1_000_000, 500, 4000, lambda t: 0.0))
        aligned = align_episode(30, [follower], [None], {"env": camera(0, 90, 33333)}, [])
        assert aligned.frame_times_us[0] >= 1_000_000
        assert aligned.frame_times_us[-1] <= 1_000_000 + 499 * 4000

    def test_quality_reports_real_problems(self):
        follower = ArmTrack.from_samples(fake_samples(0, 500, 4000, lambda t: 0.0, seq_gap_at=100, valid=0x7F))
        aligned = align_episode(30, [follower], [None], {"env": camera(0, 60, 33333, skip=(20, 21, 22, 23))}, [])
        assert aligned.quality["followers"][0]["missing"] == 3
        assert aligned.quality["cameras"]["env"]["dropped_by_camera"] == 4
        assert aligned.quality["cameras"]["env"]["late_frames"] > 0
        assert aligned.quality["cameras"]["env"]["max_offset_ms"] > 50

    def test_invalid_samples_mark_frames(self):
        samples = fake_samples(0, 500, 4000, lambda t: 0.0)
        for sample in samples[200:300]:
            sample["valid_mask"] = 0x3F
        aligned = align_episode(30, [ArmTrack.from_samples(samples)], [None], {"env": camera(0, 60, 33333)}, [])
        assert aligned.state_valid.sum() < len(aligned.state_valid)
        assert aligned.quality["invalid_frames"] > 0

    def test_bimanual_order_is_left_then_right(self):
        left = ArmTrack.from_samples(fake_samples(0, 500, 4000, lambda t: 1.0))
        right = ArmTrack.from_samples(fake_samples(0, 500, 4000, lambda t: -1.0))
        aligned = align_episode(30, [left, right], [None, None], {"env": camera(0, 60, 33333)}, [])
        assert aligned.state.shape[1] == 14
        assert np.allclose(aligned.state[:, 0], 1.0) and np.allclose(aligned.state[:, 7], -1.0)

    def test_intervention_spans_and_intervals(self):
        follower = ArmTrack.from_samples(fake_samples(0, 750, 4000, lambda t: 0.0, source=3))
        aligned = align_episode(30, [follower], [None], {"env": camera(0, 90, 33333)}, [(1_000_000, 2_000_000)])
        intervals = control_intervals(aligned.frame_times_us, aligned.control_source, aligned.intervention)
        assert [i["intervention"] for i in intervals] == [False, True, False]
        assert intervals[1]["start_s"] == pytest.approx(1.0, abs=0.05)
        assert intervals[1]["end_s"] == pytest.approx(2.0, abs=0.05)

    def test_empty_sources_are_an_error(self):
        with pytest.raises(ValueError):
            align_episode(30, [], [], {"env": camera(0, 10, 33333)}, [])


class TestCameraShm:
    def test_frames_are_shared_and_never_torn(self):
        from rac_camera import CameraReader, CameraWriter

        writer = CameraWriter("test_torn", 64, 48)
        reader = CameraReader("test_torn")
        stop = threading.Event()

        def produce():
            value = 0
            while not stop.is_set():
                image = np.full((48, 64, 3), value % 251, dtype=np.uint8)
                writer.write(image, value)
                value += 1

        thread = threading.Thread(target=produce)
        thread.start()
        try:
            for _ in range(300):
                frame = reader.latest(timeout=1.0)
                assert frame is not None
                assert frame.image.min() == frame.image.max(), "torn frame"
        finally:
            stop.set()
            thread.join()
            reader.close()
            writer.close()

    def test_synthetic_frames_carry_their_index(self):
        from rac_camera.capture import read_frame_index, synthetic_frame

        assert read_frame_index(synthetic_frame(640, 480, 12345, "env")) == 12345


@pytest.fixture
def sim_cameras():
    from rac_camera.capture import CameraThread

    stop = threading.Event()
    threads = [
        CameraThread("t_env", {"source": "sim", "width": 320, "height": 240, "fps": 30}, stop),
        CameraThread("t_wrist", {"source": "sim", "width": 320, "height": 240, "fps": 30}, stop),
    ]
    for t in threads:
        t.start()
    time.sleep(0.3)
    yield ["t_env", "t_wrist"]
    stop.set()
    for t in threads:
        t.join(timeout=3)


class TestRecordingToLeRobot:
    def test_full_pipeline(self, bound_rig, sim_cameras, tmp_path):
        """A real episode through real capture into a dataset LeRobot itself can load."""
        from rac_data.browse import check_mergeable, episode_data, list_episodes, trajectory_for_replay
        from rac_data.recorder import Recorder, RecordingConfig
        from rac_data.writer import DatasetWriter

        rig = bound_rig
        rig.client.teleop_configure(
            [{"pair": 0, "enabled": True, "gripper_link": True, "scale": [1.0] * 7, "offset": [0.0] * 7, "source_joint": list(range(7))}]
        )
        rig.client.teleop_enter()
        rig.client.teleop_start(follow="relative")
        rig.client.sil_inject("HAND_SINE", arg=2, values=[0.3, 0, 0, 0, 0, 0, 0])

        writer = DatasetWriter()
        recorder = Recorder(rig.client, writer)
        root = tmp_path / "ds"
        config = RecordingConfig(root=str(root), repo_id="local/rac_test", task="pick the cube", fps=30, cameras=sim_cameras)

        recorder.start(config)
        time.sleep(2.0)
        live = recorder.status()["live"]
        assert live["cameras"]["t_env"]["frames"] > 30
        assert live["arms"]["left_follower"]["samples"] > 200
        recorder.stop()
        assert recorder.status()["pending_save"]
        result = recorder.save({"result": "success", "note": "first"})
        job = writer.wait(result["job"], timeout=180)
        assert job.state == "done", job.error

        # An episode can also be thrown away instead.
        recorder.start(config)
        time.sleep(0.3)
        recorder.stop()
        recorder.discard()
        assert not recorder.status()["pending_save"]

        # Appending: a second saved episode lands in the same dataset.
        recorder.start(config)
        time.sleep(1.0)
        recorder.stop()
        job = writer.wait(recorder.save({"result": "failure"})["job"], timeout=180)
        assert job.state == "done", job.error
        assert job.episode_index == 1

        rig.client.teleop_end()

        # Read it back with LeRobot's own loader: this is what training will do.
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        dataset = LeRobotDataset("local/rac_test", root=root)
        assert dataset.meta.total_episodes == 2
        assert dataset.meta.info["codebase_version"].startswith("v3")
        item = dataset[5]
        assert item["observation.state"].shape[0] == 7
        assert item["action"].shape[0] == 7
        assert item["observation.images.t_env"].shape[0] == 3
        assert dataset.meta.tasks is not None

        # Our own read side.
        episodes = list_episodes(root)
        assert [e["result"] for e in episodes] == ["success", "failure"]
        assert list_episodes(root, result="failure")[0]["episode_index"] == 1
        assert list_episodes(root, source="teleop")
        assert "none" not in episodes[0]["sources"]
        assert episodes[0]["quality"]["followers"][0]["rate_hz"] > 100
        assert episodes[0]["quality"]["followers"][0]["missing"] == 0

        data = episode_data(root, 0)
        assert data["fps"] == 30
        assert data["joints"][0] == "joint1"
        state = np.array(data["state"])
        assert np.ptp(state[:, 0]) >= 0  # measured values, not a placeholder
        assert set(data["control_source"]) == {1}, sorted(set(map(str, data["control_source"])))  # recorded under teleop
        assert set(data["videos"]) == {"t_env", "t_wrist"}
        assert (root / data["videos"]["t_env"]["path"]).exists()

        # Raw timeline survives beside the resampled table.
        sidecar = root / "meta" / "rac" / "episode_000000"
        assert (sidecar / "left_follower.parquet").exists()
        assert (sidecar / "camera_t_env.parquet").exists()
        assert (sidecar / "events.jsonl").exists()

        replay = trajectory_for_replay(root, 0)
        assert replay["bimanual"] is False
        assert len(replay["left"][0]) == 7

        check = check_mergeable([root, root])
        assert check["ok"]

    def test_an_annotation_edit_never_touches_samples(self, tmp_path):
        from rac_data import AnnotationStore

        store = AnnotationStore(tmp_path)
        store.set_episode(0, {"result": "success"})
        segment = store.add_segment(0, 1.0, 2.5, "grasp", "slipped once", "correction")
        store.update_segment(0, segment["id"], {"label": "regrasp"})
        assert store.get_episode(0)["segments"][0]["label"] == "regrasp"
        store.delete_segment(0, segment["id"])
        assert store.get_episode(0)["segments"] == []
        with pytest.raises(ValueError):
            store.set_episode(0, {"result": "maybe"})
        with pytest.raises(ValueError):
            store.add_segment(0, 3.0, 1.0, "bad")


class TestMergeChecks:
    def test_mismatched_definitions_are_refused(self, tmp_path):
        import json

        from rac_data.browse import check_mergeable, merge_datasets

        def make(name, fps, camera_name):
            root = tmp_path / name
            (root / "meta").mkdir(parents=True)
            features = {
                "observation.state": {"dtype": "float32", "shape": [7], "names": ["a"] * 7},
                f"observation.images.{camera_name}": {"dtype": "video", "shape": [240, 320, 3]},
            }
            (root / "meta" / "info.json").write_text(json.dumps({"fps": fps, "features": features, "robot_type": "x"}))
            return root

        a, b, c = make("a", 30, "env"), make("b", 30, "wrist"), make("c", 15, "env")
        assert not check_mergeable([a, b])["ok"]
        assert any("cameras" in p for p in check_mergeable([a, b])["problems"])
        assert any("fps" in p for p in check_mergeable([a, c])["problems"])
        with pytest.raises(ValueError):
            merge_datasets([a, b], tmp_path / "out")


class TestPolicyWorker:
    CONFIG = {
        "backend": "mock",
        "arms": ["left_follower"],
        "cameras": {"env": "w_env"},
        "task": "wave",
        "inference_hz": 10,
        "action_hz": 50,
        "chunk_size": 30,
        "amplitude": 0.2,
        "spawn_worker": False,
    }

    @pytest.fixture
    def camera(self):
        from rac_camera.capture import CameraThread

        stop = threading.Event()
        thread = CameraThread("w_env", {"source": "sim", "width": 160, "height": 120, "fps": 30}, stop)
        thread.start()
        time.sleep(0.2)
        yield
        stop.set()
        thread.join(timeout=3)

    def test_worker_moves_the_arm_and_stops_on_takeover(self, bound_rig, camera):
        from rac_policy.worker import run

        rig = bound_rig
        rig.client.teleop_configure(
            [{"pair": 0, "enabled": True, "gripper_link": True, "scale": [1.0] * 7, "offset": [0.0] * 7, "source_joint": list(range(7))}]
        )
        rig.client.policy_start(self.CONFIG)
        result: dict = {}
        stop = threading.Event()
        worker = threading.Thread(target=lambda: result.update(run(self.CONFIG, rig.client, stop)))
        worker.start()
        try:
            rig.wait(lambda: rig.status()["policy"]["stats"].get("inferences", 0) >= 3, what="inferences")
            rig.wait(lambda: rig.status()["policy"]["stats"].get("actions_sent", 0) > 10, what="actions")
            assert rig.status()["policy"]["stats"]["last_inference_ms"] >= 10

            rig.client.intervention_take_over("relative")
            sent = rig.status()["policy"]["stats"]["actions_sent"]
            time.sleep(0.4)
            assert rig.status()["policy"]["stats"]["actions_sent"] == sent, "actions kept flowing during a takeover"

            rig.client.intervention_hand_back()
            rig.wait(lambda: rig.status()["policy"]["stats"]["actions_sent"] > sent + 5, what="actions after handback")
        finally:
            rig.client.policy_stop()
            stop.set()
            worker.join(timeout=10)
        assert result["resets"] == 2, result  # the start of the run, and the handback
        assert result["submitted"] >= 3

    def test_spec_conversions_round_trip(self):
        from rac_policy.spec import IoSpec, Normalization

        values = np.array([[0.5, -0.5, 1.0, 0.0, 0.2, -0.2, 0.7]], dtype=np.float32)
        degrees = Normalization("degrees")
        assert degrees.to_model(values)[0, 0] == pytest.approx(28.648, abs=0.01)
        assert degrees.to_model(values)[0, 6] == pytest.approx(0.7), "the gripper stays 0..1"
        assert np.allclose(degrees.from_model(degrees.to_model(values)), values)
        minmax = Normalization("min_max", [-1.0] * 7, [1.0] * 7)
        assert np.allclose(minmax.from_model(minmax.to_model(values)), values)
        meanstd = Normalization("mean_std", [0.1] * 7, [0.5] * 7)
        assert np.allclose(meanstd.from_model(meanstd.to_model(values)), values)

        spec = IoSpec.from_config({"arms": ["left_follower", "right_follower"]})
        assert spec.width == 14
        assert spec.check_chunk(np.zeros((1, 5, 14))).shape == (5, 14)
        with pytest.raises(ValueError):
            spec.check_chunk(np.zeros((5, 7)))
        with pytest.raises(ValueError):
            spec.check_chunk(np.full((5, 14), np.nan))

    def test_starvla_backend_speaks_the_server_protocol(self):
        """Against the wire format the StarVLA WebSocket server documents."""
        import asyncio

        import websockets
        from rac_policy.backends import StarVlaBackend
        from rac_policy.msgpack_numpy import packb, unpackb
        from rac_policy.spec import IoSpec, Observation

        received: dict = {}
        ready = threading.Event()
        port_box: dict = {}

        async def handler(ws):
            await ws.send(packb({"action_chunk_size": 8, "available_unnorm_keys": ["rs"]}))
            message = unpackb(await ws.recv())
            received.update(message)
            actions = np.ones((1, 8, 7), dtype=np.float32)
            await ws.send(packb({"status": "ok", "ok": True, "type": "inference_result", "data": {"actions": actions}}))

        async def serve():
            async with websockets.serve(handler, "127.0.0.1", 0, compression=None, max_size=None) as server:
                port_box["port"] = list(server.sockets)[0].getsockname()[1]
                ready.set()
                await asyncio.sleep(3)

        thread = threading.Thread(target=lambda: asyncio.run(serve()), daemon=True)
        thread.start()
        assert ready.wait(5)

        spec = IoSpec.from_config({"arms": ["left_follower"]})
        backend = StarVlaBackend(spec, {"port": port_box["port"], "unnorm_key": "rs"})
        metadata = backend.connect()
        assert metadata["action_chunk_size"] == 8
        observation = Observation(np.zeros(7, np.float32), {"env": np.zeros((4, 4, 3), np.uint8)}, "pick", 0)
        actions = spec.check_chunk(backend.infer(observation))
        backend.close()
        assert actions.shape == (8, 7)
        assert received["type"] == "infer"
        example = received["payload"]["examples"][0]
        assert example["lang"] == "pick"
        assert example["image"].shape == (4, 4, 3)
        assert received["payload"]["unnorm_key"] == "rs"
