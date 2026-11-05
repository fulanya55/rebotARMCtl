"""The service against the firmware core: every flow in sections 1.1 to 1.8 of the requirements."""

from __future__ import annotations

import time

import numpy as np
import pytest
from rac_link import ServiceError


def identity_pair(client, pair=0):
    """Drop the cross-model preset: these tests check following, not the preset's numbers."""
    client.teleop_configure(
        [{"pair": pair, "enabled": True, "gripper_link": True, "scale": [1.0] * 7, "offset": [0.0] * 7, "source_joint": list(range(7))}]
    )


class TestDevice:
    def test_connecting_only_communicates(self, rig):
        status = rig.client.connect(rig.uri)
        assert status["connected"]
        assert status["simulated"]
        assert status["device"]["protocol"] == 1
        assert status["source"] == "none"
        assert all(not a.get("torque_on") for a in status["arms"])

    def test_discovery_identifies_models_without_torque(self, rig):
        rig.client.connect(rig.uri)
        ports = rig.client.discover()["ports"]
        assert [p["detected_model"] for p in ports] == ["REBOT_B601_RS", "REBOT_B601_RS", "STAR_102_HD", "STAR_102_HD"]
        assert all(p["online_mask"] == 0x7F for p in ports)

    def test_bindings_persist_across_reconnect(self, bound_rig):
        rig = bound_rig
        rig.client.disconnect()
        assert not rig.status()["connected"]
        rig.client.connect(rig.uri)
        rig.wait_valid("left_follower", "left_leader")
        assert rig.arm("left_follower")["binding"]["serial"] == "rs-l"

    def test_a_leader_only_arm_cannot_be_a_follower(self, rig):
        rig.client.connect(rig.uri)
        with pytest.raises(ServiceError) as exc:
            rig.client.bind([{"port": 2, "arm": "left_follower", "model": "star_102_ld"}])
        assert exc.value.code == "arm.unsupported"

    def test_port_kind_must_match_the_arm(self, rig):
        rig.client.connect(rig.uri)
        with pytest.raises(ServiceError) as exc:
            rig.client.bind([{"port": 0, "arm": "left_leader", "model": "star_102_hd"}])
        assert exc.value.code == "arm.unsupported"

    def test_duplicate_role_is_refused(self, rig):
        rig.client.connect(rig.uri)
        with pytest.raises(ServiceError):
            rig.client.bind(
                [
                    {"port": 0, "arm": "left_follower", "model": "rebot_b601_rs"},
                    {"port": 1, "arm": "left_follower", "model": "rebot_b601_rs"},
                ]
            )

    def test_status_comes_from_controller_feedback(self, bound_rig):
        rig = bound_rig
        follower = rig.wait(lambda: (a := rig.arm("left_follower")).get("rate_hz") and a, what="rate")
        assert follower["rate_hz"] > 50
        assert follower["missing"] == 0

    def test_clock_alignment_converges(self, bound_rig):
        rig = bound_rig
        rig.wait(lambda: rig.status()["clock"]["synced"], what="clock sync")
        clock = rig.status()["clock"]
        assert clock["best_rtt_us"] < 20000

    def test_a_token_is_required(self, bound_rig):
        from rac_link import IpcClient

        with pytest.raises(ServiceError) as exc:
            IpcClient(port=bound_rig.server.port, token="wrong")
        assert exc.value.code == "auth.refused"


class TestCalibration:
    def test_zero_direction_range_and_save(self, bound_rig):
        rig = bound_rig
        client = rig.client
        client.sil_inject("SET_HAND_POSE", arg=2, values=[0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        rig.wait(lambda: abs(rig.arm("left_leader")["position"][0] - 0.5) < 0.02, what="hand motion")

        # Zero at the current pose reads zero afterwards.
        client.calibration_capture_zero("left_leader", [0])
        rig.wait(lambda: abs(rig.arm("left_leader")["position"][0]) < 0.02, what="zeroed reading")

        # Direction flips the sign of the reading, without moving anything.
        client.sil_inject("SET_HAND_POSE", arg=2, values=[0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        rig.wait(lambda: abs(rig.arm("left_leader")["position"][0] - 0.2) < 0.02, what="positive reading")
        client.calibration_set_direction("left_leader", 0, -1)
        rig.wait(lambda: abs(rig.arm("left_leader")["position"][0] + 0.2) < 0.02, what="flipped reading")

        # Range comes from what was observed, inset by a margin.
        client.calibration_range_start("left_leader")
        client.sil_inject("SET_HAND_POSE", arg=2, values=[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        rig.wait(lambda: abs(rig.arm("left_leader")["position"][0] + 0.5) < 0.02, what="range sweep")
        client.sil_inject("SET_HAND_POSE", arg=2, values=[0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        rig.wait(lambda: abs(rig.arm("left_leader")["position"][0] - 0.2) < 0.02, what="range sweep back")
        result = client.calibration_range_stop("left_leader")
        assert result["draft"]["lower"][0] == pytest.approx(-0.48, abs=0.03)
        assert result["draft"]["upper"][0] == pytest.approx(0.18, abs=0.03)

        # Gripper range is captured in raw units and normalizes 0 to 1.
        client.sil_inject("SET_HAND_POSE", arg=2, values=[0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2])
        time.sleep(0.6)
        client.calibration_capture_gripper("left_leader", "closed")
        client.sil_inject("SET_HAND_POSE", arg=2, values=[0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2])
        time.sleep(0.6)
        client.calibration_capture_gripper("left_leader", "open")
        rig.wait(lambda: abs(rig.arm("left_leader")["position"][6] - 1.0) < 0.05, what="gripper open reading")

        saved = client.calibration_save("left_leader", note="bench")
        assert saved["draft"]["version"] == 1
        assert set(saved["draft"]["steps_done"]) >= {"zero", "direction", "range", "gripper"}

        # A new version never overwrites an old one.
        client.calibration_set_direction("left_leader", 0, 1)
        assert client.calibration_save("left_leader")["draft"]["version"] == 2
        loaded = client.calibration_load("left_leader", 1)
        assert loaded["draft"]["direction"][0] == -1.0
        assert rig.arm("left_leader")["calibration_version"] == 1

    def test_calibration_follows_the_serial_across_reconnect(self, bound_rig):
        rig = bound_rig
        rig.client.calibration_set_direction("left_leader", 2, -1)
        rig.client.calibration_save("left_leader")
        rig.client.disconnect()
        rig.client.connect(rig.uri)
        rig.wait_valid("left_leader")
        arm = rig.arm("left_leader")
        assert arm["calibration_version"] == 1
        assert arm["serial"] == "hd-l"

    def test_verify_joint_reports_a_real_motion(self, bound_rig):
        rig = bound_rig
        result = rig.client.calibration_verify_joint("left_follower", 0, 0.2)
        assert result["direction_ok"] and result["tracking_ok"], result
        assert abs(result["moved"]) > 0.1

    def test_calibrating_an_energized_arm_is_refused(self, bound_rig):
        rig = bound_rig
        rig.client.move_to("left_follower", [0.1] * 6 + [0.5], 0.5)
        rig.wait(lambda: rig.arm("left_follower").get("torque_on"), what="torque on")
        with pytest.raises(ServiceError) as exc:
            rig.client.calibration_capture_zero("left_follower")
        assert exc.value.code == "control.busy"


class TestTeleop:
    def test_align_then_follow_runs_on_the_controller(self, bound_rig):
        rig = bound_rig
        client = rig.client
        identity_pair(client)
        client.sil_inject("SET_HAND_POSE", arg=2, values=[0.5, 0.4, 0.4, 0.2, -0.2, 0.3, 0.6])
        rig.wait(lambda: abs(rig.arm("left_leader")["position"][0] - 0.5) < 0.05, what="leader pose")

        client.teleop_enter()
        assert rig.status()["source"] == "teleop"
        before = rig.arm("left_follower")["position"][0]
        time.sleep(0.3)
        assert rig.arm("left_follower")["position"][0] == pytest.approx(before, abs=0.03)

        client.teleop_align(speed=1.0)
        rig.wait(lambda: rig.status()["pairs"][0]["phase"] == "ALIGNED", timeout=15, what="alignment")
        assert rig.arm("left_follower")["position"][0] == pytest.approx(0.5, abs=0.05)

        client.teleop_start()
        client.sil_inject("SET_HAND_POSE", arg=2, values=[-0.2, 0.4, 0.4, 0.2, -0.2, 0.3, 0.2])
        rig.wait(lambda: abs(rig.arm("left_follower")["position"][0] + 0.2) < 0.05, what="following")
        # The leader's raw 0.2 rad over its 60 degrees of travel is 0.19 of the range.
        assert rig.arm("left_follower")["position"][6] == pytest.approx(0.2 / np.radians(60), abs=0.05)

        # Sync timing is reported by the controller, on its own grid.
        pair = rig.status()["pairs"][0]
        assert pair["phase"] == "FOLLOWING"
        assert pair["sync_period_us"] == pytest.approx(4000, abs=1000)
        assert pair["sync_latency_us"] < 30000

    def test_following_survives_the_computer_doing_nothing(self, bound_rig):
        """The mapping runs on the controller: no host traffic is needed to follow."""
        rig = bound_rig
        identity_pair(rig.client)
        rig.client.teleop_enter()
        rig.client.teleop_start(follow="relative")
        q0 = rig.arm("left_follower")["position"][0]
        rig.client.sil_inject("HAND_SINE", arg=2, values=[0.3, 0, 0, 0, 0, 0, 0])
        rig.keep_samples = True
        time.sleep(3.0)
        rig.keep_samples = False
        follower = [s for s in rig.samples if s["arm"] == "left_follower"]
        assert len(follower) > 600
        positions = np.array([s["position"][0] for s in follower])
        assert positions.max() - positions.min() > 0.2
        assert all(s["source"] == 1 for s in follower)
        assert q0 == pytest.approx(0.0, abs=0.05)

    def test_relative_offset_does_not_touch_calibration(self, bound_rig):
        rig = bound_rig
        identity_pair(rig.client)
        rig.client.sil_inject("SET_HAND_POSE", arg=2, values=[0.6, 0, 0, 0, 0, 0, 0])
        rig.wait(lambda: abs(rig.arm("left_leader")["position"][0] - 0.6) < 0.03, what="leader pose")
        version = rig.arm("left_follower").get("calibration_version")
        rig.client.teleop_enter()
        rig.client.teleop_start(follow="relative")
        pair = rig.wait(lambda: (p := rig.status()["pairs"][0])["phase"] == "FOLLOWING" and p, what="following")
        assert pair["follow_mode"] == "RELATIVE"
        assert rig.status()["pairs"][0]["rel_offset"][0] == pytest.approx(-0.6, abs=0.05)
        assert rig.arm("left_follower")["position"][0] == pytest.approx(0.0, abs=0.04)
        assert rig.arm("left_follower").get("calibration_version") == version

    def test_pause_resume_end(self, bound_rig):
        rig = bound_rig
        identity_pair(rig.client)
        rig.client.teleop_enter()
        rig.client.teleop_start(follow="relative")
        rig.client.teleop_pause()
        rig.client.sil_inject("SET_HAND_POSE", arg=2, values=[0.5, 0, 0, 0, 0, 0, 0])
        time.sleep(0.8)
        assert rig.arm("left_follower")["position"][0] == pytest.approx(0.0, abs=0.04)
        rig.client.teleop_resume()
        rig.client.teleop_end()
        status = rig.status()
        assert status["source"] == "none"
        assert status["teleop"]["active"] is False
        rig.wait(lambda: rig.arm("left_follower")["mode"] == "HOLD", what="hold mode")

    def test_presets_cover_cross_model_pairs(self, bound_rig):
        config = bound_rig.client.teleop_config()
        assert config["pairs"]["0"]["preset"] is True
        # Seeed's Star to reBot directions. Both arms count from their calibration pose.
        assert config["pairs"]["0"]["scale"] == [1.0, 1.0, -1.0, -1.0, -1.0, 1.0, 1.0]
        assert config["pairs"]["0"]["offset"] == [0.0] * 7

    def test_two_pairs_run_together(self, rig):
        client = rig.client
        client.connect(rig.uri)
        client.bind(
            [
                {"port": 0, "arm": "left_follower", "model": "rebot_b601_rs"},
                {"port": 1, "arm": "right_follower", "model": "rebot_b601_rs"},
                {"port": 2, "arm": "left_leader", "model": "star_102_hd"},
                {"port": 3, "arm": "right_leader", "model": "star_102_hd"},
            ]
        )
        rig.wait_valid("left_follower", "right_follower", "left_leader", "right_leader")
        identity_pair(client, 0)
        identity_pair(client, 1)
        client.teleop_enter()
        client.teleop_start()
        client.sil_inject("SET_HAND_POSE", arg=2, values=[0.3, 0, 0, 0, 0, 0, 0])
        client.sil_inject("SET_HAND_POSE", arg=3, values=[-0.3, 0, 0, 0, 0, 0, 0])
        rig.wait(
            lambda: abs(rig.arm("left_follower")["position"][0] - 0.3) < 0.05
            and abs(rig.arm("right_follower")["position"][0] + 0.3) < 0.05,
            what="both followers",
        )
        pairs = rig.status()["pairs"]
        assert pairs[0]["phase"] == pairs[1]["phase"] == "FOLLOWING"


class TestExecution:
    def test_execute_runs_a_trajectory_and_finishes(self, bound_rig):
        rig = bound_rig
        points = np.linspace(0, 0.4, 50)
        trajectory = [[float(p)] * 6 + [0.5] for p in points]
        result = rig.client.execute("left_follower", trajectory, 0.02, "teach", blend_s=0.5)
        assert rig.status()["source"] == "teach"
        rig.wait(lambda: rig.status()["source"] == "none", timeout=15, what="finish")
        assert result["points"] > 50
        assert rig.arm("left_follower")["position"][0] == pytest.approx(0.4, abs=0.05)
        assert "execution_finished" in rig.event_kinds()

    def test_long_trajectory_respects_the_queue(self, bound_rig):
        rig = bound_rig
        t = np.arange(0, 6.0, 0.01)  # 600 points: well past the 128 point queue
        q = 0.2 * np.sin(2 * np.pi * 0.5 * t)
        trajectory = [[float(v)] * 6 + [0.5] for v in q]
        rig.client.execute("left_follower", trajectory, 0.01, "replay", blend_s=0.3)
        rig.wait(lambda: rig.status()["source"] == "none", timeout=20, what="finish")
        assert not [e for e in rig.events if e["kind"] == "execution_finished" and e.get("error")]

    def test_only_one_source_at_a_time(self, bound_rig):
        rig = bound_rig
        rig.client.execute("left_follower", [[0.2] * 6 + [0.5]] * 200, 0.02, "replay")
        with pytest.raises(ServiceError) as exc:
            rig.client.teleop_enter()
        assert exc.value.code == "control.busy"

    def test_stop_clears_both_queues_and_holds(self, bound_rig):
        rig = bound_rig
        trajectory = [[float(v)] * 6 + [0.5] for v in np.linspace(0, 1.0, 500)]
        rig.client.execute("left_follower", trajectory, 0.02, "replay", blend_s=0.2)
        rig.wait(lambda: rig.arm("left_follower")["position"][0] > 0.05, what="motion")
        rig.client.stop()
        status = rig.status()
        assert status["source"] == "none"
        held = rig.arm("left_follower")["position"][0]
        time.sleep(0.5)
        assert rig.arm("left_follower")["position"][0] == pytest.approx(held, abs=0.03)
        assert rig.arm("left_follower")["queue_depth"] == 0
        assert rig.arm("left_follower")["mode"] == "HOLD"

    def test_pause_and_resume_a_trajectory(self, bound_rig):
        rig = bound_rig
        trajectory = [[float(v)] * 6 + [0.5] for v in np.linspace(0, 0.8, 200)]
        rig.client.execute("left_follower", trajectory, 0.02, "replay", blend_s=0.2)
        rig.wait(lambda: rig.arm("left_follower")["position"][0] > 0.1, what="motion")
        rig.client.pause()
        time.sleep(0.2)
        paused_at = rig.arm("left_follower")["position"][0]
        time.sleep(0.6)
        assert rig.arm("left_follower")["position"][0] == pytest.approx(paused_at, abs=0.03)
        rig.client.resume()
        rig.wait(lambda: rig.status()["source"] == "none", timeout=20, what="finish")
        assert rig.arm("left_follower")["position"][0] == pytest.approx(0.8, abs=0.05)

    def test_home_is_independent(self, bound_rig):
        rig = bound_rig
        rig.client.move_to("left_follower", [0.4, 0.4, 0.4, 0, 0, 0, 0.5], 0.5)
        rig.wait(lambda: rig.status()["source"] == "none", timeout=10, what="move")
        rig.client.home(duration_s=1.0)
        assert rig.status()["source"] == "home"
        rig.wait(lambda: rig.status()["source"] == "none", timeout=10, what="home")
        assert rig.arm("left_follower")["position"][0] == pytest.approx(0.0, abs=0.05)

    def test_nan_trajectories_never_reach_the_arm(self, bound_rig):
        with pytest.raises(ServiceError) as exc:
            bound_rig.client.execute("left_follower", [[float("nan")] * 7], 0.02, "teach")
        assert exc.value.code == "request.bad_argument"

    def test_a_leader_only_arm_refuses_to_execute(self, rig):
        rig.client.connect(rig.uri)
        rig.client.bind([{"port": 2, "arm": "left_leader", "model": "star_102_ld"}])
        rig.wait_valid("left_leader")
        with pytest.raises(ServiceError) as exc:
            rig.client.execute("left_leader", [[0.0] * 7], 0.02, "teach")
        assert exc.value.code == "arm.unsupported"

    def test_live_target_updates(self, bound_rig):
        rig = bound_rig
        for v in np.linspace(0, 0.3, 30):
            rig.client.set_target("left_follower", [float(v)] * 6 + [0.5])
            time.sleep(0.02)
        time.sleep(0.8)
        assert rig.arm("left_follower")["position"][0] == pytest.approx(0.3, abs=0.05)

    def test_a_target_step_is_smoothed(self, bound_rig):
        """A slider dragged across its range still starts gently."""
        rig = bound_rig
        rig.client.set_target("left_follower", [0.8] + [0.0] * 5 + [0.5])
        time.sleep(0.12)
        # At the 4 rad/s2 limit this much time covers about 3 cm of a radian.
        assert abs(rig.arm("left_follower")["target"][0]) < 0.08
        rig.wait(lambda: abs(rig.arm("left_follower")["target"][0] - 0.8) < 0.01, timeout=5, what="target reached")

    def test_stopping_ends_a_live_target(self, bound_rig):
        rig = bound_rig
        rig.client.set_target("left_follower", [0.8] + [0.0] * 5 + [0.5])
        time.sleep(0.2)
        rig.client.call("run.stop", kind="hold")
        time.sleep(0.1)
        held = rig.arm("left_follower")["target"][0]
        time.sleep(0.4)
        assert rig.arm("left_follower")["target"][0] == pytest.approx(held, abs=0.02)
        assert abs(held - 0.8) > 0.1

    def test_a_finished_move_settles_into_hold(self, bound_rig):
        rig = bound_rig
        pose = [0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.5]
        rig.client.move_to("left_follower", pose, 0.5)
        rig.wait(lambda: rig.status()["source"] == "none", timeout=10, what="move")
        rig.wait(lambda: rig.arm("left_follower")["mode"] == "HOLD", what="hold mode")
        assert rig.arm("left_follower")["torque_on"]


class TestGuide:
    def test_guide_is_rs_only(self, rig):
        rig.client.connect(rig.uri)
        rig.client.bind([{"port": 2, "arm": "left_follower", "model": "star_102_fl"}])
        rig.wait_valid("left_follower")
        with pytest.raises(ServiceError) as exc:
            rig.client.guide_start("left_follower")
        assert exc.value.code == "arm.unsupported"

    def test_guide_moves_by_hand_and_stops_holding(self, bound_rig):
        rig = bound_rig
        rig.client.guide_start("left_follower", gravity=False)
        assert rig.status()["source"] == "guide"
        rig.client.sil_inject("SET_HAND_POSE", arg=0, values=[0.4, 0.3, 0.3, 0, 0, 0, 0])
        rig.wait(lambda: abs(rig.arm("left_follower")["position"][0] - 0.4) < 0.05, what="hand guiding")
        rig.client.guide_stop("left_follower")
        rig.client.sil_inject("SET_HAND_POSE", arg=0, values=[0.0, 0.3, 0.3, 0, 0, 0, 0])
        time.sleep(0.3)
        assert rig.status()["source"] == "none"
        assert rig.arm("left_follower")["mode"] == "HOLD"


class TestSafety:
    def test_estop_stops_everything_and_needs_a_reset(self, bound_rig):
        rig = bound_rig
        trajectory = [[float(v)] * 6 + [0.5] for v in np.linspace(0, 1.0, 500)]
        rig.client.execute("left_follower", trajectory, 0.02, "replay", blend_s=0.2)
        rig.wait(lambda: rig.arm("left_follower")["position"][0] > 0.05, what="motion")

        rig.client.sil_inject("PRESS_ESTOP")
        rig.wait(lambda: rig.status()["estop"] == "PRESSED", what="estop")
        status = rig.status()
        assert status["source"] == "none"
        assert rig.arm("left_follower")["mode"] == "ESTOP"
        assert "estop_pressed" in rig.event_kinds()

        with pytest.raises(ServiceError) as exc:
            rig.client.teleop_enter()
        assert exc.value.code == "safety.estop"
        with pytest.raises(ServiceError):
            rig.client.reset_estop()

        rig.client.sil_inject("RELEASE_ESTOP")
        rig.wait(lambda: rig.status()["estop"] == "LATCHED", what="latched")
        rig.client.reset_estop()
        rig.wait(lambda: rig.status()["estop"] == "CLEAR", what="cleared")
        assert rig.arm("left_follower")["mode"] == "IDLE"

        # After a reset the platform works again, from a fresh generation.
        rig.wait_valid("left_follower")
        rig.client.move_to("left_follower", [0.0] * 6 + [0.5], 0.5)
        rig.wait(lambda: rig.status()["source"] == "none", timeout=10, what="move after reset")

    def test_motor_loss_stops_teleop(self, bound_rig):
        rig = bound_rig
        identity_pair(rig.client)
        rig.client.teleop_enter()
        rig.client.teleop_start(follow="relative")
        rig.client.sil_inject("DROP_MOTOR", arg=0, arg2=1 << 2)
        rig.wait(lambda: rig.status()["pairs"][0]["phase"] == "OFF", what="pair off")
        rig.wait(lambda: "motor_lost" in rig.event_kinds(), what="motor lost event")

    def test_service_loss_stops_the_arm(self, bound_rig):
        """The host watchdog on the controller. Heartbeats stop when the link closes."""
        rig = bound_rig
        trajectory = [[float(v)] * 6 + [0.5] for v in np.linspace(0, 1.0, 500)]
        rig.client.execute("left_follower", trajectory, 0.02, "replay", blend_s=0.2)
        rig.wait(lambda: rig.arm("left_follower")["position"][0] > 0.03, what="motion")
        rig.client.disconnect()
        rig.client.connect(rig.uri)
        status = rig.status()
        assert status["source"] == "none"
        rig.wait_valid("left_follower")
        q = rig.arm("left_follower")["position"][0]
        time.sleep(0.4)
        assert rig.arm("left_follower")["position"][0] == pytest.approx(q, abs=0.04)


class TestPolicyAndIntervention:
    CONFIG = {"backend": "external", "arms": ["left_follower"], "inference_hz": 5, "action_hz": 50, "spawn_worker": False}

    def chunk(self, value, length=40):
        return [[float(value)] * 6 + [0.5]] * length

    def test_action_chunks_execute_through_the_one_entry(self, bound_rig):
        rig = bound_rig
        status = rig.client.policy_start(self.CONFIG)
        assert rig.status()["source"] == "policy"
        token = status["token"]
        from rac_link import host_now_us

        rig.client.policy_submit(self.chunk(0.3), token, host_now_us(), 12.0)
        rig.wait(lambda: rig.status()["policy"]["stats"].get("actions_sent", 0) > 30, what="actions")
        rig.wait(lambda: abs(rig.arm("left_follower")["position"][0] - 0.3) < 0.1, what="moving")
        stats = rig.status()["policy"]["stats"]
        assert stats["inferences"] == 1
        assert stats["last_inference_ms"] == 12.0
        rig.client.policy_stop()
        assert rig.status()["source"] == "none"

    def test_rates_are_independent(self, bound_rig):
        rig = bound_rig
        config = {**self.CONFIG, "action_hz": 25}
        token = rig.client.policy_start(config)["token"]
        from rac_link import host_now_us

        rig.client.policy_submit(self.chunk(0.1, 200), token, host_now_us(), 5.0)
        time.sleep(1.0)
        sent = rig.status()["policy"]["stats"]["actions_sent"]
        assert 18 <= sent <= 32

    def test_takeover_discards_old_results_and_stops_queued_motion(self, bound_rig):
        rig = bound_rig
        identity_pair(rig.client)
        from rac_link import host_now_us

        token = rig.client.policy_start(self.CONFIG)["token"]
        rig.client.policy_submit(self.chunk(1.0, 400), token, host_now_us(), 5.0)
        rig.wait(lambda: rig.arm("left_follower")["position"][0] > 0.05, what="policy motion")

        result = rig.client.intervention_take_over("relative")
        assert result["ok"]
        status = rig.status()
        assert status["source"] == "intervention"
        assert status["intervention"]["active"]
        assert status["policy"]["state"] == "suspended"
        assert status["policy"]["buffered"] == 0

        # An inference that was already running comes back late.
        with pytest.raises(ServiceError) as exc:
            rig.client.policy_submit(self.chunk(1.0), token, host_now_us(), 5.0)
        assert exc.value.code == "policy.stale_token"
        assert rig.status()["policy"]["stats"]["rejected"] >= 1

        q = rig.arm("left_follower")["position"][0]
        time.sleep(0.5)
        assert rig.arm("left_follower")["position"][0] == pytest.approx(q, abs=0.04)

        # The person moves the leader, and the follower follows by the same amount.
        rig.client.sil_inject("SET_HAND_POSE", arg=2, values=[0.3, 0, 0, 0, 0, 0, 0])
        rig.wait(lambda: abs(rig.arm("left_follower")["position"][0] - (q + 0.3)) < 0.06, what="correction")

        back = rig.client.intervention_hand_back()
        assert back["token"] != token
        status = rig.status()
        assert status["source"] == "policy"
        assert not status["intervention"]["active"]
        assert status["intervention"]["segments"][-1]["outcome"] == "handed_back"

        # The worker sees a one-shot reset and the new token.
        poll = rig.client.call("policy.poll")
        assert poll["reset"] is True
        assert poll["token"] == back["token"]
        assert rig.client.call("policy.poll")["reset"] is False

        # With the new token, chunks execute again.
        rig.client.policy_submit(self.chunk(0.0), back["token"], host_now_us(), 5.0)
        rig.wait(lambda: abs(rig.arm("left_follower")["position"][0]) < 0.1, timeout=8, what="model back in control")
        kinds = rig.event_kinds()
        for kind in ("intervention_begin", "intervention_end", "policy_suspended", "policy_resumed_after_intervention"):
            assert kind in kinds

    def test_takeover_by_alignment(self, bound_rig):
        rig = bound_rig
        identity_pair(rig.client)
        rig.client.policy_start(self.CONFIG)
        rig.client.sil_inject("SET_HAND_POSE", arg=2, values=[0.2, 0, 0, 0, 0, 0, 0.5])
        rig.wait(lambda: abs(rig.arm("left_leader")["position"][0] - 0.2) < 0.03, what="leader")
        rig.client.intervention_take_over("align", speed=1.0)
        rig.wait(lambda: rig.status()["pairs"][0]["phase"] == "ALIGNED", timeout=10, what="aligned")
        rig.client.call("intervention.confirm_aligned")
        rig.wait(lambda: rig.status()["pairs"][0]["phase"] == "FOLLOWING", what="following")

    def test_takeover_without_a_policy_is_refused(self, bound_rig):
        with pytest.raises(ServiceError) as exc:
            bound_rig.client.intervention_take_over()
        assert exc.value.code == "intervention.no_policy"

    def test_wrong_action_width_is_refused(self, bound_rig):
        from rac_link import host_now_us

        token = bound_rig.client.policy_start(self.CONFIG)["token"]
        with pytest.raises(ServiceError) as exc:
            bound_rig.client.policy_submit([[0.0] * 14], token, host_now_us(), 1.0)
        assert exc.value.code == "policy.bad_shape"


def test_a_partial_mapping_update_still_reads_back_complete(bound_rig):
    """The mapping page reads these fields for every joint and breaks without them."""
    bound_rig.client.teleop_configure([{"pair": 0, "enabled": True, "scale": [1.0] * 7, "offset": [0.0] * 7}])
    pair = bound_rig.client.teleop_config()["pairs"]["0"]
    assert pair["source_joint"] == list(range(7))
    assert pair["gripper_link"] is True
    assert "preset" not in pair
