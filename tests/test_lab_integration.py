"""The Lab's HTTP surface over the real service and firmware core, and the LeRobot proxies."""

from __future__ import annotations

import os
import threading
import time

import numpy as np
import pytest

os.environ.pop("MAKERMODSLAB_NO_CONTROLLER", None)


@pytest.fixture
def lab(rig, monkeypatch):
    """The FastAPI app talking to this test's service."""
    from fastapi.testclient import TestClient
    from makermodslab import controller_routes
    from makermodslab.server import app

    controller_routes.bridge.__init__()
    controller_routes.bridge.client = rig.client
    rig.client.on_status(controller_routes.bridge._on_status)
    rig.client.on_event(controller_routes.bridge._on_event)
    rig.client.on_state(controller_routes.bridge._on_state)
    client = TestClient(app)
    client.rig = rig
    yield client
    controller_routes.bridge.__init__()


def post(lab, path, **body):
    response = lab.post(f"/api/v1/controller{path}", json=body)
    assert response.status_code == 200, response.text
    return response.json()


def get(lab, path):
    response = lab.get(f"/api/v1/controller{path}")
    assert response.status_code == 200, response.text
    return response.json()


class TestArmFamily:
    def test_the_controller_family_is_registered_as_an_extension(self):
        from makermodslab.arms import registry
        from makermodslab.arms.controller import CONTROLLER
        from makermodslab.server import app  # noqa: F401, importing installs it

        assert registry.get("controller") is CONTROLLER
        assert "so101" in registry.ids()

    def test_the_manifest_describes_it_for_the_frontend(self):
        from makermodslab.arms.manifest import arms_manifest
        from makermodslab.server import app  # noqa: F401

        entry = next(f for f in arms_manifest() if f["id"] == "controller")
        assert entry["joints_per_arm"] == 7
        assert entry["calibration"]["kind"] == "panel"
        assert entry["calibration"]["panel_url"].startswith("/controller")

    def test_sessions_get_service_proxies_not_a_bus(self):
        from makermodslab.arms.controller import CONTROLLER

        robot, teleop = CONTROLLER.build_single_configs(None, {"env": {}}, "l", "f")
        assert robot.type == "rac_follower"
        assert robot.shared_cameras == ["env"]
        assert teleop.type == "rac_leader"
        robot, _ = CONTROLLER.build_bimanual_configs(None, {}, "b", "ls", "fs")
        assert robot.arms == ["left_follower", "right_follower"]


class TestRoutes:
    def test_full_operator_flow(self, lab, tmp_path):
        rig = lab.rig

        # 1.1 connect, discover, bind
        status = post(lab, "/connect", uri=rig.uri)
        assert status["connected"] and status["simulated"]
        ports = post(lab, "/discover")["ports"]
        assert ports[0]["detected_model"] == "REBOT_B601_RS"
        post(
            lab,
            "/bind",
            bindings=[
                {"port": 0, "arm": "left_follower", "model": "rebot_b601_rs", "serial": "rs-1"},
                {"port": 2, "arm": "left_leader", "model": "star_102_hd", "serial": "hd-1"},
            ],
        )
        rig.wait_valid("left_follower", "left_leader")
        assert len(get(lab, "/models")) == 4

        # 1.2 calibration through the routes
        state = post(lab, "/calibration/capture-zero", arm="left_leader", joints=[0])
        assert "zero" in state["draft"]["steps_done"]
        assert post(lab, "/calibration/save", arm="left_leader", note="t")["draft"]["version"] == 1
        assert get(lab, "/calibration/list")[0]["serial"] == "hd-1"

        # 1.3 mapping and following
        config = get(lab, "/teleop/config")
        assert config["pairs"]["0"]["preset"] is True
        post(lab, "/teleop/config", pairs=[{"pair": 0, "enabled": True, "scale": [1.0] * 7, "offset": [0.0] * 7}])
        post(lab, "/teleop/enter")
        post(lab, "/teleop/start", follow="relative")
        rig.wait(lambda: get(lab, "/status")["pairs"][0]["phase"] == "FOLLOWING", what="following")

        # A refusal has a code the page can read, and a status that says why.
        response = lab.post("/api/v1/controller/motion/home", json={})
        assert response.status_code == 409
        assert response.json()["detail"]["code"] == "control.busy"

        post(lab, "/teleop/end")
        assert get(lab, "/status")["source"] == "none"

        # 1.4 kinematics, and a program from teach points
        fk = post(lab, "/kinematics/fk", model="rebot_b601_rs", q=[0.2, 1.0, 1.0, 0.1, 0.1, 0.1])
        ik = post(lab, "/kinematics/ik", model="rebot_b601_rs", xyz=fk["xyz"], rpy=fk["rpy"], seed=[0.25, 0.95, 1.05, 0.1, 0.1, 0.1])
        assert ik["converged"]
        assert np.allclose(ik["q"], [0.2, 1.0, 1.0, 0.1, 0.1, 0.1], atol=0.02)

        program = {
            "name": "demo",
            "arm": "left_follower",
            "model": "rebot_b601_rs",
            "points": [{"name": "a", "q": [0.3, 0.2, 0.2, 0, 0, 0, 0.5], "move_time_s": 0.5, "dwell_s": 0}],
        }
        post(lab, "/programs", **program)
        assert get(lab, "/programs")[0]["name"] == "demo"
        assert get(lab, "/programs/demo")["points"][0]["name"] == "a"
        preview = post(lab, "/programs/preview", program=program, start=[0.0] * 7)
        assert preview["points"][-1][0] == pytest.approx(0.3, abs=1e-3)
        post(lab, "/programs/execute", program=program)
        rig.wait(lambda: get(lab, "/status")["source"] == "none", timeout=15, what="program finished")
        assert rig.arm("left_follower")["position"][0] == pytest.approx(0.3, abs=0.05)

        # 1.8 run control
        post(lab, "/motion/home")
        assert get(lab, "/status")["source"] == "home"
        post(lab, "/run/stop", kind="hold")
        assert get(lab, "/status")["source"] == "none"

        # Live state reaches the browser over the WebSocket.
        with lab.websocket_connect("/api/v1/controller/ws") as socket:
            seen_status = seen_samples = False
            for _ in range(60):
                message = socket.receive_json()
                seen_samples = seen_samples or "left_follower" in message.get("samples", {})
                seen_status = seen_status or "status" in message
                if seen_samples and seen_status:
                    break
            assert seen_samples and seen_status

    def test_a_bad_program_name_cannot_escape_the_directory(self, lab):
        assert lab.get("/api/v1/controller/programs/..%2F..%2Fsecret").status_code in (400, 404)
        response = lab.post("/api/v1/controller/programs", json={"name": "../evil", "arm": "left_follower", "points": []})
        assert response.status_code == 400

    def test_remote_clients_cannot_reach_controller_routes(self, lab):
        from fastapi.testclient import TestClient
        from makermodslab.server import app

        remote = TestClient(app, client=("192.168.1.50", 50000))
        response = remote.get("/api/v1/controller/status")
        assert response.status_code == 403
        assert response.json()["detail"]["code"] == "auth.local_only"

    def test_video_paths_stay_inside_the_dataset(self, lab, tmp_path):
        (tmp_path / "videos").mkdir()
        (tmp_path / "secret.mp4").write_bytes(b"x")
        response = lab.get("/api/v1/controller/data/video", params={"root": str(tmp_path), "path": "../secret.mp4"})
        assert response.status_code == 404

    def test_robot_descriptions_are_served(self, lab):
        assert lab.get("/robots/rebot_b601_rs/urdf/ReBot_Arm_RS.urdf").status_code == 200
        assert lab.get("/robots/star_arm_102/urdf/star_arm_102.urdf").status_code == 200


class TestLeRobotProxies:
    def test_robot_and_teleoperator_go_through_the_service(self, bound_rig, monkeypatch):
        from lerobot_robot_rac import RacFollower, RacFollowerConfig
        from lerobot_teleoperator_rac import RacLeader, RacLeaderConfig

        rig = bound_rig
        rig.client.teleop_configure([{"pair": 0, "enabled": True, "scale": [1.0] * 7, "offset": [0.0] * 7}])

        # The proxies find the service through the endpoint file.
        from rac_link import ipc

        monkeypatch.setattr(ipc, "read_endpoint", lambda: {"port": rig.server.port, "token": rig.server.token})

        teleop = RacLeader(RacLeaderConfig(id="t", arms=["left_follower"], follow="relative"))
        robot = RacFollower(RacFollowerConfig(id="r", arms=["left_follower"], source="teach"))
        assert list(robot.action_features)[0] == "joint1.pos"
        assert len(robot.action_features) == 7

        robot.connect()
        rig.wait(lambda: robot._latest.get("left_follower"), what="first sample")
        observation = robot.get_observation()
        assert set(observation) == set(robot.action_features)

        # Teleoperation is the controller's own loop: the proxy only observes it.
        teleop.connect()
        assert rig.status()["source"] == "teleop"
        rig.client.sil_inject("SET_HAND_POSE", arg=2, values=[0.3, 0, 0, 0, 0, 0, 0])
        rig.wait(lambda: abs(rig.arm("left_follower")["position"][0] - 0.3) < 0.08, what="following")
        rig.wait(lambda: teleop._latest.get("left_follower"), what="teleop sample")
        action = teleop.get_action()
        assert action["joint1.pos"] == pytest.approx(0.3, abs=0.08), "reports what was dispatched"
        teleop.disconnect()
        assert rig.status()["source"] == "none"

        # send_action reaches the arm through the one execution entry.
        for _ in range(40):
            robot.send_action({key: 0.1 if key != "gripper.pos" else 0.5 for key in robot.action_features})
            time.sleep(0.02)
        rig.wait(lambda: abs(rig.arm("left_follower")["position"][0] - 0.1) < 0.05, what="send_action")
        assert rig.status()["source"] == "teach"
        robot.disconnect()

    def test_bimanual_key_order_is_left_then_right(self):
        from lerobot_robot_rac.robot import joint_keys

        keys = joint_keys(["left_follower", "right_follower"])
        assert keys[0] == "left_joint1.pos"
        assert keys[6] == "left_gripper.pos"
        assert keys[7] == "right_joint1.pos"
        assert len(keys) == 14


class TestReplayRoute:
    def test_replay_from_a_time_approaches_first(self, lab, sim_cameras_for_lab, tmp_path):
        """Records a real episode, then replays from its middle on the real execution path."""
        from rac_data.browse import episode_data

        rig = lab.rig
        post(lab, "/connect", uri=rig.uri)
        post(
            lab,
            "/bind",
            bindings=[
                {"port": 0, "arm": "left_follower", "model": "rebot_b601_rs", "serial": "rs-1"},
                {"port": 2, "arm": "left_leader", "model": "star_102_hd", "serial": "hd-1"},
            ],
        )
        rig.wait_valid("left_follower", "left_leader")

        # Record a program running: a known motion from 0.0 to 0.5 rad on joint 1.
        root = tmp_path / "ds"
        post(lab, "/record/start", root=str(root), repo_id="local/ds", task="wave", fps=30, cameras=sim_cameras_for_lab)
        rig.client.execute("left_follower", [[float(v), 0.2, 0.2, 0, 0, 0, 0.5] for v in np.linspace(0, 0.5, 100)], 0.02, "teach", blend_s=0.5)
        rig.wait(lambda: rig.status()["source"] == "none", timeout=20, what="motion recorded")
        post(lab, "/record/stop")
        post(lab, "/record/save", result="success", note="")
        rig.wait(lambda: get(lab, "/record/status")["writer"]["pending"] == 0, timeout=240, what="dataset written")
        jobs = get(lab, "/record/status")["writer"]["jobs"]
        assert jobs[-1]["state"] == "done", jobs[-1]

        data = episode_data(root, 0)
        action = np.array(data["action"])
        assert np.ptp(action[:, 0]) > 0.3

        # Browse side: datasets and episodes list through the routes.
        listed = lab.get("/api/v1/controller/data/datasets", params={"base": str(tmp_path)}).json()
        assert listed[0]["episodes"] == 1
        episodes = lab.get("/api/v1/controller/data/episodes", params={"root": str(root)}).json()
        assert episodes[0]["result"] == "success"
        assert "teach" in episodes[0]["sources"]
        video = lab.get(
            "/api/v1/controller/data/video", params={"root": str(root), "path": data["videos"][sim_cameras_for_lab[0]]["path"]}
        )
        assert video.status_code == 200 and video.headers["content-type"] == "video/mp4"

        # Annotation stays beside the samples.
        post(lab, "/data/annotate", root=str(root), episode=0, result="failure", note="edited")
        segment = post(lab, "/data/segments", root=str(root), episode=0, start_s=0.1, end_s=0.4, label="reach", kind="phase")
        assert segment["id"] == 1
        assert lab.get("/api/v1/controller/data/episodes", params={"root": str(root), "result": "failure"}).json()

        # Replay from the middle: the arm is moved far away first, then approaches.
        midpoint = len(action) / data["fps"] / 2
        rig.client.move_to("left_follower", [-0.4, 0.2, 0.2, 0, 0, 0, 0.5], 0.5)
        rig.wait(lambda: rig.status()["source"] == "none", timeout=10, what="moved away")
        rig.wait_valid("left_follower")
        start_q = rig.arm("left_follower")["position"][0]
        assert start_q < -0.3

        result = post(lab, "/replay/episode", root=str(root), episode=0, start_s=midpoint, speed=1.0, blend_s=1.0)
        assert result["points"] > 0
        rig.keep_samples = True
        rig.wait(lambda: rig.status()["source"] == "none", timeout=20, what="replay finished")
        rig.keep_samples = False
        samples = [s["position"][0] for s in rig.samples if s["arm"] == "left_follower"]
        assert rig.arm("left_follower")["position"][0] == pytest.approx(action[-1, 0], abs=0.08)
        # No sample jumped: the approach is smooth, however far away the arm started.
        steps = np.abs(np.diff(samples))
        assert steps.max() < 0.05, f"largest step {steps.max()}"


@pytest.fixture
def sim_cameras_for_lab():
    from rac_camera.capture import CameraThread

    stop = threading.Event()
    thread = CameraThread("lab_env", {"source": "sim", "width": 320, "height": 240, "fps": 30}, stop)
    thread.start()
    time.sleep(0.3)
    yield ["lab_env"]
    stop.set()
    thread.join(timeout=3)
