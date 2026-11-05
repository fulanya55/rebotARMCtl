import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
from rac_kinematics import (
    ARM_JOINTS,
    TeachPoint,
    TeachProgram,
    advance_jerk_limited_reference_over_elapsed,
    assets_root,
    blend,
    gripper_assist_torque,
    load_chain,
    min_duration,
    retime,
    window,
)
from rac_kinematics.chain import rpy_matrix
from rac_link.robots import REBOT_RS, STAR_FL, STAR_HD, STAR_LD, default_mapping

MEASURED_SWEEPS = Path(__file__).parent / "data" / "rs_gravity_sweeps.json"


@pytest.fixture(scope="module", params=[REBOT_RS, STAR_FL], ids=["rs", "star"])
def arm(request):
    spec = request.param
    return spec, load_chain(spec.urdf, spec.end_effector)


def mid_pose(spec):
    return (np.array(spec.lower[:6]) + np.array(spec.upper[:6])) / 2.0


def test_chain_has_six_arm_joints(arm):
    _, chain = arm
    assert chain.dof == 6
    assert [j.name for j in chain.active] == ARM_JOINTS


def test_jacobian_matches_finite_difference(arm):
    spec, chain = arm
    q = mid_pose(spec) + 0.1
    jac = chain.jacobian(q)
    eps = 1e-6
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = eps
        numeric = (chain.fk(q + dq)[:3, 3] - chain.fk(q - dq)[:3, 3]) / (2 * eps)
        assert np.allclose(jac[:3, i], numeric, atol=1e-6), f"joint {i}"


def test_ik_recovers_reachable_pose(arm):
    spec, chain = arm
    rng = np.random.default_rng(3)
    solved = 0
    for _ in range(20):
        lo, hi = np.array(spec.lower[:6]), np.array(spec.upper[:6])
        q_goal = lo + (hi - lo) * rng.uniform(0.2, 0.8, 6)
        target = chain.fk(q_goal)
        # Start close by, as a drag does: each solve continues from the last pose.
        result = chain.solve_ik(target, q_goal + rng.normal(0, 0.08, 6))
        if result.converged:
            solved += 1
            achieved = chain.fk(result.q)
            assert np.linalg.norm(achieved[:3, 3] - target[:3, 3]) < 1e-3
            assert np.all(result.q >= chain.lower - 1e-9) and np.all(result.q <= chain.upper + 1e-9)
    assert solved >= 18


def test_ik_unreachable_returns_best_effort_inside_limits(arm):
    spec, chain = arm
    q0 = mid_pose(spec)
    target = chain.fk(q0)
    target[:3, 3] += np.array([5.0, 0.0, 0.0])
    result = chain.solve_ik(target, q0, max_iterations=60)
    assert not result.converged
    assert np.all(result.q >= chain.lower - 1e-9) and np.all(result.q <= chain.upper + 1e-9)


def test_position_only_ik(arm):
    spec, chain = arm
    q_goal = mid_pose(spec) + 0.2
    target = chain.fk(q_goal)
    result = chain.solve_ik(target, mid_pose(spec), position_only=True)
    assert result.converged
    assert np.linalg.norm(chain.fk(result.q)[:3, 3] - target[:3, 3]) < 1e-3


def test_gravity_torque_matches_virtual_work():
    """The balancing torque must equal the derivative of potential energy at each joint."""
    chain = load_chain(REBOT_RS.urdf, REBOT_RS.end_effector)
    q = np.array([0.3, 0.8, 1.0, 0.2, -0.3, 0.1])

    def potential(qv):
        total = 0.0
        pose_stack = [(chain.root, np.eye(4))]
        while pose_stack:
            link_name, pose = pose_stack.pop()
            link = chain.links.get(link_name)
            if link and link.mass > 0:
                com = (pose @ np.append(link.com, 1.0))[:3]
                total += link.mass * 9.81 * com[2]
            for joint in chain._children_of.get(link_name, ()):
                pose_stack.append((joint.child, pose @ chain._joint_transform(joint, qv)))
        return total

    eps = 1e-5
    numeric = np.zeros(6)
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = eps
        numeric[i] = (potential(q + dq) - potential(q - dq)) / (2 * eps)
    assert np.allclose(chain.gravity_torque(q), numeric, atol=1e-5)


def test_rs_gravity_magnitude_is_plausible():
    chain = load_chain(REBOT_RS.urdf, REBOT_RS.end_effector)
    # Arm held out horizontally: the shoulder carries the most, within what an RS06 can do.
    tau = chain.gravity_torque(np.array([0.0, 1.57, 0.0, 0.0, 0.0, 0.0]))
    assert 1.0 < abs(tau[1]) < 36.0
    assert abs(tau[0]) < 1e-6  # the base axis is vertical, so gravity cannot turn it


def test_blend_ends_exactly_and_is_smooth():
    points = blend([0.0, 0.0], [1.0, -2.0], 1.0, 0.01)
    assert np.allclose(points[-1], [1.0, -2.0])
    assert len(points) == 100
    assert abs(points[0, 0]) < 0.01
    # Peak speed of a min-jerk move is 1.875 times the mean.
    assert np.max(np.abs(np.diff(points[:, 0]) / 0.01)) == pytest.approx(1.875, abs=0.02)


def test_min_duration_respects_velocity_limit():
    limits = [1.0, 1.0]
    duration = min_duration([0, 0], [2.0, 0.5], limits)
    points = blend([0, 0], [2.0, 0.5], duration, 0.002)
    assert np.max(np.abs(np.diff(points, axis=0) / 0.002)) <= 1.0 + 1e-3


def test_teach_program_stretches_fast_segments():
    program = TeachProgram(
        name="demo",
        arm="left_follower",
        points=[TeachPoint("a", [1.0, 0.0], move_time_s=0.01), TeachPoint("b", [1.0, 1.0], move_time_s=1.0, dwell_s=0.5)],
    )
    limits = [1.0, 1.0]
    points = program.sample([0.0, 0.0], 0.01, velocity_limit=limits)
    assert np.allclose(points[-1], [1.0, 1.0])
    speed = np.max(np.abs(np.diff(np.vstack([[0.0, 0.0], points]), axis=0)) / 0.01)
    assert speed <= 1.0 + 0.02
    roundtrip = TeachProgram.from_dict(program.to_dict())
    assert roundtrip.points[1].dwell_s == 0.5


def test_retime_and_window():
    source = np.linspace(0, 1, 31)[:, None]  # one second at 30 Hz
    twice = retime(source, 1 / 30, 0.01, speed=2.0)
    assert len(twice) == pytest.approx(51, abs=1)
    assert twice[-1, 0] == pytest.approx(1.0, abs=0.02)
    clip = window(source, 1 / 30, 0.5, 0.8)
    assert clip[0, 0] == pytest.approx(0.5)
    assert clip[-1, 0] == pytest.approx(0.8, abs=0.02)


def world_axes(spec) -> np.ndarray:
    """Each arm joint's axis in the base frame, with every joint at zero."""
    root = ET.parse(assets_root() / spec.urdf).getroot()
    joints = {j.get("name"): j for j in root.findall("joint")}
    by_child = {j.find("child").get("link"): j for j in joints.values()}
    axes = []
    for name in ARM_JOINTS:
        chain = [joints[name]]
        while chain[-1].find("parent").get("link") in by_child:
            chain.append(by_child[chain[-1].find("parent").get("link")])
        rotation = np.eye(3)
        for joint in reversed(chain):
            rotation = rotation @ rpy_matrix(*map(float, joint.find("origin").get("rpy").split()))
        axes.append(rotation @ np.array(list(map(float, joints[name].find("axis").get("xyz").split()))))
    return np.array(axes)


def test_star_to_rebot_mapping_agrees_with_the_real_models():
    """Seeed's published directions, checked against geometry from two other vendors."""
    mapping = default_mapping(STAR_LD, REBOT_RS)
    assert mapping["scale"][:6] == [1.0, 1.0, -1.0, -1.0, -1.0, 1.0]
    assert mapping["offset"] == [0.0] * 7
    agreement = np.sum(world_axes(STAR_LD) * world_axes(REBOT_RS), axis=1)
    assert np.allclose(agreement, mapping["scale"][:6], atol=1e-3)


def test_mapping_is_the_same_in_both_directions():
    assert default_mapping(REBOT_RS, STAR_FL)["scale"] == default_mapping(STAR_FL, REBOT_RS)["scale"]


def test_star_arms_copy_each_other():
    """Fashion Star's control script sends raw leader angles to FL unchanged."""
    assert default_mapping(STAR_HD, STAR_FL)["scale"] == [1.0] * 7
    assert np.allclose(world_axes(STAR_FL), world_axes(STAR_LD), atol=1e-3)


def test_star_leaders_read_the_range_they_really_travel():
    assert STAR_LD.upper[1] > 2.9  # the published model stops at a quarter turn
    assert STAR_LD.lower[2] < -3.4
    assert STAR_HD.lower == STAR_LD.lower and STAR_HD.upper == STAR_LD.upper


def test_star_models_carry_their_meshes():
    for spec in (STAR_FL, STAR_LD):
        path = assets_root() / spec.urdf
        root = ET.parse(path).getroot()
        meshes = [m.get("filename") for m in root.iter("mesh")]
        assert len(meshes) >= 9
        for mesh in meshes:
            assert (path.parent / mesh).is_file(), mesh


@pytest.mark.parametrize("joint, index", [("joint2", 1), ("joint3", 2), ("joint4", 3)])
def test_gravity_torque_matches_a_real_arm(joint, index):
    """Holding torque measured on a real B601-RS, against this code's URDF gravity."""
    sweep = json.loads(MEASURED_SWEEPS.read_text(encoding="utf-8"))["sweeps"][joint]
    chain = load_chain(REBOT_RS.urdf, REBOT_RS.end_effector)
    predicted = np.array([chain.gravity_torque(np.array(s["q6"]))[index] for s in sweep])
    measured = np.array([s["tau"] for s in sweep])
    direction = np.array([s["dir"] for s in sweep], dtype=float)
    # Measured torque is gravity, plus friction that opposes the sweep, plus a bias.
    design = np.column_stack([predicted, direction, np.ones_like(predicted)])
    (scale, _friction, _bias), *_ = np.linalg.lstsq(design, measured, rcond=None)
    assert 0.85 < scale < 1.1  # right sign and right size
    assert np.corrcoef(predicted, measured)[0, 1] > 0.8


def test_live_target_profile_stays_within_its_limits():
    n = 7
    position, velocity, acceleration = np.zeros(n), np.zeros(n), np.zeros(n)
    goal, limit = np.full(n, 1.0), np.full(n, 1.5)
    dt, previous = 0.02, np.zeros(n)
    for step in range(400):
        if step == 40:
            goal = np.full(n, -0.5)  # the drag changes direction while moving
        position, velocity, acceleration = advance_jerk_limited_reference_over_elapsed(
            position, velocity, acceleration, goal, limit, dt
        )
        assert np.all(np.abs(velocity) <= 1.5 + 1e-9)
        assert np.all(np.abs(acceleration) <= 4.0 + 1e-9)
        assert np.all(np.abs(acceleration - previous) <= 30.0 * dt + 1e-9)
        previous = acceleration
    assert np.array_equal(position, goal) and not velocity.any()


def test_a_late_tick_keeps_real_time():
    args = (np.zeros(1), np.zeros(1), np.zeros(1), np.ones(1), np.array([1.5]))
    late = advance_jerk_limited_reference_over_elapsed(*args, 0.04)[0]
    on_time = advance_jerk_limited_reference_over_elapsed(*args, 0.02)[0]
    assert late > on_time


def test_gripper_assist_never_pushes_on_its_own():
    assist = lambda position, velocity: gripper_assist_torque(position, velocity, 0.0, 5.0)  # noqa: E731
    assert assist(2.5, 0.0) == 0.0  # at rest
    assert assist(2.5, 0.01) == 0.0  # below the threshold
    assert assist(2.5, 0.9) == 0.0  # too fast
    assert assist(4.95, 0.2) == 0.0  # opening at the open end
    assert assist(0.05, -0.2) == 0.0  # closing at the closed end
    assert assist(2.5, 0.2) > 0.0 > assist(2.5, -0.2)  # helps the way it is moving
    assert abs(assist(2.5, 0.5)) <= 0.04
