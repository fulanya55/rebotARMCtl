import numpy as np
import pytest
from rac_kinematics import ARM_JOINTS, TeachPoint, TeachProgram, blend, load_chain, min_duration, retime, window
from rac_link.robots import REBOT_RS, STAR_FL


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
