# Copyright 2026 fulanya.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""The controller arm type: arms behind the STM32 sync controller.

Unlike the other families this one owns no bus. Sessions build LeRobot proxies
that reach the arms through the Controller Service, which is the only holder of
the controller's USB link, so the Lab's existing flows (teleoperate, record,
replay, inference) run through the same execution entry as the /controller pages.
Ports, calibration and pairing live in the service: the record's port fields are
only labels, and calibration opens the controller page instead of a step wizard.
"""

from __future__ import annotations

from typing import Any

from .base import ArmFamily, LeaderOption

CONTROLLER_LEADER_KIND = "controller"


class ControllerFamily(ArmFamily):
    id = "controller"
    label = "Sync Controller (reBot RS / Star 102)"
    short_label = "Controller"
    indefinite_label = "a controller arm"

    joints_per_arm = 7
    supports_bimanual = True
    uses_feetech_bus = False
    supports_auto_calibration = False
    calibration_kind = "panel"
    calibration_panel_url = "/controller?tab=calibration"
    supports_dagger = False

    single_robot_type = "rac_follower"
    bimanual_robot_type = "bi_rac_follower"
    robot_type_markers = ("rac_",)

    follower_probe_protocol = None
    motion_identify_energizes_follower = False
    telemetry_kind = "degrees"
    default_leader_kind = CONTROLLER_LEADER_KIND

    def leader_options(self) -> tuple[LeaderOption, ...]:
        return (LeaderOption(id=CONTROLLER_LEADER_KIND, label="Leader on the sync controller"),)

    def leader_calibration_dir(self, leader_kind: str | None = None) -> str:
        # The registry requires an answer, but no calibration file is ever written
        # here: the service owns calibration, versioned per arm serial.
        from rac_link.ipc import runtime_dir

        return str(runtime_dir() / "lab_calibration" / "leader")

    def follower_calibration_dir(self) -> str:
        from rac_link.ipc import runtime_dir

        return str(runtime_dir() / "lab_calibration" / "follower")

    def robot_cli_type(self, bimanual: bool) -> str:
        # Both cases are one proxy class: its `arms` list says how many arms it fronts.
        return "rac_follower"

    def single_follower_config(self, port: str, config_id: str):
        from lerobot_robot_rac import RacFollowerConfig

        return RacFollowerConfig(id=config_id, arms=["left_follower"])

    def single_leader_config(self, port: str, config_id: str, leader_kind: str | None = None):
        from lerobot_teleoperator_rac import RacLeaderConfig

        return RacLeaderConfig(id=config_id, arms=["left_follower"])

    def build_single_configs(self, request: Any, cameras: dict | None, leader_id: str, follower_id: str):
        from lerobot_robot_rac import RacFollowerConfig
        from lerobot_teleoperator_rac import RacLeaderConfig

        names = list((cameras or {}).keys())
        robot = RacFollowerConfig(id=follower_id, arms=["left_follower"], shared_cameras=names, source="teleop")
        teleop = RacLeaderConfig(id=leader_id, arms=["left_follower"])
        return robot, teleop

    def build_bimanual_configs(
        self,
        request: Any,
        cameras: dict | None,
        base: str,
        leader_staging: str,
        follower_staging: str,
        *args: Any,
        **kwargs: Any,
    ):
        from lerobot_robot_rac import RacFollowerConfig
        from lerobot_teleoperator_rac import RacLeaderConfig

        names = list((cameras or {}).keys())
        arms = ["left_follower", "right_follower"]
        robot = RacFollowerConfig(id=f"{base}_follower", arms=arms, shared_cameras=names, source="teleop")
        teleop = RacLeaderConfig(id=f"{base}_leader", arms=arms)
        return robot, teleop

    def preflight_ports(self, *args: Any, **kwargs: Any) -> list:
        # The service validates bindings; a record has no ports of its own to check.
        return []

    def capture_rest_poses(self, robot: Any, *, include_gripper: bool = False) -> list[tuple[Any, dict]]:
        return []

    def capture_leader_rest_poses(self, teleop: Any) -> list[tuple[Any, dict]]:
        return []

    def return_to_rest(self, *args: Any, **kwargs: Any) -> list[str]:
        # The service keeps the arm held on disconnect, so nothing falls here.
        return []

    def release_torque(self, device: Any, label: str = "device") -> list[str]:
        # Releasing a brakeless arm is an explicit request, never a side effect.
        return []

    def verify_identity(self, pairs: Any, *, skip: bool = False, config_names: Any = None) -> list:
        return []

    def prepare_follower_registers(self, robot: Any, label: str | None = None) -> list[str]:
        return []

    async def identify_by_motion(self, *args: Any, **kwargs: Any):
        raise NotImplementedError("the controller reports which arm is on which port by itself")


CONTROLLER = ControllerFamily()
