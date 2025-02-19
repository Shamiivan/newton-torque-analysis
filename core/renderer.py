from typing import Dict, Any, Set, Optional
from enum import Enum, auto
import numpy as np
from pybullet_examples.getClosestPoints import lineWidth

from .robot import Robot
import pybullet as p

class VisualisationType(Enum):
    JOINT_AXES = auto()
    CENTER_OF_MASS = auto()


class Renderer:
    """ """
    def __init__(
            self,
            robot: Robot,
            physics_client: int,
            update_frequency: int = 240
    ):
        self.physics_client: int = physics_client
        self.robot = robot
        self.rendered_ids: Dict[str, Any] = {}
        self.rendered_items: Set[VisualisationType] = set()

        self.step_counter: int = 0
        self.update_frequency =  update_frequency# update every 100 step
        self.prev_com_position: Optional[np.ndarray] = None
        self.reset()

    def reset(self) -> None:
        """Reset all visualizations"""
        for item in self.rendered_ids.values():
            p.removeUserDebugItem(item, physicsClientId=self.physics_client)
        self.rendered_ids = {"com": []}
        POINT_HEIGHT = 0.02
        com_line_id = p.addUserDebugLine(
            [0, 0, 0],
            [0, 0, POINT_HEIGHT],
            [1, 0, 0], # red
            lineWidth=3,
            physicsClientId=self.physics_client
        )
        self.rendered_ids["com"].append((com_line_id,"point"))

        # gravity vector
        gravity_line_id = p.addUserDebugLine(
            [0, 0, 0],
            [0, 0, -POINT_HEIGHT * 9,81],
            [1, 0, 0], # blue
            lineWidth=3,
            physicsClientId=self.physics_client
        )
        self.rendered_ids["com"].append((gravity_line_id, "gravity"))



    def update_com(self, position: np.ndarray )->None:
        """Visualize center of mass of robot"""
        if self.prev_com_position is None:
            return

        position = np.array(position)

        # grab id of the com line and gravity line
        for id, type in self.rendered_ids["com"]:
            if type == "point":
                p.addUserDebugLine(
                    position,
                    [position[0], position[1], position[2] + 0.02],
                    [1, 0, 0],
                    lineWidth=3,
                    replaceItemUniqueId=id[0],
                    physicsClientId=self.physics_client
                )
            elif type == "gravity":
                p.addUserDebugLine(
                    position,
                    [position[0], position[1], position[2]* 0.02 - 9.81],
                    [1, 1, 0],
                    lineWidth=3,
                    replaceItemUniqueId=id[0],
                    physicsClientId=self.physics_client
                )
            p.addUserDebugLine(
                self.prev_com_position,
                position,
                [1, 0, 0],
                lineWidth=3,
                replaceItemUniqueId=id[0],
                physicsClientId=self.physics_client
            )
        self.prev_com_position = position


        # Remove old visualization if it exists
    def update(self) -> None:
        """Update all active visualizations"""
        self.step_counter += 1
        if self.step_counter % self.update_frequency != 0:
            return
        #
        print("Updating visualizations", self.step_counter)
        com_position = self.robot.get_com()
        # self.update_com(com_position)