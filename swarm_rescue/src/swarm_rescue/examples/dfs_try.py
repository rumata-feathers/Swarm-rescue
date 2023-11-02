"""
This program can be launched directly.
To move the drone, you have to click on the map, then use the arrows on the keyboard
"""
import math
import os
import random
import sys
from typing import Type

import cv2
import numpy as np

from spg.playground import Playground
from spg.utils.definitions import CollisionTypes

# This line add, to sys.path, the path to parent path of this file
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from spg_overlay.utils.grid import Grid
from spg_overlay.utils.pose import Pose
from maps.map_intermediate_01 import MyMapIntermediate01
from spg_overlay.entities.wounded_person import WoundedPerson
from spg_overlay.utils.constants import MAX_RANGE_LIDAR_SENSOR
from spg_overlay.utils.misc_data import MiscData
from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.entities.rescue_center import RescueCenter, wounded_rescue_center_collision
from spg_overlay.gui_map.closed_playground import ClosedPlayground
from spg_overlay.gui_map.gui_sr import GuiSR
from spg_overlay.gui_map.map_abstract import MapAbstract

EVERY_N = 1
LIDAR_DIST_CLIP = 40.0
EMPTY_ZONE_VALUE = -0.602
OBSTACLE_ZONE_VALUE = 2.0
FREE_ZONE_VALUE = -4.0
THRESHOLD_MIN = -40
THRESHOLD_MAX = 40


def conv_angle(angle):
    angle = angle % (2 * math.pi)
    angle -= math.pi
    return angle / math.pi


class OccupancyGrid(Grid):
    """Simple occupancy grid"""

    def __init__(self,
                 size_area_world,
                 resolution: float,
                 lidar):
        super().__init__(size_area_world=size_area_world, resolution=resolution)

        self.size_area_world = size_area_world
        self.resolution = resolution

        self.lidar = lidar

        self.x_max_grid: int = int(self.size_area_world[0] / self.resolution + 0.5)
        self.y_max_grid: int = int(self.size_area_world[1] / self.resolution + 0.5)

        self.grid = np.zeros((self.x_max_grid, self.y_max_grid))
        self.zoomed_grid = np.empty((self.x_max_grid, self.y_max_grid))

    def update_grid(self, pose: Pose):
        """
        Bayesian map update with new observation
        lidar : lidar data
        pose : corrected pose in world coordinates
        """
        # EVERY_N = 1
        # LIDAR_DIST_CLIP = 40.0
        # EMPTY_ZONE_VALUE = -0.602
        # OBSTACLE_ZONE_VALUE = 2.0
        # FREE_ZONE_VALUE = -4.0
        # THRESHOLD_MIN = -40
        # THRESHOLD_MAX = 40

        lidar_dist = self.lidar.get_sensor_values()[::EVERY_N].copy()
        lidar_angles = self.lidar.ray_angles[::EVERY_N].copy()

        # Compute cos and sin of the absolute angle of the lidar
        cos_rays = np.cos(lidar_angles + pose.orientation)
        sin_rays = np.sin(lidar_angles + pose.orientation)

        max_range = MAX_RANGE_LIDAR_SENSOR * 0.9

        # For empty zones
        # points_x and point_y contains the border of detected empty zone
        # We use a value a little bit less than LIDAR_DIST_CLIP because of the noise in lidar
        lidar_dist_empty = np.maximum(lidar_dist - LIDAR_DIST_CLIP, 0.0)
        # All values of lidar_dist_empty_clip are now <= max_range
        lidar_dist_empty_clip = np.minimum(lidar_dist_empty, max_range)
        points_x = pose.position[0] + np.multiply(lidar_dist_empty_clip, cos_rays)
        points_y = pose.position[1] + np.multiply(lidar_dist_empty_clip, sin_rays)

        for pt_x, pt_y in zip(points_x, points_y):
            self.add_value_along_line(pose.position[0], pose.position[1], pt_x, pt_y, EMPTY_ZONE_VALUE)

        # For obstacle zones, all values of lidar_dist are < max_range
        select_collision = lidar_dist < max_range

        points_x = pose.position[0] + np.multiply(lidar_dist, cos_rays)
        points_y = pose.position[1] + np.multiply(lidar_dist, sin_rays)

        points_x = points_x[select_collision]
        points_y = points_y[select_collision]
        # print(points_x, points_y)
        # print('\n----\n')

        self.add_points(points_x, points_y, OBSTACLE_ZONE_VALUE)
        for pt_x, pt_y in zip(points_x, points_y):
            self.add_value_along_line(pose.position[0], pose.position[1], pt_x, pt_y, EMPTY_ZONE_VALUE)

        # the current position of the drone is free !
        self.add_points(pose.position[0], pose.position[1], FREE_ZONE_VALUE)

        # threshold values
        self.grid = np.clip(self.grid, THRESHOLD_MIN, THRESHOLD_MAX)

        # compute zoomed grid for displaying
        self.zoomed_grid = self.grid.copy()
        new_zoomed_size = (int(self.size_area_world[1] * 0.5), int(self.size_area_world[0] * 0.5))
        self.zoomed_grid = cv2.resize(self.zoomed_grid, new_zoomed_size, interpolation=cv2.INTER_NEAREST)

    def display(self, grid_to_display: np.ndarray, robot_pose: Pose, title="grid", show=True):
        """
        Screen display of grid and robot pose,
        using opencv (faster than the matplotlib version)
        robot_pose : [x, y, theta] nparray, corrected robot pose
        """
        img = grid_to_display.T
        img = img - img.min()
        img = img / img.max() * 255
        img = np.uint8(img)
        img_color = cv2.applyColorMap(src=img, colormap=cv2.COLORMAP_JET)

        pt2_x = robot_pose.position[0] + np.cos(robot_pose.orientation) * 20
        pt2_y = robot_pose.position[1] + np.sin(robot_pose.orientation) * 20
        pt2_x, pt2_y = self._conv_world_to_grid(pt2_x, pt2_y)

        pt1_x, pt1_y = self._conv_world_to_grid(robot_pose.position[0], robot_pose.position[1])

        pt1 = (int(pt1_x), int(pt1_y))
        pt2 = (int(pt2_x), int(pt2_y))
        # cv2.arrowedLine(img=img_color, pt1=pt1, pt2=pt2,
        #     color=(0, 0, 255), thickness=2)
        cv2.circle(img=img_color, center=pt1, radius=2, color=(100, 200, 255))
        if show:
            cv2.imshow(title, img_color)
            cv2.waitKey(1)
        return img_color


class MyDroneMapping(DroneAbstract):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.iteration: int = 0

        self.estimated_pose = Pose()

        resolution = 5
        self.grid = OccupancyGrid(size_area_world=self.size_area,
            resolution=resolution,
            lidar=self.lidar())

    def define_message_for_all(self):
        """
        Here, we don't need communication...
        """
        pass

    def control(self):
        """
        We only send a command to do nothing
        """
        # select empty
        empty_cell = self.grid.grid.copy()
        empty_cell[empty_cell > FREE_ZONE_VALUE] = 40
        # now calculate centroid
        x_pos, y_pos = np.where(empty_cell < FREE_ZONE_VALUE)
        length = x_pos.size
        sum_x = np.sum(x_pos)
        sum_y = np.sum(y_pos)
        centroid_x, centroid_y = sum_x / length, sum_y / length
        if np.isnan(centroid_x) or np.isnan(centroid_y):
            centroid_x, centroid_y = self.measured_gps_position()
        # now calculate force
        pos_x, pos_y = self.measured_gps_position()
        diff = np.array([centroid_x - pos_x, centroid_y - pos_y])
        vec_length = np.sqrt(diff.dot(diff))
        command_angle = self.measured_compass_angle() - math.asin(diff[0] / vec_length)
        if np.isnan(command_angle):
            command_angle = 0
        if np.isnan(vec_length):
            vec_length = 0

        print(centroid_x, centroid_y, command_angle, conv_angle(command_angle), vec_length)

        command = { "forward": 1,
                    "lateral": 0.0,
                    "rotation": conv_angle(command_angle),
                    "grasper": 0.0
                    }

        # increment the iteration counter
        self.iteration += 1

        self.estimated_pose = Pose(np.asarray(self.measured_gps_position()), self.measured_compass_angle())
        # self.estimated_pose = Pose(np.asarray(self.true_position()), self.true_angle())
        self.grid.update_grid(pose=self.estimated_pose)
        if self.iteration % 10 == 0:
            # self.grid.display(self.grid.grid, self.estimated_pose, title="occupancy grid")
            # self.grid.display(self.grid.zoomed_grid, self.estimated_pose, title="zoomed occupancy grid")
            self.grid.display(empty_cell, self.estimated_pose, title="occupancy grid")
            canvas = self.grid.display(self.grid.grid, self.estimated_pose, title="my grid")
            cv2.circle(img=canvas, center=(int(centroid_x), int(centroid_y)), radius=2, color=(180, 100, 100))
            cv2.imshow("ff", canvas)
            cv2.waitKey(1)
            # pass

        return command


def main():
    my_map = MyMapIntermediate01()

    playground = my_map.construct_playground(drone_type=MyDroneMapping)

    # draw_lidar_rays : enable the visualization of the lidar rays
    # draw_semantic_rays : enable the visualization of the semantic rays
    gui = GuiSR(playground=playground,
        the_map=my_map,
        draw_lidar_rays=True,
        draw_semantic_rays=True
        # use_keyboard=True,
    )
    gui.run()


if __name__ == '__main__':
    main()
