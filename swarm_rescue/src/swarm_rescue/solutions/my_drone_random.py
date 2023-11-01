"""
Simple random controller
The Drone will move forward and turn for a random angle when an obstacle is hit
"""
import math
import random
import time
import statistics
from typing import Optional
import numpy as np
from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.utils.misc_data import MiscData
from spg_overlay.utils.utils import normalize_angle


class MyDroneRandom(DroneAbstract):
    def __init__(self,
                 identifier: Optional[int] = None,
                 misc_data: Optional[MiscData] = None,
                 **kwargs):
        super().__init__(identifier=identifier,
                         misc_data=misc_data,
                         display_lidar_graph=False,
                         **kwargs)
        self.counterStraight = 0
        self.angleStopTurning = random.uniform(-math.pi, math.pi)
        self.distStopStraight = random.uniform(10, 50)
        self.isTurning = False

    def define_message_for_all(self):
        """
        Here, we don't need communication...
        """
        pass





    def process_lidar_sensor(self, the_lidar_sensor):
        command = {"forward": 0.5,
                   "lateral": 0.0,
                   "rotation": 0.0}
        angular_vel_controller = 1.0

        values = the_lidar_sensor.get_sensor_values()

        if values is None:
            return command, False

        ray_angles = the_lidar_sensor.ray_angles
        size = the_lidar_sensor.resolution
        #print(size)
        #print(ray_angles)
        #time.sleep(100)
        far_angle_raw = 0
        near_angle_raw = 0
        min_dist = 1000
        if size != 0:
            # far_angle_raw : angle with the longer distance
            far_angle_raw = ray_angles[np.argmax(values)]
            #print(far_angle_raw)
            min_dist = min(values)
            #print(min_dist)
            #time.sleep(100)
            # near_angle_raw : angle with the nearest distance
            near_angle_raw = ray_angles[np.argmin(values)]
            #print(near_angle_raw)
            #time.sleep(100)

        far_angle = far_angle_raw
        # If far_angle_raw is small then far_angle = 0
        if abs(far_angle) < 1 / 180 * np.pi:
            far_angle = 0.0

        near_angle = near_angle_raw
        far_angle = normalize_angle(far_angle)
        """
        # The drone will turn toward the zone with the more space ahead
        if size != 0:
            if far_angle > 0.3:
                command["rotation"] = angular_vel_controller
            elif far_angle > -0.3 and far_angle < 0.3:
                command["rotation"] = 0
            else:
                command["rotation"] = angular_vel_controller
"""
        # If near a wall then 'collision' is True and the drone tries to turn its back to the wall
        collision = False
        if size != 0 and min_dist < 20:
            collision = True
            #if near_angle > 0:
            #    command["rotation"] = angular_vel_controller
            #else:
            #    command["rotation"] = angular_vel_controller
        #print(min_dist)


        wall_angle=near_angle_raw+self.measured_compass_angle()
        if wall_angle > np.pi:
            wall_angle = near_angle_raw - self.measured_compass_angle()
        if wall_angle < -np.pi:
            wall_angle = near_angle_raw - self.measured_compass_angle()

        #print(near_angle_raw, self.measured_compass_angle(),-np.pi/2 , wall_angle)
        if collision == False:
            command = {"forward": 0.4,
                        "lateral": 0.0,
                        "rotation": 0.0,
                        "grasper": 0.0}

        # see if there is corner and try to turn right
        #print(statistics.mean([values[37], values[43]]), statistics.mean([values[47], values[53]]))
        if (statistics.mean([values[47], values[53]])-statistics.mean([values[37], values[43]])>10):
            command = {"forward": 0.2,
                        "lateral": 0.0,
                        "rotation": -0.7,
                        "grasper": 0.0}


        if wall_angle > np.pi/2 - 0.4 and wall_angle < np.pi/2 + 0.4 and min_dist < 40:
            #print("up",near_angle_raw, self.measured_compass_angle() , wall_angle)
            print("sverhu",wall_angle)
            if not ((self.measured_compass_angle() > np.pi - 0.05 and self.measured_compass_angle() < np.pi) or (self.measured_compass_angle() < -np.pi + 0.2 and self.measured_compass_angle() > -np.pi)):
                #print("rotating_wall up")
                command = {"forward": 0.05,
                           "lateral": 0.0,
                           "rotation": 0.6,
                           "grasper": 0.0}
            else:
                collision=False
        if ((wall_angle > np.pi - 0.4 and wall_angle < np.pi) or (wall_angle < -np.pi + 0.4 and wall_angle > -np.pi))  and min_dist < 40:
            #print("left",near_angle_raw, self.measured_compass_angle() , wall_angle)
            print("sleva",wall_angle)
            if self.measured_compass_angle() > -np.pi/2 + 0.2 or self.measured_compass_angle() < -np.pi/2 - 0.05 :
                #print("rotating_wall left")
                command = {"forward": 0.05,
                           "lateral": 0.0,
                           "rotation": 0.6,
                           "grasper": 0.0}
            else:
                collision=False
        if wall_angle > -np.pi/2 - 0.4 and wall_angle < -np.pi/2 + 0.4 and min_dist < 40:
            print("vnizu",wall_angle)
            if self.measured_compass_angle() > 0.2 or self.measured_compass_angle() < -0.05:
                command = {"forward": 0.05,
                           "lateral": 0.0,
                           "rotation": 0.6,
                           "grasper": 0.0}
            else:
                collision=False


        """ THIS PART DOESNT WORK BECAUSE I CALCULATE WALL_ANGEL INCORRECT
        if wall_angle > - 0.4 and wall_angle < + 0.4 and min_dist < 40:
            print("sprava")
            if self.measured_compass_angle() > np.pi/2 + 0.2 or self.measured_compass_angle() < np.pi/2 - 0.05:
                command = {"forward": 0.05,
                           "lateral": 0.0,
                           "rotation": 0.6,
                           "grasper": 0.0}
            else:
                collision=False
                
        """
       
        if (statistics.mean([values[43], values[47]])<30):
            command = {"forward": 0.0,
                        "lateral": 0.5,
                        "rotation": 0.0,
                        "grasper": 0.0}

        return command, collision

    def control(self):
        """
        The Drone will move forward and turn for a random angle when an obstacle is hit
        """

        command, collided = self.process_lidar_sensor(self.lidar())
        # if collided == False:
        #     command = {"forward": 1.0,
        #                 "lateral": 0.0,
        #                 "rotation": 0.0,
        #                 "grasper": 0.0}
        # #else




        command_straight = {"forward": 1.0,
                            "lateral": 0.0,
                            "rotation": 0.0,
                            "grasper": 0}

        command_turn = {"forward": 0.0,
                        "lateral": 0.0,
                        "rotation": 0.1,
                        "grasper": 0}




        #self.counterStraight += 1

        #if collided and not self.isTurning and self.counterStraight > self.distStopStraight:
         #   self.isTurning = True
           # #self.angleStopTurning = random.uniform(-math.pi, math.pi)
          #  self.angleStopTurning = self.angleStopTurning + 0.3
        if self.measured_compass_angle() is not None:
            measured_angle = self.measured_compass_angle()
            #print(measured_angle)
        if collided==True:
            self.isTurning = True
            #self.angleStopTurning = random.uniform(-math.pi, math.pi)
            self.angleStopTurning = self.angleStopTurning + 0.1
        if collided==False:
            self.isTurning = False
        #print(collided)

        # measured_angle = 0
        # if self.measured_compass_angle() is not None:
        #     measured_angle = self.measured_compass_angle()
        #     print(measured_angle)
        # diff_angle = normalize_angle(self.angleStopTurning - measured_angle)
        # if self.isTurning and abs(diff_angle) < 0.2:
        #     self.isTurning = False
        #     self.counterStraight = 0
        #     #self.distStopStraight = random.uniform(10, 50)

        #if self.isTurning:
        #    return command
        #else:
        #    return command_straight
        #print(self.measured_compass_angle())
        #time.sleep(0.1)
        return command