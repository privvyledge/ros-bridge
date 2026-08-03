#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla vehicles
"""

import carla_common.transforms as trans

from carla_ros_bridge.traffic_participant import TrafficParticipant

from derived_object_msgs.msg import Object
from std_msgs.msg import ColorRGBA


class Vehicle(TrafficParticipant):

    """
    Actor implementation details for vehicles
    """

    def __init__(self, uid, name, parent, node, carla_actor):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param carla_actor: carla vehicle actor object
        :type carla_actor: carla.Vehicle
        """
        self.classification = Object.CLASSIFICATION_CAR
        if 'object_type' in carla_actor.attributes:
            if carla_actor.attributes['object_type'] == 'car':
                self.classification = Object.CLASSIFICATION_CAR
            elif carla_actor.attributes['object_type'] == 'bike':
                self.classification = Object.CLASSIFICATION_BIKE
            elif carla_actor.attributes['object_type'] == 'motorcycle':
                self.classification = Object.CLASSIFICATION_MOTORCYCLE
            elif carla_actor.attributes['object_type'] == 'truck':
                self.classification = Object.CLASSIFICATION_TRUCK
            elif carla_actor.attributes['object_type'] == 'other':
                self.classification = Object.CLASSIFICATION_OTHER_VEHICLE
            else:
                # object_type seems to be empty all the time, so try to identify at least some of them
                # source: https://github.com/carla-simulator/ros-bridge/pull/641
                if 'number_of_wheels' in carla_actor.attributes and \
                   carla_actor.attributes['number_of_wheels'] == '2':
                    motorcycle_keys = [ "yamaha", "kawasaki", "harley-davidson", "vespa" ]
                    bicycle_keys = [ "crossbike", "gazelle", "diamondback" ]
                    if any(x in carla_actor.type_id for x in motorcycle_keys):
                      self.classification = Object.CLASSIFICATION_MOTORCYCLE
                    elif any(x in carla_actor.type_id for x in bicycle_keys):
                      self.classification = Object.CLASSIFICATION_BIKE
                else:
                    truck_keys = [ "sprinter", "ambulance", "firetruck", "carlacola" ]
                    if any(x in carla_actor.type_id for x in truck_keys):
                      self.classification = Object.CLASSIFICATION_TRUCK

        super(Vehicle, self).__init__(uid=uid,
                                      name=name,
                                      parent=parent,
                                      node=node,
                                      carla_actor=carla_actor)

    def get_marker_color(self):  # pylint: disable=no-self-use
        """
        Function (override) to return the color for marker messages.

        :return: the color used by a vehicle marker
        :rtpye : std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 255.0
        color.g = 0.0
        color.b = 0.0
        return color

    def get_marker_pose(self):
        """
        Function to return the pose for vehicles.

        :return: the pose of the vehicle
        :rtype: geometry_msgs.msg.Pose
        """
        # Moving pivot point from the bottom (CARLA) to the center (ROS) of the
        # bounding box. Delegated so vehicles, markers and objects share ONE
        # definition of that pose -- this override predated get_object_info's
        # equivalent fix and the two silently disagreed, and it cannot express
        # the fallback box used for CARLA's zero-size two-wheeler boxes.
        return self.get_bounding_box_ros_pose()

    def get_classification(self):
        """
        Function (override) to get classification
        :return:
        """
        return self.classification
