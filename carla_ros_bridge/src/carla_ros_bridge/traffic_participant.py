#!/usr/bin/env python

#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla traffic participants
"""

import carla
import carla_common.transforms as trans

from carla_ros_bridge.actor import Actor

from derived_object_msgs.msg import Object
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


# CARLA reports an all-zero bounding box for two-wheelers, so they reach ROS with
# no size at all: invisible in RViz and impossible for a planner to avoid. There
# is nothing to read the real extent from, so substitute a conservative size per
# classification. Full extents in metres (length, width, height); heights include
# the rider.
FALLBACK_EXTENT = {
    Object.CLASSIFICATION_MOTORCYCLE: (2.20, 0.90, 1.60),
    Object.CLASSIFICATION_BIKE: (1.80, 0.70, 1.60),
}


class TrafficParticipant(Actor):

    """
    actor implementation details for traffic participant
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
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        """
        self.classification_age = 0
        super(TrafficParticipant, self).__init__(uid=uid,
                                                 name=name,
                                                 parent=parent,
                                                 node=node,
                                                 carla_actor=carla_actor)

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.

        On update vehicles send:
        - tf global frame
        - object message
        - marker message

        :return:
        """
        self.classification_age += 1
        super(TrafficParticipant, self).update(frame, timestamp)

    def get_shape_extent(self):
        """
        Function to provide the half-extents to publish for this participant.

        Returns CARLA's own bounding box extent, except when that box is
        degenerate (all zero, as CARLA reports for two-wheelers), in which case
        a conservative FALLBACK_EXTENT for the classification is used. Shared by
        the object and marker paths so the two cannot describe different boxes.

        :return: (x, y, z) half-extents in metres
        :rtype: tuple
        """
        extent = self.carla_actor.bounding_box.extent
        if extent.x > 0.0 or extent.y > 0.0 or extent.z > 0.0:
            return extent.x, extent.y, extent.z
        fallback = FALLBACK_EXTENT.get(self.get_classification())
        if fallback is None:
            return extent.x, extent.y, extent.z
        return fallback[0] / 2.0, fallback[1] / 2.0, fallback[2] / 2.0

    def get_bounding_box_ros_pose(self):
        """
        Function to provide the ROS pose of this participant's bounding box centre.

        CARLA anchors an actor at its origin -- for a vehicle, on the ground
        between the axles -- while the bounding box centre sits at
        bounding_box.location relative to it. That offset is in ACTOR
        coordinates, so it has to be rotated through the actor transform rather
        than added componentwise, or a yawed vehicle is displaced sideways as
        well as vertically.

        Both derived_object_msgs/Object and visualization_msgs/Marker place the
        shape AT the pose, so publishing the bare actor origin sinks every
        vehicle box by half its own height (0.69 m for an audi.tt, 1.91 m for
        the firetruck). Walkers are unaffected: CARLA already centres them, so
        their bounding_box.location is zero and this is a no-op for them.

        :return: the ROS pose of this actor's bounding box centre
        :rtype: geometry_msgs.msg.Pose
        """
        extent = self.carla_actor.bounding_box.extent
        if extent.x <= 0.0 and extent.y <= 0.0 and extent.z <= 0.0:
            # CARLA reports a zero-size bounding box for two-wheelers, and the
            # location of a box with no size is meaningless -- harley-davidson
            # reports a 1.19 m offset for one. Applying it would displace the
            # object by more than its own length, so fall back to the actor
            # origin, which is what this method used to return for everything.
            transform = self.carla_actor.get_transform()
            half_height = self.get_shape_extent()[2]
            if half_height > 0.0:
                # Lift the substituted box so it rests on the road rather than
                # being centred on the actor origin, which sits at ground level.
                transform.location += transform.get_up_vector() * half_height
            return trans.carla_transform_to_ros_pose(transform)

        transform = self.carla_actor.get_transform()
        centre = carla.Location(
            self.carla_actor.bounding_box.location.x,
            self.carla_actor.bounding_box.location.y,
            self.carla_actor.bounding_box.location.z)
        transform.transform(centre)   # actor-local -> world, in place
        pose = trans.carla_transform_to_ros_pose(transform)
        pose.position = trans.carla_location_to_ros_point(centre)
        return pose

    def get_object_info(self):
        """
        Function to send object messages of this traffic participant.

        A derived_object_msgs.msg.Object is prepared to be published via '/carla/objects'

        :return:
        """
        obj = Object(header=self.get_msg_header("map"))
        # ID
        obj.id = self.get_id()
        # Pose
        obj.pose = self.get_bounding_box_ros_pose()
        # Twist
        obj.twist = self.get_current_ros_twist()
        # Acceleration
        obj.accel = self.get_current_ros_accel()
        # Shape
        obj.shape.type = SolidPrimitive.BOX
        extent = self.get_shape_extent()
        obj.shape.dimensions.extend([
            extent[0] * 2.0, extent[1] * 2.0, extent[2] * 2.0])

        # Classification if available in attributes
        if self.get_classification() != Object.CLASSIFICATION_UNKNOWN:
            obj.object_classified = True
            obj.classification = self.get_classification()
            obj.classification_certainty = 255
            obj.classification_age = self.classification_age

        return obj

    def get_classification(self):  # pylint: disable=no-self-use
        """
        Function to get object classification (overridden in subclasses)
        """
        return Object.CLASSIFICATION_UNKNOWN

    def get_marker_color(self):  # pylint: disable=no-self-use
        """
        Function (override) to return the color for marker messages.

        :return: default color used by traffic participants
        :rtpye : std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 0.
        color.g = 0.
        color.b = 255.
        return color

    def get_marker_pose(self):
        """
        Function to return the pose for traffic participants.

        :return: the pose of the traffic participant.
        :rtype: geometry_msgs.msg.Pose
        """
        return self.get_bounding_box_ros_pose()

    def get_marker(self, timestamp=None):
        """
        Helper function to create a ROS visualization_msgs.msg.Marker for the actor

        :return:
        visualization_msgs.msg.Marker
        """
        marker = Marker(header=self.get_msg_header(frame_id="map", timestamp=timestamp))
        marker.color = self.get_marker_color()
        marker.color.a = 0.3
        marker.id = self.get_id()
        marker.type = Marker.CUBE

        marker.pose = self.get_marker_pose()
        extent = self.get_shape_extent()
        marker.scale.x = extent[0] * 2.0
        marker.scale.y = extent[1] * 2.0
        marker.scale.z = extent[2] * 2.0
        return marker
