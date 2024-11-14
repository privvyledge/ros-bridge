#!/usr/bin/env python

#
# Copyright (c) 2018, Willow Garage, Inc.
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla lidars
"""

import numpy

from carla_ros_bridge.sensor import Sensor, create_cloud

from sensor_msgs.msg import PointCloud2, PointField
# from rclpy import qos


class Lidar(Sensor):

    """
    Actor implementation details for lidars
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(Lidar, self).__init__(uid=uid,
                                    name=name,
                                    parent=parent,
                                    relative_spawn_pose=relative_spawn_pose,
                                    node=node,
                                    carla_actor=carla_actor,
                                    synchronous_mode=synchronous_mode)

        self.lidar_publisher = node.new_publisher(PointCloud2,
                                                  self.get_topic_prefix(),
                                                  # qos_profile=qos.qos_profile_sensor_data,
                                                  qos_profile=10
                                                  )
        self.listen()
        self.channels = int(self.carla_actor.attributes.get('channels'))
        # self._frame_id = frame_id

    def destroy(self):
        super(Lidar, self).destroy()
        self.node.destroy_publisher(self.lidar_publisher)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_lidar_measurement):
        """
        Function to transform the a received lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla lidar measurement object
        :type carla_lidar_measurement: carla.LidarMeasurement
        """
        # header = self.get_msg_header(frame_id="velodyne_top", timestamp=carla_lidar_measurement.timestamp)
        # header = self.get_msg_header(frame_id=self._frame_id, timestamp=carla_lidar_measurement.timestamp)
        header = self.get_msg_header(timestamp=carla_lidar_measurement.timestamp)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.UINT8, count=1),
            PointField(name="return_type", offset=13, datatype=PointField.UINT8, count=1),
            PointField(name='channel', offset=14, datatype=PointField.UINT16, count=1),  # usually called ring but autoware uses channel [16]
            PointField(name='azimuth', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='elevation', offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='distance', offset=24, datatype=PointField.FLOAT32, count=1),
        ]

        lidar_data = numpy.fromstring(
            bytes(carla_lidar_measurement.raw_data), dtype=numpy.float32)
        lidar_data = numpy.reshape(
            lidar_data, (int(lidar_data.shape[0] / 4), 4))
        
        intensity = lidar_data[:, 3]
        intensity = (
                numpy.clip(intensity, 0, 1) * 255
        )  # CARLA lidar intensity values are between 0 and 1
        intensity = intensity.astype(numpy.uint8).reshape(-1, 1)

        return_type = numpy.zeros((lidar_data.shape[0], 1), dtype=numpy.uint8)
        channel = numpy.empty((0, 1), dtype=numpy.uint16)
        
        for i in range(self.channels):
            current_ring_points_count = carla_lidar_measurement.get_point_count(i)
            channel = numpy.vstack((
                channel,
                numpy.full((current_ring_points_count, 1), i, dtype=numpy.uint16)))

        # channel = numpy.delete(channel, 0, axis=0)
        
        # we take the opposite of y axis
        # (as lidar point are express in left handed coordinate system, and ros need right handed)
        lidar_data[:, 1] *= -1

        # calculate the azimuth, elevation and distance for each point.
        # azimuth = numpy.arctan2(lidar_data[:, 1], lidar_data[:, 0]).astype(numpy.float32)
        # distance = numpy.linalg.norm(lidar_data[:, :3])
        # # distance = numpy.sqrt(lidar_data[:, 0] ** 2 + lidar_data[:, 1] ** 2 + lidar_data[:, 2] ** 2).astype(numpy.float32)
        # elevation = numpy.arctan2(lidar_data[:, 2], numpy.sqrt(lidar_data[:, 0] ** 2 + lidar_data[:, 1] ** 2)).astype(
        #     numpy.float32)

        azimuth = numpy.zeros((lidar_data.shape[0], 1), dtype=numpy.float32)
        elevation = numpy.zeros((lidar_data.shape[0], 1), dtype=numpy.float32)
        distance = numpy.zeros((lidar_data.shape[0], 1), dtype=numpy.float32)
        for i in range(lidar_data.shape[0]):
            azimuth[i] = numpy.arctan2(lidar_data[i, 1], lidar_data[i, 0])
            distance[i] = numpy.sqrt(lidar_data[i, 0] ** 2 + lidar_data[i, 1] ** 2 + lidar_data[i, 2] ** 2)
            elevation[i] = numpy.arctan2(lidar_data[i, 2], distance[i])

        # lidar_data = numpy.hstack((lidar_data[:, :3], intensity, return_type, channel))
        lidar_data = numpy.hstack((lidar_data[:, :3], intensity, return_type, channel, azimuth, elevation, distance))

        # changes made for autoware (https://github.com/autowarefoundation/autoware.universe/blob/main/simulator/autoware_carla_interface/src/autoware_carla_interface/carla_ros.py#L240)
        # https://github.com/Robotics010/carla_autoware_bridge/blob/master/carla_autoware_bridge/converter/lidar_ex.py#L137
        # https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/sensing/data-types/point-cloud/#list-of-modules
        dtype = [
            ("x", "f4"),
            ("y", "f4"),
            ("z", "f4"),
            ("intensity", "u1"),
            ("return_type", "u1"),
            ("channel", "u2"),
            ("azimuth", "f4"),
            ("elevation", "f4"),
            ("distance", "f4"),
        ]

        structured_lidar_data = numpy.zeros(lidar_data.shape[0], dtype=dtype)
        structured_lidar_data["x"] = lidar_data[:, 0]
        structured_lidar_data["y"] = lidar_data[:, 1]
        structured_lidar_data["z"] = lidar_data[:, 2]
        structured_lidar_data["intensity"] = lidar_data[:, 3].astype(numpy.uint8)
        structured_lidar_data["return_type"] = lidar_data[:, 4].astype(numpy.uint8)
        structured_lidar_data["channel"] = lidar_data[:, 5].astype(numpy.uint16)
        structured_lidar_data["azimuth"] = lidar_data[:, 6].astype(numpy.float32)
        structured_lidar_data["elevation"] = lidar_data[:, 7].astype(numpy.float32)
        structured_lidar_data["distance"] = lidar_data[:, 8].astype(numpy.float32)

        point_cloud_msg = create_cloud(header, fields, structured_lidar_data)
        self.lidar_publisher.publish(point_cloud_msg)


class SemanticLidar(Sensor):

    """
    Actor implementation details for semantic lidars
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(SemanticLidar, self).__init__(uid=uid,
                                            name=name,
                                            parent=parent,
                                            relative_spawn_pose=relative_spawn_pose,
                                            node=node,
                                            carla_actor=carla_actor,
                                            synchronous_mode=synchronous_mode)

        self.semantic_lidar_publisher = node.new_publisher(
            PointCloud2,
            self.get_topic_prefix(),
            qos_profile=10)
        self.listen()

    def destroy(self):
        super(SemanticLidar, self).destroy()
        self.node.destroy_publisher(self.semantic_lidar_publisher)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_lidar_measurement):
        """
        Function to transform a received semantic lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla semantic lidar measurement object
        :type carla_lidar_measurement: carla.SemanticLidarMeasurement
        """
        header = self.get_msg_header(timestamp=carla_lidar_measurement.timestamp)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='CosAngle', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ObjIdx', offset=16, datatype=PointField.UINT32, count=1),
            PointField(name='ObjTag', offset=20, datatype=PointField.UINT32, count=1)
        ]

        lidar_data = numpy.fromstring(bytes(carla_lidar_measurement.raw_data),
                                      dtype=numpy.dtype([
                                          ('x', numpy.float32),
                                          ('y', numpy.float32),
                                          ('z', numpy.float32),
                                          ('CosAngle', numpy.float32),
                                          ('ObjIdx', numpy.uint32),
                                          ('ObjTag', numpy.uint32)
                                      ]))

        # we take the oposite of y axis
        # (as lidar point are express in left handed coordinate system, and ros need right handed)
        lidar_data['y'] *= -1
        point_cloud_msg = create_cloud(header, fields, lidar_data.tolist())
        self.semantic_lidar_publisher.publish(point_cloud_msg)
