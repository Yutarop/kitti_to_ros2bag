#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys

try:
    import pykitti
except ImportError as e:
    print('Could not load module \'pykitti\'. Please run `pip install pykitti`')
    sys.exit(1)

import tf2_ros
try:
    import transforms3d
except ImportError as e:
    print('Could not load module \'transforms3d\'. Please run `pip install transforms3d`')
    sys.exit(1)
import os
import cv2
import rclpy
from rclpy.serialization import serialize_message
from rclpy.time import Time
from rclpy.duration import Duration
import rosbag2_py
import progressbar
from tf2_msgs.msg import TFMessage
from datetime import datetime
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatFix
import sensor_msgs_py.point_cloud2 as pcl2
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform
from cv_bridge import CvBridge
import numpy as np
import argparse

def save_imu_data(writer, kitti, imu_frame_id, topic):
    print("Exporting IMU")
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        q = transforms3d.euler.euler2quat(oxts.packet.roll, oxts.packet.pitch, oxts.packet.yaw)
        imu = Imu()
        imu.header.frame_id = imu_frame_id
        imu.header.stamp = Time(seconds=float(timestamp.strftime("%s")), 
                              nanoseconds=int(float(timestamp.strftime("0.%f")) * 1e9)).to_msg()
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        imu.linear_acceleration.x = oxts.packet.af
        imu.linear_acceleration.y = oxts.packet.al
        imu.linear_acceleration.z = oxts.packet.au
        imu.angular_velocity.x = oxts.packet.wf
        imu.angular_velocity.y = oxts.packet.wl
        imu.angular_velocity.z = oxts.packet.wu
        
        writer.write(topic, serialize_message(imu), int(Time.from_msg(imu.header.stamp).nanoseconds))


def save_dynamic_tf(writer, kitti, kitti_type, initial_time):
    print("Exporting time dependent transformations")
    if kitti_type.find("raw") != -1:
        for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
            tf_oxts_msg = TFMessage()
            tf_oxts_transform = TransformStamped()
            tf_oxts_transform.header.stamp = Time(seconds=float(timestamp.strftime("%s")), 
                                                nanoseconds=int(float(timestamp.strftime("0.%f")) * 1e9)).to_msg()
            tf_oxts_transform.header.frame_id = 'world'
            tf_oxts_transform.child_frame_id = 'base_link'

            transform = (oxts.T_w_imu)
            t = transform[0:3, 3]
            q = transforms3d.quaternions.mat2quat(transform[:3, :3])
            oxts_tf = Transform()

            oxts_tf.translation.x = t[0]
            oxts_tf.translation.y = t[1]
            oxts_tf.translation.z = t[2]

            oxts_tf.rotation.x = q[0]
            oxts_tf.rotation.y = q[1]
            oxts_tf.rotation.z = q[2]
            oxts_tf.rotation.w = q[3]

            tf_oxts_transform.transform = oxts_tf
            tf_oxts_msg.transforms.append(tf_oxts_transform)

            writer.write('/tf', serialize_message(tf_oxts_msg), 
                        int(Time.from_msg(tf_oxts_msg.transforms[0].header.stamp).nanoseconds))

    elif kitti_type.find("odom") != -1:
        timestamps = map(lambda x: initial_time + x.total_seconds(), kitti.timestamps)
        for timestamp, tf_matrix in zip(timestamps, kitti.T_w_cam0):
            tf_msg = TFMessage()
            tf_stamped = TransformStamped()
            tf_stamped.header.stamp = Time(seconds=timestamp).to_msg()
            tf_stamped.header.frame_id = 'world'
            tf_stamped.child_frame_id = 'camera_left'
            
            t = tf_matrix[0:3, 3]
            q = transforms3d.quaternions.mat2quat(tf_matrix[:3, :3])
            transform = Transform()

            transform.translation.x = t[0]
            transform.translation.y = t[1]
            transform.translation.z = t[2]

            transform.rotation.x = q[0]
            transform.rotation.y = q[1]
            transform.rotation.z = q[2]
            transform.rotation.w = q[3]

            tf_stamped.transform = transform
            tf_msg.transforms.append(tf_stamped)

            writer.write('/tf', serialize_message(tf_msg), 
                        int(Time.from_msg(tf_msg.transforms[0].header.stamp).nanoseconds))
          
        
def save_camera_data(writer, kitti_type, kitti, util, bridge, camera, camera_frame_id, topic, initial_time):
    print("Exporting camera {}".format(camera))
    if kitti_type.find("raw") != -1:
        camera_pad = '{0:02d}'.format(camera)
        image_dir = os.path.join(kitti.data_path, 'image_{}'.format(camera_pad))
        image_path = os.path.join(image_dir, 'data')
        image_filenames = sorted(os.listdir(image_path))
        with open(os.path.join(image_dir, 'timestamps.txt')) as f:
            image_datetimes = map(lambda x: datetime.strptime(x[:-4], '%Y-%m-%d %H:%M:%S.%f'), f.readlines())
        
        calib = CameraInfo()
        calib.header.frame_id = camera_frame_id
        calib.width, calib.height = [int(x) for x in util['S_rect_{}'.format(camera_pad)].tolist()]
        calib.distortion_model = 'plumb_bob'
        calib.k = util['K_{}'.format(camera_pad)].flatten().tolist()
        calib.r = util['R_rect_{}'.format(camera_pad)].flatten().tolist()
        calib.d = util['D_{}'.format(camera_pad)].flatten().tolist()
        calib.p = util['P_rect_{}'.format(camera_pad)].flatten().tolist()
            
    elif kitti_type.find("odom") != -1:
        camera_pad = '{0:01d}'.format(camera)
        image_path = os.path.join(kitti.sequence_path, 'image_{}'.format(camera_pad))
        image_filenames = sorted(os.listdir(image_path))
        image_datetimes = map(lambda x: initial_time + x.total_seconds(), kitti.timestamps)
        
        calib = CameraInfo()
        calib.header.frame_id = camera_frame_id
        calib.p = util['P{}'.format(camera_pad)].flatten().tolist()
    
    iterable = zip(image_datetimes, image_filenames)
    bar = progressbar.ProgressBar()
    for dt, filename in bar(iterable):
        image_filename = os.path.join(image_path, filename)
        cv_image = cv2.imread(image_filename)
        calib.height, calib.width = int(cv_image.shape[0]), int(cv_image.shape[1])
        if camera in (0, 1):
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        encoding = "mono8" if camera in (0, 1) else "bgr8"
        image_message = bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
        image_message.header.frame_id = camera_frame_id
        if kitti_type.find("raw") != -1:
            image_message.header.stamp = Time(seconds=float(datetime.strftime(dt, "%s")), 
                                            nanoseconds=int(float(datetime.strftime(dt, "0.%f")) * 1e9)).to_msg()
            topic_ext = "/image_raw"
        elif kitti_type.find("odom") != -1:
            image_message.header.stamp = Time(seconds=dt).to_msg()
            topic_ext = "/image_rect"
        calib.header.stamp = image_message.header.stamp
        
        writer.write(topic + topic_ext, serialize_message(image_message), 
                    int(Time.from_msg(image_message.header.stamp).nanoseconds))
        writer.write(topic + '/camera_info', serialize_message(calib), 
                    int(Time.from_msg(calib.header.stamp).nanoseconds))
        
def save_velo_data(writer, kitti, velo_frame_id, topic):
    print("Exporting velodyne data")
    velo_path = os.path.join(kitti.data_path, 'velodyne_points')
    velo_data_dir = os.path.join(velo_path, 'data')
    velo_filenames = sorted(os.listdir(velo_data_dir))
    with open(os.path.join(velo_path, 'timestamps.txt')) as f:
        lines = f.readlines()
        velo_datetimes = []
        for line in lines:
            if len(line) == 1:
                continue
            dt = datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
            velo_datetimes.append(dt)

    iterable = zip(velo_datetimes, velo_filenames)
    bar = progressbar.ProgressBar()
    for dt, filename in bar(iterable):
        if dt is None:
            continue

        velo_filename = os.path.join(velo_data_dir, filename)

        # read binary data
        scan = (np.fromfile(velo_filename, dtype=np.float32)).reshape(-1, 4)

        # create header
        header = Header()
        header.frame_id = velo_frame_id
        header.stamp = Time(seconds=float(datetime.strftime(dt, "%s")), 
                          nanoseconds=int(float(datetime.strftime(dt, "0.%f")) * 1e9)).to_msg()

        # fill pcl msg
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                  PointField(name='i', offset=12, datatype=PointField.FLOAT32, count=1)]
        pcl_msg = pcl2.create_cloud(header, fields, scan)

        writer.write(topic + '/pointcloud', serialize_message(pcl_msg), 
                    int(Time.from_msg(pcl_msg.header.stamp).nanoseconds))


def get_static_transform(from_frame_id, to_frame_id, transform):
    t = transform[0:3, 3]
    q = transforms3d.quaternions.mat2quat(transform[:3, :3])
    tf_msg = TransformStamped()
    tf_msg.header.frame_id = from_frame_id
    tf_msg.child_frame_id = to_frame_id
    tf_msg.transform.translation.x = float(t[0])
    tf_msg.transform.translation.y = float(t[1])
    tf_msg.transform.translation.z = float(t[2])
    tf_msg.transform.rotation.x = float(q[0])
    tf_msg.transform.rotation.y = float(q[1])
    tf_msg.transform.rotation.z = float(q[2])
    tf_msg.transform.rotation.w = float(q[3])
    return tf_msg


def inv(transform):
    "Invert rigid body transformation matrix"
    R = transform[0:3, 0:3]
    t = transform[0:3, 3]
    t_inv = -1 * R.T.dot(t)
    transform_inv = np.eye(4)
    transform_inv[0:3, 0:3] = R.T
    transform_inv[0:3, 3] = t_inv
    return transform_inv


def save_static_transforms(writer, transforms, timestamps):
    print("Exporting static transformations")
    tfm = TFMessage()
    for transform in transforms:
        t = get_static_transform(from_frame_id=transform[0], to_frame_id=transform[1], transform=transform[2])
        tfm.transforms.append(t)
    for timestamp in timestamps:
        time_stamp = Time(seconds=float(timestamp.strftime("%s")), 
                         nanoseconds=int(float(timestamp.strftime("0.%f")) * 1e9)).to_msg()
        for i in range(len(tfm.transforms)):
            tfm.transforms[i].header.stamp = time_stamp
        writer.write('/tf_static', serialize_message(tfm), 
                    int(Time.from_msg(time_stamp).nanoseconds))


def save_gps_fix_data(writer, kitti, gps_frame_id, topic):
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header.frame_id = gps_frame_id
        navsatfix_msg.header.stamp = Time(seconds=float(timestamp.strftime("%s")), 
                                        nanoseconds=int(float(timestamp.strftime("0.%f")) * 1e9)).to_msg()
        navsatfix_msg.latitude = oxts.packet.lat
        navsatfix_msg.longitude = oxts.packet.lon
        navsatfix_msg.altitude = oxts.packet.alt
        navsatfix_msg.status.service = 1
        writer.write(topic, serialize_message(navsatfix_msg), 
                    int(Time.from_msg(navsatfix_msg.header.stamp).nanoseconds))


def save_gps_vel_data(writer, kitti, gps_frame_id, topic):
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = gps_frame_id
        twist_msg.header.stamp = Time(seconds=float(timestamp.strftime("%s")), 
                                    nanoseconds=int(float(timestamp.strftime("0.%f")) * 1e9)).to_msg()
        twist_msg.twist.linear.x = oxts.packet.vf
        twist_msg.twist.linear.y = oxts.packet.vl
        twist_msg.twist.linear.z = oxts.packet.vu
        twist_msg.twist.angular.x = oxts.packet.wf
        twist_msg.twist.angular.y = oxts.packet.wl
        twist_msg.twist.angular.z = oxts.packet.wu
        writer.write(topic, serialize_message(twist_msg), 
                    int(Time.from_msg(twist_msg.header.stamp).nanoseconds))


def run_kitti2bag():
    parser = argparse.ArgumentParser(description = "Convert KITTI dataset to ROS2 bag file the easy way!")
    # Accepted argument values
    kitti_types = ["raw_synced", "odom_color", "odom_gray"]
    odometry_sequences = []
    for s in range(22):
        odometry_sequences.append(str(s).zfill(2))
    
    parser.add_argument("kitti_type", choices = kitti_types, help = "KITTI dataset type")
    parser.add_argument("dir", nargs = "?", default = os.getcwd(), help = "base directory of the dataset, if no directory passed the deafult is current working directory")
    parser.add_argument("-t", "--date", help = "date of the raw dataset (i.e. 2011_09_26), option is only for RAW datasets.")
    parser.add_argument("-r", "--drive", help = "drive number of the raw dataset (i.e. 0001), option is only for RAW datasets.")
    parser.add_argument("-s", "--sequence", choices = odometry_sequences,help = "sequence of the odometry dataset (between 00 - 21), option is only for ODOMETRY datasets.")
    args = parser.parse_args()

    # Initialize ROS2
    rclpy.init()

    bridge = CvBridge()
    
    # CAMERAS
    cameras = [
        (0, 'camera_gray_left', '/kitti/camera_gray_left'),
        (1, 'camera_gray_right', '/kitti/camera_gray_right'),
        (2, 'camera_color_left', '/kitti/camera_color_left'),
        (3, 'camera_color_right', '/kitti/camera_color_right')
    ]

    if args.kitti_type.find("raw") != -1:
    
        if args.date == None:
            print("Date option is not given. It is mandatory for raw dataset.")
            print("Usage for raw dataset: kitti2bag raw_synced [dir] -t <date> -r <drive>")
            sys.exit(1)
        elif args.drive == None:
            print("Drive option is not given. It is mandatory for raw dataset.")
            print("Usage for raw dataset: kitti2bag raw_synced [dir] -t <date> -r <drive>")
            sys.exit(1)
        
        bag_filename = "kitti_{}_drive_{}_{}_ros2".format(args.date, args.drive, args.kitti_type[4:])
        
        # Create bag writer
        storage_options = rosbag2_py.StorageOptions(uri=bag_filename, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        writer = rosbag2_py.SequentialWriter()
        writer.open(storage_options, converter_options)
        
        kitti = pykitti.raw(args.dir, args.date, args.drive)
        if not os.path.exists(kitti.data_path):
            print('Path {} does not exists. Exiting.'.format(kitti.data_path))
            sys.exit(1)

        if len(kitti.timestamps) == 0:
            print('Dataset is empty? Exiting.')
            sys.exit(1)

        # Create topic info
        topic_infos = [
            rosbag2_py.TopicMetadata(name='/tf', type='tf2_msgs/msg/TFMessage', serialization_format='cdr'),
            rosbag2_py.TopicMetadata(name='/tf_static', type='tf2_msgs/msg/TFMessage', serialization_format='cdr'),
            rosbag2_py.TopicMetadata(name='/kitti/oxts/imu', type='sensor_msgs/msg/Imu', serialization_format='cdr'),
            rosbag2_py.TopicMetadata(name='/kitti/oxts/gps/fix', type='sensor_msgs/msg/NavSatFix', serialization_format='cdr'),
            rosbag2_py.TopicMetadata(name='/kitti/oxts/gps/vel', type='geometry_msgs/msg/TwistStamped', serialization_format='cdr'),
            rosbag2_py.TopicMetadata(name='/kitti/velo/pointcloud', type='sensor_msgs/msg/PointCloud2', serialization_format='cdr'),
        ]
        
        for camera in cameras:
            topic_infos.append(rosbag2_py.TopicMetadata(name=camera[2] + '/image_raw', type='sensor_msgs/msg/Image', serialization_format='cdr'))
            topic_infos.append(rosbag2_py.TopicMetadata(name=camera[2] + '/camera_info', type='sensor_msgs/msg/CameraInfo', serialization_format='cdr'))
        
        for topic_info in topic_infos:
            writer.create_topic(topic_info)

        try:
            # IMU
            imu_frame_id = 'imu_link'
            imu_topic = '/kitti/oxts/imu'
            gps_fix_topic = '/kitti/oxts/gps/fix'
            gps_vel_topic = '/kitti/oxts/gps/vel'
            velo_frame_id = 'velo_link'
            velo_topic = '/kitti/velo'

            T_base_link_to_imu = np.eye(4, 4)
            T_base_link_to_imu[0:3, 3] = [-2.71/2.0-0.05, 0.32, 0.93]

            # tf_static
            transforms = [
                ('base_link', imu_frame_id, T_base_link_to_imu),
                (imu_frame_id, velo_frame_id, inv(kitti.calib.T_velo_imu)),
                (imu_frame_id, cameras[0][1], inv(kitti.calib.T_cam0_imu)),
                (imu_frame_id, cameras[1][1], inv(kitti.calib.T_cam1_imu)),
                (imu_frame_id, cameras[2][1], inv(kitti.calib.T_cam2_imu)),
                (imu_frame_id, cameras[3][1], inv(kitti.calib.T_cam3_imu))
            ]

            util = pykitti.utils.read_calib_file(os.path.join(kitti.calib_path, 'calib_cam_to_cam.txt'))

            # Export
            save_static_transforms(writer, transforms, kitti.timestamps)
            save_dynamic_tf(writer, kitti, args.kitti_type, initial_time=None)
            save_imu_data(writer, kitti, imu_frame_id, imu_topic)
            save_gps_fix_data(writer, kitti, imu_frame_id, gps_fix_topic)
            save_gps_vel_data(writer, kitti, imu_frame_id, gps_vel_topic)
            for camera in cameras:
                save_camera_data(writer, args.kitti_type, kitti, util, bridge, camera=camera[0], camera_frame_id=camera[1], topic=camera[2], initial_time=None)
            save_velo_data(writer, kitti, velo_frame_id, velo_topic)

        finally:
            print("## OVERVIEW ##")
            print("Bag file created: {}.db3".format(bag_filename))
            
    elif args.kitti_type.find("odom") != -1:
        
        if args.sequence == None:
            print("Sequence option is not given. It is mandatory for odometry dataset.")
            print("Usage for odometry dataset: kitti2bag {odom_color, odom_gray} [dir] -s <sequence>")
            sys.exit(1)
            
        bag_filename = "kitti_data_odometry_{}_sequence_{}_ros2".format(args.kitti_type[5:],args.sequence)
        
        # Create bag writer
        storage_options = rosbag2_py.StorageOptions(uri=bag_filename, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        writer = rosbag2_py.SequentialWriter()
        writer.open(storage_options, converter_options)
        
        kitti = pykitti.odometry(args.dir, args.sequence)
        if not os.path.exists(kitti.sequence_path):
            print('Path {} does not exists. Exiting.'.format(kitti.sequence_path))
            sys.exit(1)

        kitti.load_calib()         
        kitti.load_timestamps() 
             
        if len(kitti.timestamps) == 0:
            print('Dataset is empty? Exiting.')
            sys.exit(1)
            
        if args.sequence in odometry_sequences[:11]:
            print("Odometry dataset sequence {} has ground truth information (poses).".format(args.sequence))
            kitti.load_poses()

        # Create topic info for odometry
        topic_infos = [
            rosbag2_py.TopicMetadata(name='/tf', type='tf2_msgs/msg/TFMessage', serialization_format='cdr'),
        ]
        
        if args.kitti_type.find("gray") != -1:
            used_cameras = cameras[:2]
        elif args.kitti_type.find("color") != -1:
            used_cameras = cameras[-2:]
            
        for camera in used_cameras:
            topic_infos.append(rosbag2_py.TopicMetadata(name=camera[2] + '/image_rect', type='sensor_msgs/msg/Image', serialization_format='cdr'))
            topic_infos.append(rosbag2_py.TopicMetadata(name=camera[2] + '/camera_info', type='sensor_msgs/msg/CameraInfo', serialization_format='cdr'))
        
        for topic_info in topic_infos:
            writer.create_topic(topic_info)

        try:
            util = pykitti.utils.read_calib_file(os.path.join(args.dir,'sequences',args.sequence, 'calib.txt'))
            current_epoch = (datetime.utcnow() - datetime(1970, 1, 1)).total_seconds()
            # Export
            save_dynamic_tf(writer, kitti, args.kitti_type, initial_time=current_epoch)
            for camera in used_cameras:
                save_camera_data(writer, args.kitti_type, kitti, util, bridge, camera=camera[0], camera_frame_id=camera[1], topic=camera[2], initial_time=current_epoch)

        finally:
            print("## OVERVIEW ##")
            print("Bag file created: {}.db3".format(bag_filename))

    # Shutdown ROS2
    rclpy.shutdown()

if __name__ == '__main__':
    run_kitti2bag()