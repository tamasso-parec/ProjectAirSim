"""
Copyright (C) Microsoft Corporation. 
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.
ROS bridge for Project AirSim: Sensor message conversion module
"""

import math

import geometry_msgs.msg as rosgeommsg
import radar_msgs.msg as rosradarmsg
import sensor_msgs.msg as rossensmsg
import std_msgs.msg as rosstdmsg

import numpy as np

from . import utils
from .interface_profile import DepthSettings
from .node import ROSNode
from .sim_time import SimTimeSource


class MsgConverter:
    """
    This class contains methods to convert Project AirSim topic messages to and
    from ROS topics messages.
    """

    # -------------------------------------------------------------------------
    # MsgConverter Constants
    # -------------------------------------------------------------------------

    # Global Navigation Satellite System (GNSS) fix type
    GNSS_FIX_NO_FIX = 0
    GNSS_FIX_TIME_ONLY = 1
    GNSS_FIX_2D_FIX = 2
    GNSS_FIX_3D_FIX = 3

    # Initialized covariance matrix-as-array indicating no covariance data
    NO_COVARIANCE_MATRIX = [0.0] * 9

    # Raw 16UC1 value Project AirSim writes when a depth sample is beyond the
    # representable range.  See ImagePackingAsyncTask.cpp, which saturates
    # millimetre depth at UINT16_MAX.
    DEPTH_SATURATED_MM = 65535

    def __init__(
        self,
        ros_node: ROSNode,
        sim_time: SimTimeSource = None,
        coords: utils.CoordinateConverter = None,
        depth: DepthSettings = None,
    ):
        """
        Constructor.

        Arguments:
            ros_node - Project AirSim ROS node object
            sim_time - Simulation clock source used to stamp messages; if
                None, messages are stamped from the ROS wall clock
            coords - Coordinate converter; if None, the process-wide default
                is used
            depth - Depth image conversion settings; if None, defaults apply
        """
        self.ros_node = ros_node  # ROS node
        self.sim_time = (
            sim_time if sim_time is not None else SimTimeSource(ros_node, enabled=False)
        )
        self.coords = (
            coords if coords is not None else utils.get_default_coordinate_converter()
        )
        self.depth = depth if depth is not None else DepthSettings()
        self.robot_base_frame_ids = (
            {}
        )  # Mapping from Project AirSim topic name to robot's base transform frame ID

    @property
    def max_depth_mm(self) -> int:
        """
        Maximum depth in millimetres represented by the legacy mono8 depth
        encoding, where it maps to a pixel value of 255.
        """
        return int(self.depth.mono8_max_range_m * 1000.0)

    # -------------------------------------------------------------------------
    # Coordinate conversion helpers
    #
    # These use this converter's own CoordinateConverter rather than the
    # module-level default, so the configured frame convention applies even
    # when several converters coexist (for instance in tests).
    #
    # World quantities follow the configured world convention; body-relative
    # quantities always convert Project AirSim FRD to ROS FLU.
    # -------------------------------------------------------------------------

    def _world_point(self, projectairsim_vector) -> rosgeommsg.Point:
        x, y, z = self.coords.world_vector(
            (
                projectairsim_vector["x"],
                projectairsim_vector["y"],
                projectairsim_vector["z"],
            )
        )
        return rosgeommsg.Point(x=x, y=y, z=z)

    def _world_vector3(self, projectairsim_vector) -> rosgeommsg.Vector3:
        x, y, z = self.coords.world_vector(
            (
                projectairsim_vector["x"],
                projectairsim_vector["y"],
                projectairsim_vector["z"],
            )
        )
        return rosgeommsg.Vector3(x=x, y=y, z=z)

    def _world_position_list(self, projectairsim_vector):
        return self.coords.world_vector(
            (
                projectairsim_vector["x"],
                projectairsim_vector["y"],
                projectairsim_vector["z"],
            )
        )

    def _world_quaternion(self, projectairsim_quaternion) -> rosgeommsg.Quaternion:
        w, x, y, z = self.coords.world_quaternion(
            (
                projectairsim_quaternion["w"],
                projectairsim_quaternion["x"],
                projectairsim_quaternion["y"],
                projectairsim_quaternion["z"],
            )
        )
        return rosgeommsg.Quaternion(x=x, y=y, z=z, w=w)

    def _world_quaternion_list(self, projectairsim_quaternion):
        w, x, y, z = self.coords.world_quaternion(
            (
                projectairsim_quaternion["w"],
                projectairsim_quaternion["x"],
                projectairsim_quaternion["y"],
                projectairsim_quaternion["z"],
            )
        )
        return (x, y, z, w)

    def _body_vector3(self, projectairsim_vector) -> rosgeommsg.Vector3:
        x, y, z = self.coords.body_vector(
            (
                projectairsim_vector["x"],
                projectairsim_vector["y"],
                projectairsim_vector["z"],
            )
        )
        return rosgeommsg.Vector3(x=x, y=y, z=z)

    def _body_vector_list(self, projectairsim_vector):
        return self.coords.body_vector(
            (
                projectairsim_vector["x"],
                projectairsim_vector["y"],
                projectairsim_vector["z"],
            )
        )

    def _to_projectairsim_world_position(self, ros_vector):
        x, y, z = self.coords.world_vector((ros_vector.x, ros_vector.y, ros_vector.z))
        return {"x": x, "y": y, "z": z}

    def _to_projectairsim_world_quaternion(self, ros_quaternion):
        w, x, y, z = self.coords.world_quaternion(
            (ros_quaternion.w, ros_quaternion.x, ros_quaternion.y, ros_quaternion.z)
        )
        return {"x": x, "y": y, "z": z, "w": w}

    def convert_actual_pose_to_ros(self, projectairsim_topic_name, projectairsim_pose):
        """
        Convert a Project AirSim vehicle pose into a ROS Pose message.

        Arguments:
            projectairsim_topic_name - The Project AirSim topic name
            projectairsim_pose - The vehicle pose received from the Project AirSim topic

        Return:
            (return) - Corresponding ROS Pose message
        """
        posestamped = rosgeommsg.PoseStamped()
        posestamped.header.stamp = self.sim_time.stamp(projectairsim_pose)
        # posestamped.header.frame_id is set by PoseBridgeToROS

        posestamped.pose.position = self._world_point(projectairsim_pose["position"])
        posestamped.pose.orientation = self._world_quaternion(
            projectairsim_pose["orientation"]
        )

        return posestamped

    def convert_barometer_to_ros(self, projectairsim_topic_name, projectairsim_msg):
        """
        Convert a Project AirSim barometer sensor message into a ROS FluidPressure message.

        Arguments:
            projectairsim_topic_name - The Project AirSim topic name
            projectairsim_msg - The barometer data received from the Project AirSim topic

        Returns:
            (return) - Corresponding ROS FluidPressure message
        """
        fluid_pressure = rossensmsg.FluidPressure()
        fluid_pressure.header = self._get_standard_ros_header(
            projectairsim_topic_name, projectairsim_msg
        )

        fluid_pressure.fluid_pressure = float(projectairsim_msg["pressure"])
        fluid_pressure.variance = 0.0

        return fluid_pressure

    def convert_desired_pose_from_ros(self, ros_topic_name, ros_posestamped):
        """
        Convert a ROS vehicle pose request into a Project AirSim pose message.

        Arguments:
            topic_sub_handler - Topic handler for this topic
            ros_topic_name - The ROS topic name
            ros_posestamped - The vehicle pose received from the ROS topic

        Return:
            (return) - Corresponding Project AirSim Pose message
        """
        projectairsim_pose = {
            "position": self._to_projectairsim_world_position(
                ros_posestamped.pose.position
            ),
            "orientation": self._to_projectairsim_world_quaternion(
                ros_posestamped.pose.orientation
            ),
        }
        return projectairsim_pose

    def convert_gps_to_ros(self, projectairsim_topic_name, projectairsim_msg):
        """
        Convert a Project AirSim GPS sensor message into a ROS NavSatFix message.

        Arguments:
            projectairsim_topic_name - The Project AirSim topic name
            projectairsim_msg - The GPS data received from the Project AirSim topic

        Returns:
            (return) - Corresponding ROS NavSatFix message
        """
        nav_sat_fix = rossensmsg.NavSatFix()
        nav_sat_fix.header = self._get_standard_ros_header(
            projectairsim_topic_name, projectairsim_msg
        )

        nav_sat_fix.status.status = (
            rossensmsg.NavSatStatus.STATUS_SBAS_FIX
            if projectairsim_msg["fix_type"] >= self.GNSS_FIX_2D_FIX
            else rossensmsg.NavSatStatus.STATUS_NO_FIX
        )
        nav_sat_fix.status.service = rossensmsg.NavSatStatus.SERVICE_GPS

        nav_sat_fix.latitude = float(projectairsim_msg["latitude"])
        nav_sat_fix.longitude = float(projectairsim_msg["longitude"])
        nav_sat_fix.altitude = float(projectairsim_msg["altitude"])
        nav_sat_fix.position_covariance = [0.0] * 9
        nav_sat_fix.position_covariance_type = rossensmsg.NavSatFix.COVARIANCE_TYPE_UNKNOWN

        return nav_sat_fix

    def convert_image_to_ros(self, projectairsim_topic_name, projectairsim_image):
        """
        Convert a Project AirSim image message into a ROS image message.

        Arguments:
            projectairsim_topic_name - The Project AirSim topic name
            projectairsim_image - The image message received from the Project AirSim topic

        Return:
            (return) - Corresponding ROS Image message
        """
        if projectairsim_image["encoding"] == "BGR":
            return self.convert_image_bgr8_to_ros(
                projectairsim_topic_name, projectairsim_image
            )
        elif projectairsim_image["encoding"] == "16UC1":
            return self.convert_image_16uc1_to_ros(
                projectairsim_topic_name, projectairsim_image
            )
        else:
            raise ValueError(
                f"Can only handle image encoding BGR or 16UC1, not \"{projectairsim_image['encoding']}\""
            )

    def convert_image_bgr8_to_ros(
        self, projectairsim_topic_name, projectairsim_image_bgr8
    ):
        """
        Convert a Project AirSim bgr8 image message into a ROS image message.

        Arguments:
            projectairsim_topic_name - The Project AirSim topic name
            projectairsim_image - The bgr8 image message received from the Project AirSim topic

        Return:
            (return) - Corresponding ROS Image message
        """
        image = rossensmsg.Image()
        image.header.stamp = self.sim_time.stamp(projectairsim_image_bgr8)
        # image.header.frame_id must be set by caller

        # Get image parameters
        image.height = int(projectairsim_image_bgr8["height"])
        image.width = int(projectairsim_image_bgr8["width"])
        image.encoding = "bgr8"
        image.is_bigendian = int(projectairsim_image_bgr8["big_endian"])

        # Convert image data to uncompressed bitmap data
        image.data = projectairsim_image_bgr8["data"]
        image.step = 3 * image.width

        return image

    def convert_image_16uc1_to_ros(
        self, projectairsim_topic_name, projectairsim_image_16uc1
    ):
        """
        Convert a Project AirSim 16uc1 depth image message into a ROS image
        message.

        Project AirSim transmits depth as 16-bit unsigned millimetres,
        saturating at DEPTH_SATURATED_MM.  Which ROS encoding this becomes is
        controlled by the bridge's depth settings:

            32FC1 (default) - metres as 32-bit float, the encoding the ROS
                depth ecosystem expects.  Samples with no reading become NaN
                and samples beyond the sensor range become +infinity, so that
                consumers such as depth_image_proc and OctoMap skip them.
            16UC1 - millimetres, unchanged from the wire format.  Samples
                beyond the sensor range become 0, the 16UC1 "no reading"
                value.
            mono8 - the bridge's historical output, scaled so that the
                configured mono8 range maps to 255.  Lossy; suitable only for
                viewing.

        Arguments:
            projectairsim_topic_name - The Project AirSim topic name
            projectairsim_image - The 16uc1 image message received from the Project AirSim topic

        Return:
            (return) - Corresponding ROS Image message
        """
        image = rossensmsg.Image()
        image.header.stamp = self.sim_time.stamp(projectairsim_image_16uc1)
        # image.header.frame_id must be set by caller

        # Get image parameters
        height = int(projectairsim_image_16uc1["height"])
        width = int(projectairsim_image_16uc1["width"])
        image.height = height
        image.width = width

        # Respect the transmitted byte order when reading the raw samples, but
        # always emit native little-endian values.
        source_dtype = ">u2" if int(projectairsim_image_16uc1["big_endian"]) else "<u2"
        depth_mm = np.frombuffer(
            projectairsim_image_16uc1["data"], dtype=source_dtype
        ).reshape(height, width)
        image.is_bigendian = 0

        encoding = self.depth.encoding
        if encoding == DepthSettings.ENCODING_32FC1:
            depth_m = self._depth_metres(depth_mm)
            image.data = depth_m.tobytes()
            image.encoding = "32FC1"
            image.step = 4 * width
            return image

        if encoding == DepthSettings.ENCODING_MONO8:
            scaled = depth_mm.astype("float32") * (255.0 / self.max_depth_mm)
            image.data = np.clip(scaled, 0.0, 255.0).astype("uint8").tobytes()
            image.encoding = "mono8"
            image.step = width
            return image

        max_range_mm = (
            self.depth.max_range_m * 1000.0 if self.depth.max_range_m > 0.0 else None
        )

        # 16UC1 millimetres, as transmitted.
        output = depth_mm.astype("<u2")
        # 0 is the 16UC1 convention for "no reading", which is what both a
        # saturated sample and an out-of-range sample amount to.
        invalid = depth_mm >= self.DEPTH_SATURATED_MM
        if max_range_mm is not None:
            invalid = invalid | (depth_mm > max_range_mm)
        if invalid.any():
            output = np.where(invalid, np.uint16(0), output).astype("<u2")
        image.data = output.tobytes()
        image.encoding = "16UC1"
        image.step = 2 * width
        return image

    def _depth_metres(self, depth_mm):
        """
        Convert raw Project AirSim millimetre depth into metres.

        Samples the renderer produced no reading for become NaN, and samples
        beyond the sensor range become +infinity, following the ROS depth
        conventions so that consumers skip them rather than treating them as
        geometry.

        Arguments:
            depth_mm - Raw uint16 millimetre depth array

        Returns:
            (return) - float32 metre depth array, little-endian
        """
        depth_m = depth_mm.astype("<f4") * np.float32(0.001)
        # A zero sample is physically impossible for a rendered depth buffer,
        # so it means the renderer produced no reading.
        depth_m = np.where(depth_mm == 0, np.float32("nan"), depth_m)

        too_far = depth_mm >= self.DEPTH_SATURATED_MM
        if self.depth.max_range_m > 0.0:
            too_far = too_far | (depth_mm > self.depth.max_range_m * 1000.0)
        depth_m = np.where(too_far, np.float32("inf"), depth_m)

        return depth_m.astype("<f4")

    def convert_depth_image_to_point_cloud(
        self,
        projectairsim_image_16uc1,
        intrinsic_camera_matrix,
        frame_id,
        points_settings,
    ):
        """
        Reproject a Project AirSim depth image into a ROS PointCloud2.

        Project AirSim has no point-cloud sensor of its own for cameras, so
        the cloud a depth-camera-based stack expects is produced here from the
        depth image and the camera intrinsics.  Doing it in the bridge rather
        than in a downstream depth_image_proc node avoids shipping the depth
        image over DDS only to convert it, and gives the cloud the same
        simulation timestamp and frame as the image it came from.

        The cloud is organised (one point per pixel, row major) and not dense:
        invalid pixels are NaN, exactly as a Gazebo depth camera publishes
        them, so that a consumer can still index by pixel.

        Arguments:
            projectairsim_image_16uc1 - The depth image message from Project AirSim
            intrinsic_camera_matrix - Row-major 3x3 camera matrix, as in
                CameraInfo.k
            frame_id - Transform frame the points are expressed in
            points_settings - PointCloudSettings controlling axis convention
                and decimation

        Returns:
            (return) - Corresponding ROS PointCloud2 message, or None when the
                camera intrinsics are not known yet
        """
        fx = float(intrinsic_camera_matrix[0])
        fy = float(intrinsic_camera_matrix[4])
        cx = float(intrinsic_camera_matrix[2])
        cy = float(intrinsic_camera_matrix[5])
        if fx == 0.0 or fy == 0.0:
            # Camera info has not arrived yet; without focal lengths every
            # point would collapse onto the optical axis.
            return None

        height = int(projectairsim_image_16uc1["height"])
        width = int(projectairsim_image_16uc1["width"])
        source_dtype = ">u2" if int(projectairsim_image_16uc1["big_endian"]) else "<u2"
        depth_mm = np.frombuffer(
            projectairsim_image_16uc1["data"], dtype=source_dtype
        ).reshape(height, width)

        step = max(1, int(points_settings.decimation))
        if step > 1:
            depth_mm = depth_mm[::step, ::step]

        depth_m = self._depth_metres(depth_mm)
        rows, columns = depth_m.shape

        # Pixel centres of the retained samples, in the full-resolution grid.
        u = (np.arange(columns, dtype="<f4") * step - np.float32(cx)) / np.float32(fx)
        v = (np.arange(rows, dtype="<f4") * step - np.float32(cy)) / np.float32(fy)

        # An infinite reading has no position, so it joins the invalid samples
        # as NaN rather than becoming a point at infinity.
        valid_depth = np.where(np.isfinite(depth_m), depth_m, np.float32("nan"))

        forward = valid_depth
        right = valid_depth * u[np.newaxis, :]
        down = valid_depth * v[:, np.newaxis]

        cloud = np.empty((rows, columns, 3), dtype="<f4")
        if points_settings.is_optical:
            # Optical frame: X right, Y down, Z forward.
            cloud[..., 0] = right
            cloud[..., 1] = down
            cloud[..., 2] = forward
        else:
            # Body frame: X forward, Y left, Z up.
            cloud[..., 0] = forward
            cloud[..., 1] = -right
            cloud[..., 2] = -down

        point_cloud = rossensmsg.PointCloud2()
        point_cloud.header.stamp = self.sim_time.stamp(projectairsim_image_16uc1)
        point_cloud.header.frame_id = frame_id
        point_cloud.height = rows
        point_cloud.width = columns
        point_cloud.fields = [
            rossensmsg.PointField(
                name=name,
                offset=offset,
                datatype=rossensmsg.PointField.FLOAT32,
                count=1,
            )
            for name, offset in (("x", 0), ("y", 4), ("z", 8))
        ]
        point_cloud.is_bigendian = False
        point_cloud.point_step = 12
        point_cloud.row_step = 12 * columns
        point_cloud.data = cloud.tobytes()
        # Invalid pixels are retained as NaN to keep the cloud organised.
        point_cloud.is_dense = False

        return point_cloud

    def convert_collision_info_to_gz_contacts(
        self, projectairsim_topic_name, projectairsim_msg
    ):
        """
        Convert a Project AirSim collision report into a ros_gz_interfaces
        Contacts message.

        Project AirSim reports one collision at a time, so the Contacts
        message carries a single contact.  ros_gz_interfaces is imported
        lazily by the caller and passed in, keeping it an optional dependency.

        Arguments:
            projectairsim_topic_name - The Project AirSim topic name
            projectairsim_msg - The collision info received from the topic

        Returns:
            (return) - Corresponding ros_gz_interfaces/Contacts message, or
                None when the message reports no collision
        """
        gz_msgs = self._gz_interfaces_msgs()

        contacts = gz_msgs.Contacts()
        contacts.header = self._get_standard_ros_header(
            projectairsim_topic_name, projectairsim_msg
        )

        # Project AirSim republishes the last collision with an empty object
        # name when nothing is touching; a contact-free Contacts message is
        # what a Gazebo consumer expects in that case.
        object_name = projectairsim_msg.get("object_name") or ""
        if not object_name:
            return contacts

        contact = gz_msgs.Contact()
        # Gazebo names both sides of a contact.  Project AirSim only knows the
        # object the robot hit, so the robot's own frame stands in for the
        # other side.
        contact.collision1.name = self.robot_base_frame_ids.get(
            projectairsim_topic_name, ""
        )
        contact.collision2.name = str(object_name)

        # Contact.positions and .normals are Vector3 sequences, not Point.
        contact.positions = [
            self._world_vector3(projectairsim_msg["impact_point"])
        ]
        contact.normals = [self._world_vector3(projectairsim_msg["normal"])]
        contact.depths = [float(projectairsim_msg.get("penetration_depth", 0.0))]

        contacts.contacts = [contact]

        return contacts

    @staticmethod
    def _gz_interfaces_msgs():
        """
        Import ros_gz_interfaces on demand.

        Collision reporting is the only feature that needs it, and it is off
        by default, so the package stays an optional dependency rather than
        one every Project AirSim bridge user has to install.
        """
        try:
            import ros_gz_interfaces.msg as gz_msgs
        except ImportError as exc:
            raise ImportError(
                "collision.message 'gz_contacts' needs the ros_gz_interfaces "
                "package, which is not installed. Install it with "
                "'sudo apt install ros-$ROS_DISTRO-ros-gz-interfaces', or set "
                "collision.message to 'none' in the interface profile."
            ) from exc
        return gz_msgs

    def convert_imu_to_ros(self, projectairsim_topic_name, projectairsim_msg):
        """
        Convert a Project AirSim IMU sensor message into a ROS Imu message.

        Arguments:
            projectairsim_topic_name - The Project AirSim topic name
            projectairsim_msg - The IMU data received from the Project AirSim topic

        Returns:
            (return) - Corresponding ROS Imu message
        """
        imu = rossensmsg.Imu()
        imu.header = self._get_standard_ros_header(
            projectairsim_topic_name, projectairsim_msg
        )

        imu.orientation = self._world_quaternion(projectairsim_msg["orientation"])
        imu.orientation_covariance = self.NO_COVARIANCE_MATRIX
        imu.angular_velocity = self._body_vector3(
            projectairsim_msg["angular_velocity"]
        )
        imu.angular_velocity_covariance = self.NO_COVARIANCE_MATRIX
        imu.linear_acceleration = self._body_vector3(
            projectairsim_msg["linear_acceleration"]
        )
        imu.linear_acceleration_covariance = self.NO_COVARIANCE_MATRIX

        return imu

    def convert_lidar_to_ros(self, projectairsim_topic_name, projectairsim_lidar):
        """
        Convert a Project AirSim LIDAR point cloud message into a ROS PointCloud2 message.

        Arguments:
            projectairsim_topic_name - The Project AirSim topic name
            projectairsim_lidar - The LIDAR data received from the Project AirSim topic

        Returns:
            (return) - ROS geometry_msg.PointCloud2 message
        """
        point_cloud_airsim = projectairsim_lidar["point_cloud"]

        # Convert data stream into array of 3D points.  The points are
        # relative to the sensor, so this is the body FRD-to-FLU conversion
        # and is independent of the configured world convention.
        body_vector = self.coords.body_vector
        points = [
            body_vector(point_cloud_airsim[i : i + 3])
            for i in range(0, len(point_cloud_airsim), 3)
        ]

        # Create PointCloud2 message from 3D point array
        header = rosstdmsg.Header()
        header.stamp = self.sim_time.stamp(projectairsim_lidar)
        header.frame_id = projectairsim_lidar["frame_id"]
        pointcloud2 = self.ros_node.PointCloud2.create_cloud_xyz32(header, points)

        return pointcloud2

    def convert_lidar_to_ros_transform(
        self, projectairsim_topic_name, projectairsim_msg
    ):
        """
        Returns the ROS transform to the sensor from the parent
        transform frame (usually the vehicle frame.)

        Arguments:
            projectairsim_topic_name - The Project AirSim topic name
            projectairsim_msg - The barometer data received from the Project AirSim topic

        Returns:
            (return) - Corresponding ROS Transform object
        """
        transform = rosgeommsg.Transform()
        projectairsim_pose = projectairsim_msg["pose"]
        (
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ) = self._world_position_list(projectairsim_pose["position"])
        (
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ) = self._world_quaternion_list(projectairsim_pose["orientation"])

        return transform

    def convert_magnetometer_to_ros(self, projectairsim_topic_name, projectairsim_msg):
        """
        Convert a Project AirSim magnetometer sensor message into a ROS MagneticField message.

        Arguments:
            projectairsim_topic_name - The Project AirSim topic name
            projectairsim_msg - The magnetometer data received from the Project AirSim topic

        Returns:
            (return) - Corresponding ROS MagneticField message
        """
        magnetic_field = rossensmsg.MagneticField()
        magnetic_field.header = self._get_standard_ros_header(
            projectairsim_topic_name, projectairsim_msg
        )

        magnetic_field.magnetic_field = self._body_vector3(
            projectairsim_msg["magnetic_field_body"]
        )

        magnetic_field.magnetic_field_covariance = self.NO_COVARIANCE_MATRIX
        projectairsim_covariance = projectairsim_msg["magnetic_field_covariance"]
        for i in range(0, min(len(projectairsim_covariance), 9)):
            magnetic_field.magnetic_field_covariance[i] = float(
                projectairsim_covariance[i]
            )

        return magnetic_field

    def convert_radar_detection_to_ros(
        self, projectairsim_topic_name, projectairsim_radar_detections
    ):
        """
        Convert a Project AirSim RADAR detections message into a ROS RadarScan message.

        Arguments:
            topic_pub_handler - Topic handler for this topic
            projectairsim_topic_name - The Project AirSim topic name
            projectairsim_radar_detections - The RADAR data received from the Project AirSim topic

        Returns:
            (return) - Corresponding ROS RadarScan message
        """
        radarscan = rosradarmsg.RadarScan()
        radarscan.header = self._get_standard_ros_header(
            projectairsim_topic_name, projectairsim_radar_detections
        )

        radar_returns = radarscan.returns
        rdProjectAirSim = projectairsim_radar_detections["radar_detections"]
        for i in range(len(rdProjectAirSim)):
            radar_detection = rdProjectAirSim[i]
            range_target = radar_detection["range"]

            radar_return = rosradarmsg.RadarReturn()
            radar_return.range = float(range_target)
            radar_return.azimuth = float(radar_detection["azimuth"])
            radar_return.elevation = float(radar_detection["elevation"])
            radar_return.doppler_velocity = float(radar_detection["velocity"])

            # Attempt to convert the radar cross-section to a signal amplitude
            # See: https://en.wikipedia.org/wiki/Radar_cross-section#Measurement
            radar_return.amplitude = 10 * math.log10(
                radar_detection["rcs_sqm"]
                * 1000  # Typical antenna gain (30 dB)
                / (
                    16.0
                    * math.pi
                    * math.pi
                    * range_target
                    * range_target
                    * range_target
                    * range_target
                )  # Spherical spread modeling of power density at transmitter and scattered by target
                * 0.7  # Typical large radar antenna aperture efficiency
                * 15
            )  # Antenna geometric area (meters squared)

            radar_returns.append(radar_return)

        return radarscan

    def convert_radar_track_to_ros(
        self, projectairsim_topic_name, projectairsim_radar_track
    ):
        """
        Converts the Project AirSim radar track data mesage into a ROS RadarTracks message.

        Arguments:
            topic_pub_handler - Topic handler for this topic
            projectairsim_topic_name - The Project AirSim topic name
            projectairsim_radar_track - The RADAR data received from the Project AirSim topic

        Returns:
            (return) - Corresponding ROS RadarTracks message
        """
        radartracks = rosradarmsg.RadarTracks()
        radartracks.header = self._get_standard_ros_header(
            projectairsim_topic_name, projectairsim_radar_track
        )

        tracks = radartracks.tracks
        rdProjectAirSim = projectairsim_radar_track["radar_tracks"]
        for i in range(len(rdProjectAirSim)):
            radar_track = rdProjectAirSim[i]

            radartrack = rosradarmsg.RadarTrack()

            (
                radartrack.position.x,
                radartrack.position.y,
                radartrack.position.z,
            ) = self._body_vector_list(radar_track["position_est"])
            (
                radartrack.velocity.x,
                radartrack.velocity.y,
                radartrack.velocity.z,
            ) = self._body_vector_list(radar_track["velocity_est"])
            (
                radartrack.acceleration.x,
                radartrack.acceleration.y,
                radartrack.acceleration.z,
            ) = self._body_vector_list(radar_track["accel_est"])

            tracks.append(radartrack)

        return radartracks

    def set_robot_base_frame_ids(self, robot_base_frame_ids: list):
        """
        Sets the mapping from Project AirSim topic name to transform frame IDs

        Arguments:
            robot_base_frame_ids - Dictionary mapping topic names to frame IDs
        """
        self.robot_base_frame_ids = robot_base_frame_ids

    def _get_standard_ros_header(
        self, projectairsim_topic_name: str, projectairsim_msg=None
    ):
        """
        Returns a ROS header stamped from the Project AirSim message's own
        simulation timestamp, with the frame ID set to the corresponding robot
        base's transform frame ID.

        Arguments:
            projectairsim_topic_name - The Project AirSim topic name
            projectairsim_msg - The Project AirSim topic message, whose
                time_stamp field provides the header timestamp
        """
        header = rosstdmsg.Header()
        header.stamp = self.sim_time.stamp(projectairsim_msg)
        header.frame_id = self.robot_base_frame_ids[
            projectairsim_topic_name
        ]

        return header
