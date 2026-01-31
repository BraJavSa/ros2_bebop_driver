#include "ros2_bebop_driver/bebop_driver_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>

#include <chrono>
#include <cstdio>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace std::chrono_literals;

namespace bebop_driver {

#define SIMPLECALLBACK(subscriber, msgtype, topic, method)              \
    subscriber = this->create_subscription<msgtype>(                    \
	topic, 1,                                                       \
	[this]([[maybe_unused]] const msgtype::SharedPtr msg) -> void { \
	    this->bebop->method();                                      \
	});

BebopDriverNode::BebopDriverNode()
    : rclcpp::Node("bebop_driver_node"),
      bebop(std::make_shared<Bebop>()),
      last_odom_time({}) {
    {
	// Make std cout/cerr unbuffered
	// This can create performance issues
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
	param_desc.description = "The IP of the bebop to connect to";

	this->declare_parameter("bebop_ip", "192.168.42.1", param_desc);
    }
    {
	auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
	param_desc.description = "The port of the bebop to connect to";

	this->declare_parameter("bebop_port", 44444, param_desc);
    }

    cinfo_manager =
	std::make_shared<camera_info_manager::CameraInfoManager>(this);
    {
	/* get ROS2 config parameter for camera calibration file */
	auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
	param_desc.description = "The path to the yaml camera calibration file";
	auto camera_calibration_file_param = this->declare_parameter(
	    "camera_calibration_file",
	    "package://ros2_bebop_driver/config/bebop2_camera_calib.yaml");
	cinfo_manager->setCameraName("bebop_front");
	cinfo_manager->loadCameraInfo(camera_calibration_file_param);
    }
    {
	auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
	param_desc.description = "The frame id of the camera";
	camera_frame_id =
	    this->declare_parameter("camera_frame_id", "camera_optical");
    }
    {
	auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
	param_desc.description =
	    "The frame id of the odometry frame of reference";
	odom_frame_id = this->declare_parameter("odom_frame_id", "odom");
    }

    auto bebop_ip = this->get_parameter("bebop_ip").as_string();
    auto bebop_port = this->get_parameter("bebop_port").as_int();
    RCLCPP_INFO(this->get_logger(), "Connecting to the bebop %s:%d",
		bebop_ip.c_str(), bebop_port);
    bebop->connect(bebop_ip, bebop_port);
    RCLCPP_INFO(this->get_logger(), "Connected");

    // The core functions subscribers
    // For: TakeOff, Land, Emergency, flatTrim, navigateHome, animationFlip,

    // move and moveCamera
    // TakeOff
    SIMPLECALLBACK(subscription_takeoff, std_msgs::msg::Empty, "takeoff",
		   takeOff);
    // Landing
    SIMPLECALLBACK(subscription_land, std_msgs::msg::Empty, "land", land);

    // Emergency
    SIMPLECALLBACK(subscription_emergency, std_msgs::msg::Empty, "reset",
		   emergency);

    // FlatTrim
    SIMPLECALLBACK(subscription_flattrim, std_msgs::msg::Empty, "flattrim",

		   flatTrim);

    // navigateHome
    subscription_navigateHome = this->create_subscription<std_msgs::msg::Bool>(
	"autoflight/navigate_home", 1,
	[this]([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr msg)
	    -> void { this->bebop->navigateHome(msg->data); });

    // animationFlip
    subscription_animationFlip =
	this->create_subscription<std_msgs::msg::UInt8>(
	    "flip", 1,
	    [this](const std_msgs::msg::UInt8::SharedPtr msg) -> void {
		this->bebop->animationFlip(msg->data);
	    });

    // cmdvel
    subscription_cmdVel = this->create_subscription<geometry_msgs::msg::Twist>(
	"cmd_vel", 1,
	std::bind(&BebopDriverNode::cmdVelCallback, this,
		  std::placeholders::_1));
    
    // moveCamera
    subscription_moveCamera = this->create_subscription<geometry_msgs::msg::Vector3>(
    "move_camera", 1,
    [this](const geometry_msgs::msg::Vector3::SharedPtr msg) -> void {
        this->bebop->moveCamera(msg->x, msg->y);
    });

    // photo
    subscription_photo = this->create_subscription<std_msgs::msg::Bool>(
    "photo", 1,
    [this](const std_msgs::msg::Bool::SharedPtr msg) -> void {
        this->bebop->photo(msg->data);
    });


    // Camera
    publisher_camera =
	image_transport::create_camera_publisher(this, "camera/image_raw");
    bebop->startStreaming();
    if (bebop->isStreamingStarted()) {
	// Camera info publication on a regular basis
	camera_timer = this->create_wall_timer(
	    30ms, std::bind(&BebopDriverNode::publishCamera, this));
	RCLCPP_INFO(this->get_logger(), "Streaming is started");
    } else {
	RCLCPP_ERROR(this->get_logger(), "Failed to start streaming");
    }

    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Odometry: published at 15Hz
    publisher_odometry =
	this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    odom_timer = this->create_wall_timer(
	66ms, std::bind(&BebopDriverNode::publishOdometry, this));

    // Position publisher at 15Hz (from odometry)
    publisher_position =
	this->create_publisher<geometry_msgs::msg::PointStamped>("position", 10);

    // Altitude publisher at 10 Hz (barometer)
    publisher_altitude =
	this->create_publisher<std_msgs::msg::Float64>("altitude", 10);
    altitude_timer = this->create_wall_timer(
	100ms, std::bind(&BebopDriverNode::publishAltitude, this));

    // GPS publisher at 10 Hz
    publisher_gps =
	this->create_publisher<sensor_msgs::msg::NavSatFix>("gps", 10);
    gps_timer = this->create_wall_timer(
	100ms, std::bind(&BebopDriverNode::publishGps, this));

    // Flying state publisher at 10 Hz
    publisher_flying_state =
	this->create_publisher<std_msgs::msg::UInt8>("flying_state", 10);
    flying_state_timer = this->create_wall_timer(
	100ms, std::bind(&BebopDriverNode::publishFlyingState, this));

    // IMU publisher at 30 Hz
    publisher_imu =
	this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    imu_timer = this->create_wall_timer(
	33ms, std::bind(&BebopDriverNode::publishImu, this));

    // Velocity publisher at 15 Hz
    publisher_velocity =
	this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_feedback", 10);
    velocity_timer = this->create_wall_timer(
	66ms, std::bind(&BebopDriverNode::publishVelocity, this));
}

void BebopDriverNode::publishCamera(void) {
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg(
	new sensor_msgs::msg::CameraInfo(cinfo_manager->getCameraInfo()));

    rclcpp::Time timestamp = this->get_clock()->now();

    camera_info_msg->header.stamp = timestamp;
    camera_info_msg->header.frame_id = camera_frame_id;

    sensor_msgs::msg::Image::SharedPtr img_msg =
	std::make_shared<sensor_msgs::msg::Image>();

    auto header = std::make_shared<std_msgs::msg::Header>();
    img_msg->header.stamp = timestamp;
    img_msg->header.frame_id = camera_frame_id;
    this->bebop->getFrontCameraFrame(img_msg->data, img_msg->width,
				     img_msg->height);
    img_msg->encoding = sensor_msgs::image_encodings::BGR8;
    /* img_msg->is_bigendian = ; TODO*/
    img_msg->step = 3 * img_msg->width;

    publisher_camera.publish(img_msg, camera_info_msg);
}

void BebopDriverNode::publishOdometry(void) {
    if (!last_odom_time) {
	last_odom_time = this->get_clock()->now();
	return;
    }

    // Reads the values received from the callback
    /* std::string speed_frame_id; */
    /* bebop_driver::time_point speed_time; */
    /* float beb_vx_enu, beb_vy_enu, beb_vz_enu; */
    /* auto [speed_frame_id, speed_time, beb_vx_enu, beb_vy_enu, beb_vz_enu] =
     */
    /* std::tie(speed_frame_id, speed_time, beb_vx_enu, beb_vy_enu, beb_vz_enu)
     * = */
    /* bebop->getArdrone3PilotingStateSpeed(); */

    auto [speed_frame_id, speed_time, beb_vx_enu, beb_vy_enu, beb_vz_enu] =
	bebop->getArdrone3PilotingStateSpeed();
    RCLCPP_DEBUG(this->get_logger(),
		 "I received Piloting State Speed : %s, %f, %f, %f",
		 speed_frame_id.c_str(), beb_vx_enu, beb_vy_enu, beb_vz_enu);

    auto [attitude_frame_id, attitude_time, beb_roll, beb_pitch, beb_yaw] =
	bebop->getArdrone3PilotingStateAttitude();
    RCLCPP_DEBUG(this->get_logger(),
		 "I received Piloting State Attitude : %s, %f, %f, %f",
		 attitude_frame_id.c_str(), beb_roll, beb_pitch, beb_yaw);

    auto time = std::max(speed_time, attitude_time).time_since_epoch();
    auto ros_stamp = rclcpp::Time(
	std::chrono::duration_cast<std::chrono::seconds>(time).count(),
	std::chrono::duration_cast<std::chrono::nanoseconds>(time).count() %
	    1000000000UL);

    beb_vy_enu = -beb_vy_enu;
    beb_vz_enu = -beb_vz_enu;
    beb_pitch = -beb_pitch;
    beb_yaw = -beb_yaw;

    auto beb_vx_m = cos(beb_yaw) * beb_vx_enu + sin(beb_yaw) * beb_vy_enu;
    auto beb_vy_m = -sin(beb_yaw) * beb_vx_enu + cos(beb_yaw) * beb_vy_enu;
    auto beb_vz_m = beb_vz_enu;

    // Update the TF message content
    tf_odom_to_base.header.stamp = ros_stamp;
    tf_odom_to_base.header.frame_id = odom_frame_id;
    tf_odom_to_base.child_frame_id = "base_link";

    auto now = this->get_clock()->now();
    auto dt = (now - *last_odom_time).seconds();
    tf_odom_to_base.transform.translation.x += beb_vx_enu * dt;
    tf_odom_to_base.transform.translation.y += beb_vy_enu * dt;
    tf_odom_to_base.transform.translation.z += beb_vz_enu * dt;

    tf2::Quaternion q;
    q.setRPY(beb_roll, beb_pitch, beb_yaw);
    tf_odom_to_base.transform.rotation.x = q.x();
    tf_odom_to_base.transform.rotation.y = q.y();
    tf_odom_to_base.transform.rotation.z = q.z();
    tf_odom_to_base.transform.rotation.w = q.w();

    // Broadcast the TF
    tf_broadcaster->sendTransform(tf_odom_to_base);

    auto odom_message = nav_msgs::msg::Odometry();
    odom_message.header.stamp = ros_stamp;
    odom_message.header.frame_id = odom_frame_id;
    odom_message.child_frame_id = "base_link";

    // The position and orientation
    odom_message.pose.pose.position.x = tf_odom_to_base.transform.translation.x;
    odom_message.pose.pose.position.y = tf_odom_to_base.transform.translation.y;
    odom_message.pose.pose.position.z = tf_odom_to_base.transform.translation.z;
    tf2::convert(tf_odom_to_base.transform.rotation,
		 odom_message.pose.pose.orientation);
    /*TODO odom_message.pose.covariance = ; */

    // The velocities
    odom_message.twist.twist.linear.x = beb_vx_m;
    odom_message.twist.twist.linear.y = beb_vy_m;
    odom_message.twist.twist.linear.z = beb_vz_m;
    /*TODO odom_message.twist.twist.angular = {0.0, 0.0, 0.0}; */
    /*TODO odom_message.twist.covariance = ; */

    publisher_odometry->publish(odom_message);
    
    // Publish position
    auto position_msg = geometry_msgs::msg::PointStamped();
    position_msg.header.stamp = ros_stamp;
    position_msg.header.frame_id = odom_frame_id;
    position_msg.point.x = tf_odom_to_base.transform.translation.x;
    position_msg.point.y = tf_odom_to_base.transform.translation.y;
    position_msg.point.z = tf_odom_to_base.transform.translation.z;
    publisher_position->publish(position_msg);
    
    last_odom_time = now;
}

void BebopDriverNode::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
    double roll = -msg->linear.y;
    double pitch = msg->linear.x;
    double gaz_speed = msg->linear.z;
    double yaw_speed = -msg->angular.z;
    bebop->move(roll, pitch, gaz_speed, yaw_speed);
}

void BebopDriverNode::publishAltitude(void) {
    auto [frame_id, time, altitude] = bebop->getArdrone3AltitudeChanged();
    RCLCPP_DEBUG(this->get_logger(), "Altitude (barometer): %f m", altitude);

    auto altitude_msg = std_msgs::msg::Float64();
    altitude_msg.data = altitude;

    publisher_altitude->publish(altitude_msg);
}

void BebopDriverNode::publishGps(void) {
    auto [frame_id, time, latitude, longitude, altitude] =
        bebop->getArdrone3GpsLocationChanged();
    RCLCPP_DEBUG(this->get_logger(), "GPS: lat=%f, lon=%f, alt=%f", latitude,
                 longitude, altitude);

    auto gps_msg = sensor_msgs::msg::NavSatFix();
    gps_msg.header.stamp = this->get_clock()->now();
    gps_msg.header.frame_id = "gps";
    gps_msg.latitude = latitude;
    gps_msg.longitude = longitude;
    gps_msg.altitude = altitude;
    gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    publisher_gps->publish(gps_msg);
}

void BebopDriverNode::publishFlyingState(void) {
    auto [frame_id, time, state] = bebop->getArdrone3FlyingStateChanged();
    RCLCPP_DEBUG(this->get_logger(), "Flying state: %d", state);

    auto state_msg = std_msgs::msg::UInt8();
    state_msg.data = state;

    publisher_flying_state->publish(state_msg);
}

void BebopDriverNode::publishImu(void) {
    auto [attitude_frame_id, attitude_time, roll, pitch, yaw] =
        bebop->getArdrone3PilotingStateAttitude();
    auto [speed_frame_id, speed_time, vx, vy, vz] =
        bebop->getArdrone3PilotingStateSpeed();
    
    RCLCPP_DEBUG(this->get_logger(), "IMU: roll=%f, pitch=%f, yaw=%f, vel=(%f, %f, %f)", roll,
                 pitch, yaw, vx, vy, vz);

    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";

    // Set orientation from attitude (roll, pitch, yaw -> quaternion)
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = -q.y();
    imu_msg.orientation.z = -q.z();
    imu_msg.orientation.w = q.w();

    // Orientation covariance
    imu_msg.orientation_covariance[0] = 0.01;  // roll variance
    imu_msg.orientation_covariance[4] = 0.01;  // pitch variance
    imu_msg.orientation_covariance[8] = 0.01;  // yaw variance

    // Angular velocity (estimated from acceleration, not directly available from Bebop)
    // For now, we set to zero as Bebop doesn't provide raw angular velocity
    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;
    imu_msg.angular_velocity_covariance[0] = -1;  // Not available

    // Linear acceleration (estimated from velocity changes)
    // Apply coordinate frame transformation to match standard ROS conventions
    imu_msg.linear_acceleration.x = vx * 0.5;   // Forward/backward
    imu_msg.linear_acceleration.y = vy * 0.5;   // Left/right  
    imu_msg.linear_acceleration.z = vz;  // Up/down

    // Linear acceleration covariance
    imu_msg.linear_acceleration_covariance[0] = 0.1;
    imu_msg.linear_acceleration_covariance[4] = 0.1;
    imu_msg.linear_acceleration_covariance[8] = 0.1;

    publisher_imu->publish(imu_msg);
}

void BebopDriverNode::publishVelocity(void) {
    auto [speed_frame_id, speed_time, vx, vy, vz] =
        bebop->getArdrone3PilotingStateSpeed();
    
    RCLCPP_DEBUG(this->get_logger(), "Velocity: vx=%f, vy=%f, vz=%f", vx, vy, vz);

    auto vel_msg = geometry_msgs::msg::Twist();
    
    // Linear velocities
    vel_msg.linear.x = vx;
    vel_msg.linear.y = vy;
    vel_msg.linear.z = vz;
    
    // Angular velocities (not available from Bebop)
    vel_msg.angular.x = 0.0;
    vel_msg.angular.y = 0.0;
    vel_msg.angular.z = 0.0;

    publisher_velocity->publish(vel_msg);
}

}  // namespace bebop_driver

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bebop_driver::BebopDriverNode>());
    rclcpp::shutdown();
    return 0;
}

