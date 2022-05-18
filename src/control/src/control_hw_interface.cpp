#include "control/control_hw_interface.h"

control_hw_interface::ControlHWInterface::ControlHWInterface() {}
control_hw_interface::ControlHWInterface::~ControlHWInterface() {}


bool control_hw_interface::ControlHWInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {

    JointStateHandle state_handle_lt("left_wheel", &left_wheel_position_state, &left_wheel_velocity_state, &left_effort);
    jnt_state_interface.registerHandle(state_handle_lt);

    JointStateHandle state_handle_rt("right_wheel", &right_wheel_position_state, &right_wheel_velocity_state, &right_effort);
    jnt_state_interface.registerHandle(state_handle_rt);

    registerInterface(&jnt_state_interface);

    JointHandle vel_handle_lt(jnt_state_interface.getHandle("left_wheel"), &left_wheel_velocity_cmd);
    jnt_velocity_interface.registerHandle(vel_handle_lt);

    JointHandle vel_handle_rt(jnt_state_interface.getHandle("right_wheel"), &right_wheel_velocity_cmd);
    jnt_velocity_interface.registerHandle(vel_handle_rt);

    registerInterface(&jnt_velocity_interface);

    std::string topic1 = root_nh.getNamespace() + "/drivers/set_motor_speed";
    std::string topic2 = root_nh.getNamespace() + "/drivers/get_motor_speed";

    ROS_INFO("%s %s", topic1.c_str(), topic2.c_str());

    set_motor_speed = root_nh.advertise<drivers::SpeedMessage>(topic1, 10);
    get_motor_speed = root_nh.serviceClient<drivers::SpeedCommand>(topic2);

    root_nh.getParam("/wormbot/movement_controller/wheel_radius", wheel_radius);

    ROS_INFO("Radius %f", wheel_radius);

    return true;
}

bool control_hw_interface::ControlHWInterface::read(ros::Time timestamp, ros::Duration period)
{
   drivers::SpeedCommand srv;

    srv.request.driver = 1;

    if (get_motor_speed.call(srv)) {
        const float left_wheel_rpm = srv.response.speedL;
        const float right_wheel_rpm = srv.response.speedR;

        const double left_wheel_rads = left_wheel_rpm * ((2 * M_PI) / 60);
        const double right_wheel_rads = right_wheel_rpm * ((2 * M_PI) / 60);

        const double left_wheel_linear = left_wheel_rads * wheel_radius;
        const double right_wheel_linear = right_wheel_rads * wheel_radius;

        left_wheel_position_state = left_wheel_linear * period.toSec();
        right_wheel_position_state = right_wheel_linear * period.toSec();

        left_wheel_velocity_state = left_wheel_rads;
        right_wheel_velocity_state = right_wheel_rads;

        ROS_INFO_THROTTLE_NAMED(120, "read", "%0.2f, %0.2f, %0.2f, %0.2f", left_wheel_position_state, left_wheel_velocity_state, right_wheel_position_state, right_wheel_velocity_state);

        return true;
    }
    else {
        ROS_ERROR_THROTTLE(360, "Failed to call the service get_motor_speed");
        return false;
    }

    return true;
}

bool control_hw_interface::ControlHWInterface::write()
{
    // conversion from rad/s to rpm
    double left_speed = left_wheel_velocity_cmd * (60 / (2 * M_PI));

    double right_speed = right_wheel_velocity_cmd * (60 / (2 * M_PI));

    ROS_INFO_THROTTLE_NAMED(120, "write", "%0.2f, %0.2f", left_speed, right_speed);

    drivers::SpeedMessage msg;
    msg.speedL = round(left_speed);
    msg.speedR = round(right_speed);

    set_motor_speed.publish(msg);

    return true;
}