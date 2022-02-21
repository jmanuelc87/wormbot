#include "control/control_hw_interface.h"

control_hw_interface::ControlHWInterface::ControlHWInterface() {}
control_hw_interface::ControlHWInterface::~ControlHWInterface() {}


bool control_hw_interface::ControlHWInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {

    JointStateHandle state_handle_lt("left_wheel", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_lt);

    JointStateHandle state_handle_rt("right_wheel", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_rt);

    registerInterface(&jnt_state_interface);

    JointHandle vel_handle_lt(jnt_state_interface.getHandle("left_wheel"), &cmd[0]);
    jnt_velocity_interface.registerHandle(vel_handle_lt);

    JointHandle vel_handle_rt(jnt_state_interface.getHandle("right_wheel"), &cmd[1]);
    jnt_velocity_interface.registerHandle(vel_handle_rt);

    registerInterface(&jnt_velocity_interface);

    std::string topic1 = root_nh.getNamespace() + "/drivers/set_motor_speed";
    std::string topic2 = root_nh.getNamespace() + "/drivers/get_motor_speed";

    speed_publisher = root_nh.advertise<drivers::Speed>(topic1, 10);

    return true;
}

bool control_hw_interface::ControlHWInterface::read()
{
    // read from ros topic
    return true;
}

bool control_hw_interface::ControlHWInterface::write() {

    double left_speed = jnt_velocity_interface.getHandle("left_wheel").getCommand();

    double right_speed = jnt_velocity_interface.getHandle("right_wheel").getCommand();

    ROS_INFO_THROTTLE(60, "%f, %f", left_speed, right_speed);

    drivers::Speed msg;
    msg.speedL = left_speed;
    msg.speedR = right_speed;

    speed_publisher.publish(msg);


    return true;
}