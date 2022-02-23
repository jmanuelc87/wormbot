#ifndef CONTROL_HW_INTERFACE_HPP
#define CONTROL_HW_INTERFACE_HPP

#include <math.h>
#include <string>
#include <ros/console.h>

#include "drivers/SpeedMessage.h"
#include "drivers/SpeedCommand.h"

#include "hardware_interface/robot_hw.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"

using namespace hardware_interface;


namespace control_hw_interface
{
    class ControlHWInterface : public RobotHW
    {
    private:
        JointStateInterface jnt_state_interface;
        VelocityJointInterface jnt_velocity_interface;

        double left_wheel_velocity_cmd = 0;
        double right_wheel_velocity_cmd = 0;

        double left_wheel_position_state = 0;
        double right_wheel_position_state = 0;

        double left_wheel_velocity_state = 0;
        double right_wheel_velocity_state = 0;

        double left_effort = 0;
        double right_effort = 0;

        double wheel_radius = 0;

        ros::Publisher set_motor_speed;
        ros::ServiceClient get_motor_speed;

    public:
        ControlHWInterface();
        ~ControlHWInterface();

        bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);

        bool read(ros::Time timestamp, ros::Duration period);

        bool write();
    };
}

#endif