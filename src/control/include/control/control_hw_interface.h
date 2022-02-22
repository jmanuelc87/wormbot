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
        double left_wheel_velocity_cmd;
        double right_wheel_velocity_cmd;

        double left_wheel_position_state;
        double right_wheel_position_state;

        double left_wheel_velocity_state;
        double right_wheel_velocity_state;

        double wheel_radius;

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