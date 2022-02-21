#ifndef CONTROL_HW_INTERFACE_HPP
#define CONTROL_HW_INTERFACE_HPP

#include <string>
#include <ros/console.h>

#include "drivers/Speed.h"

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
        PositionJointInterface jnt_position_interface;

        double cmd[2];
        double pos[2];
        double vel[2];
        double eff[2];

        ros::Publisher speed_publisher;
        ros::Subscriber speed_subscriber;

    public:
        ControlHWInterface();
        ~ControlHWInterface();

        bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);

        bool read();

        bool write();
    };
}

#endif