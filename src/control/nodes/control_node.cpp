
#include "ros/ros.h"

#include "controller_manager/controller_manager.h"
#include "control/control_hw_interface.h"

using namespace controller_manager;
using namespace control_hw_interface;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "control_node");

	ros::NodeHandle root_nh;

	ControlHWInterface hw;
	ControllerManager manager(&hw, root_nh);

	hw.init(root_nh, root_nh);

	ros::AsyncSpinner spinner(2);

	spinner.start();

	ros::Rate loop(30);

	ros::Time timestamp = ros::Time::now();
	ros::Duration period;

	while (ros::ok())
	{
		period = ros::Time::now() - timestamp;

		hw.read(timestamp, period);

		manager.update(timestamp, period);

		hw.write();

		timestamp = ros::Time::now();

		loop.sleep();
	}

	spinner.stop();

	return 0;
}
