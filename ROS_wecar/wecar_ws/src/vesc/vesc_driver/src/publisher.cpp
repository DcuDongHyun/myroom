#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "publisher");
	ros::NodeHandle nh;

	ros::Publisher speed = nh.advertise<std_msgs::Float64>("commands/motor/speed", 10);
	ros::Publisher position = nh.advertise<std_msgs::Float64>("commands/servo/position", 10);

	std_msgs::Float64 speed_msg;
	std_msgs::Float64 position_msg;

	ros::Rate rate(10);

	double i = 0;
	speed_msg.data = 2200.0;
	while(ros::ok())
	{
		position_msg.data = i;

		speed.publish(speed_msg);
		position.publish(position_msg);

		i += 0.1;
		if(i > 1)
		{
			i = 0;
			speed_msg.data *= -1;
		}
			

		rate.sleep();
	}

	return 0;
}
