#include <ros/ros.h>

#include <std_srvs/SetBool.h>

int main(int argc, char** argv){

	ros::init(argc, argv, "sticky_fingers_control"); // name this node 
	ros::NodeHandle nh;

	ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>(argv[1]);
	std_srvs::SetBool srv;

	if(argc != 3){
		ROS_INFO("Dummy gripper control node. USAGE: ");
		ROS_INFO("\"finger_control_dummy_node [message name] ['true' / 'sticky' OR 'false' / 'smooth']\"");
		return 1;
	}
	
	if(strcmp(argv[2], "false") == 0 || strcmp(argv[2], "smooth") == 0){
		ROS_INFO("Setting finger status to FALSE (smooth).");
		srv.request.data = false;
	}
	else if(strcmp(argv[2], "true") == 0 || strcmp(argv[2], "sticky") == 0){
		ROS_INFO("Setting finger status to TRUE (sticky).");
		srv.request.data = true;
	}
	else{
		ROS_INFO("Dummy gripper control node. USAGE: ");
		ROS_INFO("\"finger_control_dummy_node [message name] ['true' / 'sticky' OR 'false' / 'smooth']\"");
		return 1;
	}
	
	while(!client.call(srv) && ros::ok()){
		ROS_INFO("Sending...");
		ros::spinOnce();
	}

	ROS_INFO("Messages sent.");
	return 0;
}
