#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

enum direction {LEFT, RIGHT, FORWARD, NONE};

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    //Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Driving robot.");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if(!client.call(srv)) {
      ROS_ERROR("Failed to move robot");
    }
}

// Depending on the white ball position, call the drive_robot function and pass velocities to it
// Request a stop when there's no white ball seen by the camera
void execute_drive(direction move) {

	if(move == LEFT) {
		ROS_INFO("Move left");
		drive_robot(0.0, 0.5);
	} else if(move == FORWARD) {
		ROS_INFO("Move forward");
		drive_robot(0.5, 0.0);
	} else if(move == RIGHT) {
		ROS_INFO("Move right");
		drive_robot(0.0, -0.5);
	} else {
		ROS_INFO("Stopping");
		drive_robot(0.0,0.0);
	}
}

// Identify if this pixel falls in the left, mid, or right side of the image
direction determine_direction(int position, int img_width) {
	int location = position - (((int)position/img_width) * img_width);
	direction move;
	if(location > 0 && location <= (img_width * .35)) {
		move = LEFT;
	} else if (location > (img_width * .35) && location <= (img_width * .65)) {
		move = FORWARD;
	} else if (location > (img_width * .65) && location <= img_width) {
		move = RIGHT;
	} else {
		move = NONE;
	}
	return move;
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    bool foundWhiteBall = false;
    int ball_position = 0;

    // Loop through each pixel in the image and check if there's a bright white one
    for (int i = 0; i < img.height * img.step; i++) {
        if (img.data[i] == white_pixel) {
            foundWhiteBall = true;
	          ball_position = i;
            ROS_INFO("Found white ball at %1.2f", (float)i);
            break;
        }
    }
    direction move = determine_direction(ball_position, img.step);

    execute_drive(move);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
