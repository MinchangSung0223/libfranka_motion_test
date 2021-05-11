// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include "examples_common.h"
#include <stdio.h>
#include <signal.h>
void ctrlchandler(int){exit(EXIT_SUCCESS);}
void killhandler(int){exit(EXIT_SUCCESS);}
int trig_command = 0;
std::array<double, 7> q_goal = {{0, 0, 0, 0, 0, 0, 0}};
franka::Robot* robot;
void rosJointTrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
	//std::cout<<"-----------------------------------------------------"<<std::endl;
	trig_command++;
	for(int i = 0;i<7;i++){
		//std::cout<<msg->joint_names[i]<<" : "<<msg->points[1].positions[i]<<std::endl;	
		q_goal[i] = msg->points[0].positions[i];
		}
	//std::cout<<"-----------------------------------------------------"<<std::endl;
}



int main(int argc, char** argv) {
	signal(SIGINT, ctrlchandler);
	signal(SIGTERM, killhandler);
	int use_simulation = atoi(argv[2]);
	if (argc != 3) {
		std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
		return -1;
	}
	std::cerr << "Robot FCI IP: " << argv[1]  << std::endl;

	if(use_simulation!=1){
		try{
			std::cerr << "Waiting Robot..."  << std::endl;
			*robot=franka::Robot(argv[1]);
			setDefaultBehavior(*robot);
			std::cerr << "Robot is Connected "  << std::endl;
		}
		catch(const franka::Exception& e) {
			std::cerr << "Robot is not Connected "  << std::endl;
		}
	}

	try{
		ros::init(argc,argv,"trajectory_test_sub");
	}
	catch(int e){
		ctrlchandler(1);
	
	}
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("joint_trajectory", 1,rosJointTrajectoryCallback); 
	ros::Rate r(30);
	std::cout<<"ROS JOINT TRAJECTORY SUBSCRIBER IS ON"<<std::endl;
	while (ros::ok()){
		if(trig_command==1){
			if(use_simulation!=1){
				std::cout<<"--------------q_goal-------------"<<std::endl;								
				std::cout<<q_goal[0]<<","<<q_goal[1]<<","<<q_goal[2]<<","<<q_goal[3]<<","<<q_goal[4]<<","<<q_goal[5]<<","<<q_goal[6]<<std::endl;
				std::cout<<"---------------------------------"<<std::endl;
				try{
					MotionGenerator motion_generator(0.5, q_goal);
					std::cin.ignore();
					robot->control(motion_generator);
				}
				catch(const franka::Exception& e){
					std::cout << e.what() << std::endl;
					return -1;
				}
			}	
			else{
				std::cout<<"--------------q_goal-------------"<<std::endl;
				std::cout<<q_goal[0]<<","<<q_goal[1]<<","<<q_goal[2]<<","<<q_goal[3]<<","<<q_goal[4]<<","<<q_goal[5]<<","<<q_goal[6]<<std::endl;
				std::cout<<"---------------------------------"<<std::endl;
			}
		}
		else if(trig_command>1){break;}
		ros::spinOnce();
	}
/*
  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, 0, 0, 0, 0, 0, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    std::array<double, 7> initial_position;
    double time = 0.0;
    robot.control([&initial_position, &time](const franka::RobotState& robot_state,
                                             franka::Duration period) -> franka::JointPositions {
      time += period.toSec();
      if (time == 0.0) {
        initial_position = robot_state.q_d;
      }
      double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * time));
      franka::JointPositions output = {{initial_position[0], initial_position[1],
                                        initial_position[2], initial_position[3],
                                        initial_position[4] , initial_position[5],
                                        initial_position[6] + delta_angle}};
      if (time >= 5.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
*/
  return 0;
}
