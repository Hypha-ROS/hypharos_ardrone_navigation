/*
# Copyright 2018 HyphaROS Workshop
# Original Author: HaoChih, LIN (f44006076@gmail.com)
# Maintainer: HyphaROS Workshop (hypha.ros@gmail)
# Maintainer: PouChun, Kung (k3083518729@gmail.com) 
#
# Website: https://hypharosworkshop.wordpress.com/
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include "ros/ros.h"
#include "ros/service_callback_helper.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "tum_ardrone/filter_state.h"
#include <string>
#include <sstream>
#include <geometry_msgs/Twist.h>       //For motion command, except takeoff, land

//Kbhit() and Keyboard()
#include <termios.h>                   
#include <stdio.h>   
#include <unistd.h>  
#include <fcntl.h> 

#include <moveit_msgs/DisplayTrajectory.h> 
#include <math.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

//MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//Define parameters
#define ptam_x      0   // Current x-position from ptam
#define ptam_y      1   // Unit of position is 'm'
#define ptam_z	    2
#define ptam_yaw    3   // Unit of yaw is degree, range is from -180 ~ 180
#define PI 3.14159265358979

//--------------kbhit function model in Linux----------------
int kbhit(void)  
{  
  struct termios oldt, newt;  
  int ch;  
  int oldf;  
  tcgetattr(STDIN_FILENO, &oldt);  
  newt = oldt;  
  newt.c_lflag &= ~(ICANON | ECHO);  
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);  
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);  
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);  
  ch = getchar();  
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  
  fcntl(STDIN_FILENO, F_SETFL, oldf);  
  if(ch != EOF)  
  {  
    ungetc(ch, stdin);  
    return 1;  
  }  
  return 0;  
}  


//------Set a "class" for different kinds of functions--------
class ArdroneControl
{
  public:
    ArdroneControl();

  private:
    void takeoff();
    void land();
    void clear_targets();
    void Free_pointers();
    void Manual(int com_char);
    void targets2tum_ardrone(float x, float y, float z, float yaw);
    void Plan_request(float end_x, float end_y, float end_yaw);
    float Quaternion2Yaw(float qz, float qw);
    int keyboard(); 
    int index2;
    int path_follow_index;
    int clear_path;
    float Position[4];
    float Target[4];
    float *path_x;
    float *path_y; 
    float *path_z;
    float *path_w;
    float goal_x;
    float goal_y;
    float goal_yaw;
    float manual_com[4];
    //float min_corner[3];
    //float max_corner[3];
    int replan_index;

    moveit_msgs::DisplayTrajectory display_trajectory;

    char manual_selection;

    ros::NodeHandle n;                               // so that we dont make a copy while passing this as reference
    ros::Subscriber nav_sub;                         // subscribing to the pos_estimate_data from the tum_ardrone topic
    ros::Subscriber lsd_PC2_sub;
    ros::Subscriber path_sub;			     // subscribing to the display_planned_path from Moveit topic
    ///ros::Publisher cmd_vel_pub;                      // this will be publishing the command velocity to the drone
    ros::Publisher tum_ardrone_pub;		     // this will be publishing the Target to the TUM package
    ///ros::Publisher display_pub;
    geometry_msgs::Twist twist_manual, twist_pos;    // the message we use to send command to the drone
    
    void nav_callback(const tum_ardrone::filter_stateConstPtr tum_msg);
    void path_callback(const moveit_msgs::DisplayTrajectory msg);
    void lsd_PC2_callback(const sensor_msgs::PointCloud2ConstPtr PC2_msg);
};


//-------Set up all members in the "ArdroneControl class"------
ArdroneControl::ArdroneControl()
{
  ///cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  nav_sub = n.subscribe("/ardrone/predictedPose", 50, &ArdroneControl::nav_callback, this);				/// interrupption for UI
  path_sub = n.subscribe("/move_group/display_planned_path", 1, &ArdroneControl::path_callback, this);			/// path following
  lsd_PC2_sub = n.subscribe("/pointcloud2", 1, &ArdroneControl::lsd_PC2_callback, this);				/// interruption to send replan request
  ///display_pub = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  tum_ardrone_pub = n.advertise<std_msgs::String>("tum_ardrone/com",50, true);

  twist_manual.linear.x = twist_manual.linear.y = twist_manual.linear.x = 0.0;
  Target[ptam_x] = Target[ptam_y] = Target[ptam_z] = Target[ptam_yaw] = 0.0;
  goal_x = goal_y = goal_yaw = 0.0;
  manual_com[ptam_x] = manual_com[ptam_y] = manual_com[ptam_z] = manual_com[ptam_yaw] = 0.0;

  index2 = 0;
  path_follow_index = 0;
  replan_index = 0;
  clear_path = 0;

  path_x = (float*)malloc(20 * sizeof(float));
  path_y = (float*)malloc(20 * sizeof(float));
  path_z = (float*)malloc(20 * sizeof(float));
  path_w = (float*)malloc(20 * sizeof(float));
}

void ArdroneControl::takeoff()
{
  ros::Rate poll_rate(100);

  ros::Publisher takeoff = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1, true);
  takeoff.publish(std_msgs::Empty());

  while(takeoff.getNumSubscribers() == 0)
      poll_rate.sleep();  
  takeoff.shutdown();
 ROS_INFO("TAKEOFF");
}

void ArdroneControl::land()
{
  ros::Rate poll_rate(100);

  ros::Publisher landing = n.advertise<std_msgs::Empty>("ardrone/land", 1, true);
  landing.publish(std_msgs::Empty());

  while(landing.getNumSubscribers() == 0)
      poll_rate.sleep();  
  landing.shutdown();

  ROS_INFO("LAND");
}


//----------Function for clear all targets in TUM_ARDrone interface-------------
void ArdroneControl::clear_targets()
{
	std_msgs::String msg;
	std::stringstream ss;
	ss << "c clearCommands";
	msg.data = ss.str();
	tum_ardrone_pub.publish(msg);
}


//----------Function for sending each target to the ARDrone--------------------
void ArdroneControl::targets2tum_ardrone(float x, float y, float z, float yaw)
{
    std::stringstream ss;
    ss <<"c goto " << x << " " << y << " " << z << " "<< "0";
    std::string s = ss.str();
    ROS_INFO(s.c_str());
    std_msgs::String com;
    com.data = s.c_str();
    tum_ardrone_pub.publish(com);
}

//------Function for detecting the keyboard input without "Enter"----------
int ArdroneControl::keyboard()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
  int c = getchar();                         // read character (non-blocking)
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}


//-------Function for Manual Control-----------
void ArdroneControl::Manual(int com_char)
{
	if(ros::ok())
	{
		switch(com_char)
		{
			case 't':
				takeoff();
			break;

			case 'r':			
				replan_index = 1;
				ROS_INFO("Start to replan!!\n");
			break;

			case 'l':	
				land();
			break;

			case 'c':		
				clear_path = 1;
				ROS_INFO("Clear previous path\n");
			break;

			case 'k':	
				clear_path = 0;
				ROS_INFO("Keep previous path\n");
			break;
			
			case 'p':  //follow the path	
				clear_targets();				
				path_follow_index = 1;
				ROS_INFO("Start to follow the path\n!!");
			break;

			case 'i':  //Stop following the path and replan functions	
				clear_targets();				
				path_follow_index = 0;
				replan_index = 0;
				ROS_INFO("Stop replanning and following the path\n!!");
			break;

            		case 'o': //back to the origine
				clear_targets();
				path_follow_index = 0;	
				replan_index = 0;			
				targets2tum_ardrone(0.0, 0.0, 0.0 ,0.0);
				manual_com[ptam_x] = manual_com[ptam_y] = manual_com[ptam_z] = 0.0;				
				ROS_INFO("back to the origine\n");		
			break;

            		case 'w': //forward
				clear_targets();
				manual_com[ptam_y] = manual_com[ptam_y] + 0.5;
				targets2tum_ardrone(manual_com[ptam_x],manual_com[ptam_y],manual_com[ptam_z],manual_com[ptam_yaw]);				
				ROS_INFO("move forward\n");		
			break;

            		case 's': //backward
				clear_targets();
				manual_com[ptam_y] = manual_com[ptam_y] - 0.5;
				targets2tum_ardrone(manual_com[ptam_x],manual_com[ptam_y],manual_com[ptam_z],manual_com[ptam_yaw]);				
				ROS_INFO("move backward\n");
			break;

            		case 'a': //left
				clear_targets();
				manual_com[ptam_x] = manual_com[ptam_x] - 0.5;
				targets2tum_ardrone(manual_com[ptam_x],manual_com[ptam_y],manual_com[ptam_z],manual_com[ptam_yaw]);				
				ROS_INFO("move left\n");
			break;

            		case 'd': //right
				clear_targets();
				manual_com[ptam_x] = manual_com[ptam_x] + 0.5;
				targets2tum_ardrone(manual_com[ptam_x],manual_com[ptam_y],manual_com[ptam_z],manual_com[ptam_yaw]);				
				ROS_INFO("move right\n");
			break;

            		case '+': //up
				clear_targets();
				manual_com[ptam_z] = manual_com[ptam_z] + 0.25;
				targets2tum_ardrone(manual_com[ptam_x],manual_com[ptam_y],manual_com[ptam_z],manual_com[ptam_yaw]);				
				ROS_INFO("move up\n");
			break;

            		case '-': //down
				clear_targets();
				manual_com[ptam_z] = manual_com[ptam_z] - 0.25;
				targets2tum_ardrone(manual_com[ptam_x],manual_com[ptam_y],manual_com[ptam_z],manual_com[ptam_yaw]);				
				ROS_INFO("move down\n");
			break;
 
           		case 'q': //Yaw left
				clear_targets();
				manual_com[ptam_yaw] = manual_com[ptam_yaw] - 10;
				targets2tum_ardrone(manual_com[ptam_x],manual_com[ptam_y],manual_com[ptam_z],manual_com[ptam_yaw]);				
				ROS_INFO("Yaw left\n");
			break;

            		case 'e': //Yaw right
				clear_targets();
				manual_com[ptam_yaw] = manual_com[ptam_yaw] + 10;
				targets2tum_ardrone(manual_com[ptam_x],manual_com[ptam_y],manual_com[ptam_z],manual_com[ptam_yaw]);				
				ROS_INFO("Yaw right\n");
			break;
		}
	  
	  }
}

//-----------Function for releasing the memory of pointers---------- 
void ArdroneControl::Free_pointers()
{
    if(path_x != NULL)
    {	
	free(path_x);
	path_x = NULL;
    }
    if(path_y != NULL)
    {	
	free(path_y);
	path_y = NULL;
    }
    if(path_z != NULL)
    {	
	free(path_z);
	path_z = NULL;
    }
    if(path_w != NULL)
    {	
	free(path_w);
	path_w = NULL;
    }
}

//-----------Function for calculating the Yaw angle(degree)---------
float ArdroneControl::Quaternion2Yaw(float qz, float qw)
{
    return(atan2(2*qz*qw,1-2*qz*qz)*180/PI);
}


//-----------Function for publishing the "plan_request"-------------
void ArdroneControl::Plan_request(float end_x, float end_y, float end_yaw)
{
	moveit::planning_interface::MoveGroup group("altHold");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

	std::vector<double> group_variable_values;
	group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

	group_variable_values[0] = end_x;
	group_variable_values[1] = end_y;
	group.setJointValueTarget(group_variable_values);
	group.setPlanningTime(2.5);

	moveit::planning_interface::MoveGroup::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
    ROS_INFO_NAMED("moveo", "Visualizing plan 1 (pose goal) %s", success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "" : "FAILED");

	/* Sleep to give Rviz time to visualize the plan. */
	sleep(0.5);
}


//----------Function for detecting the PointCloud2 Topic---------------
void ArdroneControl::lsd_PC2_callback(const sensor_msgs::PointCloud2ConstPtr PC2_msg)
{
	ROS_INFO("Point Cloud2 BD was updated!!");	
	if(replan_index == 1)
	{
		ROS_INFO("START Replanning!!");		
		Plan_request(goal_x, goal_y, goal_yaw);
	}
		
}

//-----------Function for subscribing the path from MoveIt!------------
void ArdroneControl::path_callback(const moveit_msgs::DisplayTrajectory msg)
{
    //---Copy the path data to pointers----
    int i = 0;
    int i_next = 0;
    int point_size = msg.trajectory[0].multi_dof_joint_trajectory.points.size();
    float qz = 0.0;
    float qw = 0.0;
    float distance = 0.0;
    
    Free_pointers();
    
    path_x = (float*)malloc(point_size * sizeof(float));
    path_y = (float*)malloc(point_size * sizeof(float));
    path_z = (float*)malloc(point_size * sizeof(float));
    path_w = (float*)malloc(point_size * sizeof(float));
    for( i = 0; i < point_size; i++)
    {
	path_x[i] = msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].translation.x;
	path_y[i] = msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].translation.y;	
	path_z[i] = msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].translation.z;
	       qz = msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].rotation.z;
	       qw = msg.trajectory[0].multi_dof_joint_trajectory.points[i].transforms[0].rotation.w;
        path_w[i] = Quaternion2Yaw(qz, qw);
    }
    ROS_INFO("Get the Path, the total size of points is: %d",point_size);

    //---Sned commands to the ARDrone---
    if(path_follow_index == 1)
    {
	i = 0;	
	i_next = 1;
	clear_targets();
	
	sleep(0.5); // Give some time to make ARDrone stable
	
	ROS_INFO("Start to follow the path!!");
	while( i_next < point_size )
        {
	    distance = sqrtf( (path_x[i_next]-path_x[i])*(path_x[i_next]-path_x[i]) + (path_y[i_next]-path_y[i])*(path_y[i_next]-path_y[i]) + (path_z[i_next]-path_z[i])*(path_z[i_next]-path_z[i]) );
	    if(distance <= 0.25) // Set the field of each target is the circle area, which is 0.25^2*PI m^2.
		i_next++;
	    if(distance > 0.25)
	    {
		targets2tum_ardrone(path_x[i_next],path_y[i_next],manual_com[ptam_z],path_w[i_next]);	// Since we only use planar path planning without height,  z = manual_com[ptam_z]
		i = i_next;		
		i_next++;
	    }
        }
	ROS_INFO("All of targets were published!!");
    }

    //---Update requirements of the goal---
    goal_x = path_x[point_size-1];
    goal_y = path_y[point_size-1];
    goal_yaw = path_w[point_size-1]*PI/180.0;
}



//----------------Call-back function bases on tum_ardrone's navadate---------------------
void ArdroneControl::nav_callback(const tum_ardrone::filter_stateConstPtr tum_msg)
{
  	
    if(index2 == 0)
	{
        ROS_INFO("Mode Selection--> 't'->position control, 'n'->manual control, 'p'->path follow:");
		manual_selection = getchar();
        if(manual_selection == 't' || manual_selection == 'n' || manual_selection == 'p')
		index2 = 1;
	}
    if(manual_selection == 't')
    {  	
        Position[ptam_x] = float(tum_msg->x);
        Position[ptam_y] = float(tum_msg->y);
        Position[ptam_z] = float(tum_msg->z);
        Position[ptam_yaw] = float(tum_msg->yaw);
        
        ROS_INFO("Please enter your target like-->0.5 -0.5 0 0 (type x to exit, l to land)");
        float parameters[5];
        char tar_command[15];
        gets(tar_command);
        if (tar_command[0] == 'x')
            index2 = 0;
        if (tar_command[0] == 'l')
            land();
        if(sscanf(tar_command,"%f %f %f %f",&parameters[0], &parameters[1], &parameters[2], &parameters[3]) == 4)
        {
		targets2tum_ardrone(parameters[0],parameters[1],parameters[2],parameters[3]);
                ROS_INFO("Set up the Target!!");
        }  
    }  
    if(manual_selection == 'n')
    {
	ROS_INFO("\nManual Mode--> w a s d; t:takeoff, l:land \np:follow the path, r:replan, i:stop, o:origine, x:exit  :");		
	int com_char = keyboard();
	if( com_char == 'x')
	{
		ROS_INFO("Break"); 
		clear_targets();
           	path_follow_index = 0;
		replan_index = 0;
		Free_pointers();	
		index2 = 0;
	}
	if( com_char != 'x')			
    		Manual(com_char);
    }  
    if(manual_selection == 'p')
    {
	ROS_INFO("Path Mode--> s: start following the path, r: replanning, p: pause   (x for \"STOP\" ):");
        int com_char = keyboard();
	if( com_char == 'x')
	{
		ROS_INFO("Break"); 
		path_follow_index = 0;
		clear_targets(); 
		Free_pointers();          		
		index2 = 0;
	}
        if( com_char == 's')
	{	
		path_follow_index = 1;
		ROS_INFO("START");
	}
        if( com_char == 'r')
	{	
		path_follow_index = 0;  // 1		
		replan_index = 1;		
		ROS_INFO("START");
	}
	if( com_char == 'p')
	{
		replan_index = 0;		
		path_follow_index = 0;
		ROS_INFO("Pause");
	}
    }
}


//------------Main function-----------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ArdroneController");
  ROS_INFO("START");
  ROS_INFO("Wait for connecting the ARDrone!!");  
  ArdroneControl ardrone_control;

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
