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
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <tf/transform_broadcaster.h>

//---kbhit() header---
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h> 

#define pos_x 0
#define pos_y 1
#define pos_z 2
#define ori_x 3
#define ori_y 4
#define ori_z 5
#define ori_w 6

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



using namespace Eigen;

class Pose_Conversion
{
  public:
    Pose_Conversion();

  private:
    int peirod;
    int publish_index;  // The index for detecting the PC2_callback function
    int publish_choice; // The index for deciding whether the PC2_scaled is published or not
    int ptam_loop;
    int scale_lock;
    float ptam_pose_cur[7];
    double slam_time_cur;
    double ptam_time_cur;
    double looprate;

    ros::NodeHandle n;                               
    ros::Subscriber lsdslam_pose_sub;                         
    ros::Subscriber tumptam_pose_sub;
    ros::Subscriber pointcloud_sub;			     
    ros::Publisher  pointcloud2_pub;             
    
    Matrix4f P;
    Matrix4f R; 
    Matrix3f dq2M; 
    Vector4f Q;
    Vector4f S;    
    Vector4f dx; 
    Vector4f dq; 

    void lsdslam_pose_CB(const geometry_msgs::PoseStampedConstPtr slam_msg);
    void tumptam_pose_CB(const geometry_msgs::PoseStampedConstPtr ptam_msg);
    void pointcloud_CB(const sensor_msgs::PointCloudConstPtr PC_msg);
};

Pose_Conversion::Pose_Conversion()
{
  pointcloud2_pub = n.advertise<sensor_msgs::PointCloud2>("pointcloud2_scaled", 1, true);
  pointcloud_sub = n.subscribe("/pointcloud", 1, &Pose_Conversion::pointcloud_CB, this);
  lsdslam_pose_sub = n.subscribe("/lsd_slam/pose", 1, &Pose_Conversion::lsdslam_pose_CB, this);
  tumptam_pose_sub = n.subscribe("/tum_ardrone/pose", 1, &Pose_Conversion::tumptam_pose_CB, this);

  ptam_pose_cur[0]=ptam_pose_cur[1]=ptam_pose_cur[2]=ptam_pose_cur[3]=ptam_pose_cur[4]=ptam_pose_cur[5]=ptam_pose_cur[6]=0.0;

  publish_index = 0;
  publish_choice = 0;
  ptam_loop = 0;
  peirod = 0.500;
  scale_lock = 0;
  slam_time_cur = 0.0;
  ptam_time_cur = 0.0;
  looprate = ros::Time::now().toSec();

  P << 0,0,0,0,
       0,0,0,0,
       0,0,0,0,
       0,0,0,0;
  R = P;

  dq2M << 0,0,0,
          0,0,0,
          0,0,0;

  Q << 0,0,0,0;
  S = dx = dq = Q;
}


void Pose_Conversion::lsdslam_pose_CB(const geometry_msgs::PoseStampedConstPtr slam_msg)
{	
	slam_time_cur = slam_msg->header.stamp.toSec();

	//----Copy data from slam_pose and ptam_pose---
	float ptam_pose[7];
	ptam_pose[pos_x] = ptam_pose_cur[pos_x];
	ptam_pose[pos_y] = ptam_pose_cur[pos_y];
	ptam_pose[pos_z] = ptam_pose_cur[pos_z];
	ptam_pose[ori_x] = ptam_pose_cur[ori_x];
	ptam_pose[ori_y] = ptam_pose_cur[ori_y];
	ptam_pose[ori_z] = ptam_pose_cur[ori_z];
	ptam_pose[ori_w] = ptam_pose_cur[ori_w];

	float slam_pose[7];
	slam_pose[pos_x] = slam_msg->pose.position.x;
	slam_pose[pos_y] = slam_msg->pose.position.y;
	slam_pose[pos_z] = slam_msg->pose.position.z;
	slam_pose[ori_x] = slam_msg->pose.orientation.x;
	slam_pose[ori_y] = slam_msg->pose.orientation.y;
	slam_pose[ori_z] = slam_msg->pose.orientation.z;
	slam_pose[ori_w] = slam_msg->pose.orientation.w;		

	//---Declare matrix and vector for least square calculating---
	Matrix4f temp_p; // q_lsd matrix
	MatrixXf temp_r(3,4);
	Vector3f xlsd_temp(0,0,0);



	//---Transform the tum_pose_ori to new tum_pose_ori---
	Vector4f qtf1(0.70710678, 0.70710678,0 ,0);
	Vector4f qtf1_inv(-0.70710678, -0.70710678,0 ,0);		
	Matrix4f M_dq;
	Vector4f qtum(ptam_pose[ori_x],ptam_pose[ori_y],ptam_pose[ori_z],ptam_pose[ori_w]);
	Vector3f xtum(ptam_pose[pos_x],ptam_pose[pos_y],ptam_pose[pos_z]);

	M_dq << qtf1_inv(3), qtf1_inv(2),-qtf1_inv(1), qtf1_inv(0),
	       -qtf1_inv(2), qtf1_inv(3), qtf1_inv(0), qtf1_inv(1),
                qtf1_inv(1),-qtf1_inv(0), qtf1_inv(3), qtf1_inv(2),
               -qtf1_inv(0),-qtf1_inv(1),-qtf1_inv(2), qtf1_inv(3);	
	
	qtum = M_dq*qtum;

	M_dq << qtum(3), qtum(2),-qtum(1), qtum(0),
	       -qtum(2), qtum(3), qtum(0), qtum(1),
                qtum(1),-qtum(0), qtum(3), qtum(2),
               -qtum(0),-qtum(1),-qtum(2), qtum(3);

	qtum = M_dq*qtf1; // Set up new tum_pose_ori !!



	//---Transform the lsd_pose to new lsd_pose---
	Vector4f qtf2(0.70710678, 0, 0, 0.70710678);		// x-axis 90 degree
	Vector4f qtf2_inv(-0.70710678, 0, 0, 0.70710678);		
	Matrix4f M_dq2;
	Vector4f qlsd(slam_pose[ori_x],slam_pose[ori_y],slam_pose[ori_z],slam_pose[ori_w]);
	Vector3f xlsd(slam_pose[pos_x],slam_pose[pos_y],slam_pose[pos_z]);	
	
	M_dq2 << qtf2(3), qtf2(2),-qtf2(1), qtf2(0),
	        -qtf2(2), qtf2(3), qtf2(0), qtf2(1),
                 qtf2(1),-qtf2(0), qtf2(3), qtf2(2),
                -qtf2(0),-qtf2(1),-qtf2(2), qtf2(3);	
	
	qlsd = M_dq2*qlsd;

	M_dq2 << qlsd(3), qlsd(2),-qlsd(1), qlsd(0),
	        -qlsd(2), qlsd(3), qlsd(0), qlsd(1),
                 qlsd(1),-qlsd(0), qlsd(3), qlsd(2),
                -qlsd(0),-qlsd(1),-qlsd(2), qlsd(3);

	qlsd = M_dq2*qtf2_inv; // Set up new lsd_pose_ori !!

	Matrix3f R_qtf2;
	
	R_qtf2 <<  -2*qtf2(1)*qtf2(1)-2*qtf2(2)*qtf2(2)+1,    2*qtf2(0)*qtf2(1)-2*qtf2(2)*qtf2(3),    2*qtf2(0)*qtf2(2)+2*qtf2(1)*qtf2(3),
		      2*qtf2(0)*qtf2(1)+2*qtf2(2)*qtf2(3), -2*qtf2(0)*qtf2(0)-2*qtf2(2)*qtf2(2)+1,    2*qtf2(1)*qtf2(2)-2*qtf2(0)*qtf2(3),
		      2*qtf2(0)*qtf2(2)-2*qtf2(1)*qtf2(3),    2*qtf2(0)*qtf2(3)+2*qtf2(1)*qtf2(2), -2*qtf2(0)*qtf2(0)-2*qtf2(1)*qtf2(1)+1;

	xlsd = (R_qtf2.transpose())*xlsd;    // Set up new lsd_pose_pos !!


	if( scale_lock == 0)
	{	
		//---Loop rate is 2 Hz---
		if( (ros::Time::now().toSec() - looprate) >= peirod && ptam_loop == 1)
		{	
			//---Do the least square method---		
			temp_p <<  qlsd(3), qlsd(2),-qlsd(1), qlsd(0),
				  -qlsd(2), qlsd(3), qlsd(0), qlsd(1),
		                   qlsd(1),-qlsd(0), qlsd(3), qlsd(2),
		                  -qlsd(0),-qlsd(1),-qlsd(2), qlsd(3);	

			P = P + (temp_p.transpose())*temp_p;
			Q = Q + (temp_p.transpose())*qtum;
		
			if(P.determinant() > 0.000001)
			{
				dq = (P.inverse())*Q;
				dq = dq/(sqrt(dq(0)*dq(0) + dq(1)*dq(1) + dq(2)*dq(2) + dq(3)*dq(3) ));
			}

			dq2M <<  -2*dq(1)*dq(1)-2*dq(2)*dq(2)+1,    2*dq(0)*dq(1)-2*dq(2)*dq(3),    2*dq(0)*dq(2)+2*dq(1)*dq(3),
		                    2*dq(0)*dq(1)+2*dq(2)*dq(3), -2*dq(0)*dq(0)-2*dq(2)*dq(2)+1,    2*dq(1)*dq(2)-2*dq(0)*dq(3),
		                    2*dq(0)*dq(2)-2*dq(1)*dq(3),    2*dq(0)*dq(3)+2*dq(1)*dq(2), -2*dq(0)*dq(0)-2*dq(1)*dq(1)+1;
		
			temp_r << 1,0,0,0,
				  0,1,0,0,
	       			  0,0,1,0;

		        xlsd_temp = (dq2M.transpose())*xlsd;	
			temp_r(0,3) = xlsd_temp(0);
	 		temp_r(1,3) = xlsd_temp(1);
			temp_r(2,3) = xlsd_temp(2);
		
			Matrix4f Rtemp;

			Rtemp = R+(temp_r.transpose())*temp_r;
		
			if( R(0,0) == 0)
			{
				R = Rtemp;
				S = S+(temp_r.transpose())*xtum;
			}
			else if(  (Rtemp/ (Rtemp(0,0)) ).determinant() > (R/ (R(0,0))).determinant() )
			{
				R = Rtemp;
				S = S+(temp_r.transpose())*xtum;
			}

		

			if( (R/ (R(0,0))).determinant() > 0.000001)
			{
				dx = (R.inverse())*S;
				publish_index = 1;
				ROS_INFO("Least-square successful !!");
			}

			std::stringstream ss;
			ss <<"\ndx:" << dx.transpose() << "\ndq:" << dq.transpose() << " \nx_lsd:" << xlsd.transpose() << "\nx_tum:" << xtum.transpose()<<"  \nTime:" << ros::Time::now() ;
			//ss <<"\ndx:" << dx.transpose() << "\nR:" << R << " \nS:" <<S.transpose();	
			std::string s = ss.str();
			ROS_INFO(s.c_str());
			ptam_loop = 0; 
			looprate = looprate + peirod;
		}
	}

	if( kbhit() )
	{
		int ch = getchar();
		if( ch == 'l')
		{
			scale_lock = 1;
			ROS_INFO("Scale Lock!! (Enter 'r' to unlock)");
		}		
		if( ch == 'r')
			scale_lock = 0;
		if( ch == 'p' )
		{
			publish_choice = 1;
			ROS_INFO("Start to publish the PC2_scaled !!");
		}
		if( ch == 's' )
		{
			publish_choice = 0;
			ROS_INFO("Stop publishing the PC2_scaled !!");
		}
	}
	//---Broadcasting the TF (from map to shadow) by using lsd_pose---
	Matrix4f Mqlsd;
	Vector4f qlsd_tf_temp(0,0,0,0);
	Mqlsd << qlsd(3), qlsd(2),-qlsd(1), qlsd(0),
		-qlsd(2), qlsd(3), qlsd(0), qlsd(1),
                 qlsd(1),-qlsd(0), qlsd(3), qlsd(2),
                -qlsd(0),-qlsd(1),-qlsd(2), qlsd(3);
	
	qlsd_tf_temp = Mqlsd*dq;

	Vector3f xlsd_tf_temp(0,0,0);
	MatrixXf temp_tf(3,4);
	temp_tf << 1,0,0,0,
		   0,1,0,0,
       		   0,0,1,0;
	xlsd_tf_temp = (dq2M.transpose())*xlsd;
	temp_tf(0,3) = xlsd_tf_temp(0);
	temp_tf(1,3) = xlsd_tf_temp(1);
	temp_tf(2,3) = xlsd_tf_temp(2);
	
	xlsd_tf_temp = temp_tf*dx;

	static tf::TransformBroadcaster br;	
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(xlsd_tf_temp(0), xlsd_tf_temp(1), xlsd_tf_temp(2)));
	transform.setRotation(tf::Quaternion( qlsd_tf_temp(0), qlsd_tf_temp(1), qlsd_tf_temp(2), qlsd_tf_temp(3)));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "shadow")); 
}



void Pose_Conversion::tumptam_pose_CB(const geometry_msgs::PoseStampedConstPtr ptam_msg)
{
	ptam_loop = 1;	
	ptam_time_cur = ptam_msg->header.stamp.toSec();
	ptam_pose_cur[pos_x] = ptam_msg->pose.position.x;
	ptam_pose_cur[pos_y] = ptam_msg->pose.position.y;
	ptam_pose_cur[pos_z] = ptam_msg->pose.position.z;
	ptam_pose_cur[ori_x] = ptam_msg->pose.orientation.x;
	ptam_pose_cur[ori_y] = ptam_msg->pose.orientation.y;
	ptam_pose_cur[ori_z] = ptam_msg->pose.orientation.z;
	ptam_pose_cur[ori_w] = ptam_msg->pose.orientation.w;
}


void Pose_Conversion::pointcloud_CB(const sensor_msgs::PointCloudConstPtr PC_msg)
{
	ROS_INFO("pointcloud_CB");
	if(publish_index == 1 && publish_choice == 1)
        {	
		sensor_msgs::PointCloud cloud;
		cloud.header.stamp = ros::Time::now();
		cloud.header.frame_id = "map";
		cloud.points.resize(PC_msg->points.size());

		Matrix3f R_qtf2;
		Vector4f qtf2(0.70710678, 0, 0, 0.70710678);
		Vector3f x_temp(0,0,0);		
		Vector3f x_ori(0,0,0);
		Vector3f x_new(0,0,0);	
		MatrixXf temp_r(3,4);	
		Matrix3f dq2M_cur; 
		Vector4f dx_cur(0,0,0,0);		

		dq2M_cur = dq2M;
		dx_cur = dx;

		for(int total=0; total < (PC_msg->points.size()) ; total++)
		{	
			// Transform lsd_pose_pointcloud
			x_ori << PC_msg->points[total].x, PC_msg->points[total].y, PC_msg->points[total].z;			
			
			R_qtf2 <<  -2*qtf2(1)*qtf2(1)-2*qtf2(2)*qtf2(2)+1,    2*qtf2(0)*qtf2(1)-2*qtf2(2)*qtf2(3),    2*qtf2(0)*qtf2(2)+2*qtf2(1)*qtf2(3),
				      2*qtf2(0)*qtf2(1)+2*qtf2(2)*qtf2(3), -2*qtf2(0)*qtf2(0)-2*qtf2(2)*qtf2(2)+1,    2*qtf2(1)*qtf2(2)-2*qtf2(0)*qtf2(3),
				      2*qtf2(0)*qtf2(2)-2*qtf2(1)*qtf2(3),    2*qtf2(0)*qtf2(3)+2*qtf2(1)*qtf2(2), -2*qtf2(0)*qtf2(0)-2*qtf2(1)*qtf2(1)+1;

			x_ori = (R_qtf2.transpose())*x_ori;    // Set up new lsd_pose_pointcloud !!

			// Transform lsd_pose_pointcloud from lsd_world to map
			temp_r << 1,0,0,0,
				  0,1,0,0,
	       			  0,0,1,0;		
						
			x_temp = (dq2M_cur.transpose())*x_ori;
			temp_r(0,3) = x_temp(0);
	 		temp_r(1,3) = x_temp(1);
			temp_r(2,3) = x_temp(2);
	
			x_new = temp_r*dx_cur;
	
			cloud.points[total].x = x_new(0);
			cloud.points[total].y = x_new(1);
			cloud.points[total].z = x_new(2);
		}
		sensor_msgs::PointCloud2 cloud2;
		convertPointCloudToPointCloud2(cloud,cloud2);	
		pointcloud2_pub.publish(cloud2);
		ROS_INFO("Publishing Scaled Point_Cloud2");
	}
	else
		ROS_INFO("Still un-invertible");
}



//------------Main function-----------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "conversion");
  ROS_INFO("START");
  ROS_INFO("l:lock the scale, r:unlock, p:publish PC2_scale, s:Stop publishing  :");
  Pose_Conversion pose_conversion;

  ros::AsyncSpinner spinner(3); // Use 3 threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
