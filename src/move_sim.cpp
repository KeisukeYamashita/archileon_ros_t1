#include "ros/ros.h"

#include "std_msgs/Float32.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_broadcaster.h>

#include "math.h"
#include <sstream>
#include <string>

#define PS3_Button_Max 17
#define PS3_Select 0
#define PS3_L3     1
#define PS3_R3     2
#define PS3_Start  3
#define PS3_Up     4
#define PS3_Right  5
#define PS3_Down   6
#define PS3_Left   7
#define PS3_L2     8
#define PS3_R2     9
#define PS3_L1    10
#define PS3_R1    11
#define PS3_triangle 12
#define PS3_circle   13
#define PS3_cross    14
#define PS3_square   15
#define PS3_PS    16

float target_width[2]={0.12,0.12};
float WHEEL_BASE =0.26;
float target_steer[4]={0,0,0,0};
float target_motor[4]={0,0,0,0};
void width0_callback(const std_msgs::Float32& float_msg){
	target_width[0]=float_msg.data;
}
void width1_callback(const std_msgs::Float32& float_msg){
	target_width[1]=float_msg.data;
}
void steer0_callback(const std_msgs::Float32& float_msg){
	target_steer[0]=float_msg.data;
}
void steer1_callback(const std_msgs::Float32& float_msg){
	target_steer[1]=float_msg.data;
}
void steer2_callback(const std_msgs::Float32& float_msg){
	target_steer[2]=float_msg.data;
}
void steer3_callback(const std_msgs::Float32& float_msg){
	target_steer[3]=float_msg.data;
}
void motor0_callback(const std_msgs::Float32& float_msg){
	target_motor[0]=float_msg.data;
}
void motor1_callback(const std_msgs::Float32& float_msg){
	target_motor[1]=float_msg.data;
}
void motor2_callback(const std_msgs::Float32& float_msg){
	target_motor[2]=float_msg.data;
}
void motor3_callback(const std_msgs::Float32& float_msg){
	target_motor[3]=float_msg.data;
}

/*
void joy_callback(const sensor_msgs::Joy& joy_msg){
	bool b_push[PS3_Button_Max]={0};
	bool b_release[PS3_Button_Max]={0};
	static bool b_last[PS3_Button_Max]={0};
	for(int i=0;i<PS3_Button_Max;i++){
		if(joy_msg.buttons[i] && !b_last[i])b_push[i]=true;
		else b_push[i]=false;
		if(!joy_msg.buttons[i] && b_last[i])b_release[i]=true;
		else b_release[i]=false;
		b_last[i]=joy_msg.buttons[i];
	}	

	//move
	move_acc=joy_msg.axes[1];
	move_str=joy_msg.axes[0];
	
	if(b_push[PS3_Up])   width_front+=0.01;
	if(b_push[PS3_Right])width_front-=0.01;
	if(b_push[PS3_Left]) width_rear +=0.01;
	if(b_push[PS3_Down]) width_rear -=0.01;
}
*/

ros::Publisher joint_pub;
void jp(){
	sensor_msgs::JointState js0;
	js0.header.stamp = ros::Time::now();
	js0.name.resize(14);
	js0.name[ 0]="slider0_joint";
	js0.name[ 1]="slider1_joint";
	js0.name[ 2]="wheelset0_holder1_joint";
	js0.name[ 3]="wheelset0_holder2_joint";
	js0.name[ 4]="wheelset0_wheel_joint";
	js0.name[ 5]="wheelset1_holder1_joint";
	js0.name[ 6]="wheelset1_holder2_joint";
	js0.name[ 7]="wheelset1_wheel_joint";
	js0.name[ 8]="wheelset2_holder1_joint";
	js0.name[ 9]="wheelset2_holder2_joint";
	js0.name[10]="wheelset2_wheel_joint";
	js0.name[11]="wheelset3_holder1_joint";
	js0.name[12]="wheelset3_holder2_joint";
	js0.name[13]="wheelset3_wheel_joint";
	
	js0.position.resize(14);
	js0.position[ 0]= WHEEL_BASE/2;
	js0.position[ 1]=-WHEEL_BASE/2;
	js0.position[ 2]= target_width[0]/2;
	js0.position[ 3]= target_steer[0];
	js0.position[ 4]= 0;
	js0.position[ 5]=-target_width[0]/2;
	js0.position[ 6]= target_steer[1];
	js0.position[ 7]= 0;
	js0.position[ 8]=-target_width[1]/2;
	js0.position[ 9]= target_steer[2];
	js0.position[10]= 0;
	js0.position[11]= target_width[1]/2;
	js0.position[12]= target_steer[3];
	js0.position[13]= 0;
	
	joint_pub.publish(js0);
}
void set_robot(float *position, float *direction){
	static tf::TransformBroadcaster br;
	tf::Transform transform;

	transform.setOrigin( tf::Vector3(position[0], position[1], position[2]) );
	tf::Quaternion q;
	q.setRPY(direction[0], direction[1], direction[2]);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));	
}
void move_robot(float dt){
	static float pos_x=0;
	static float pos_y=0;
	static float rot_z=0;
		
	float acc_x=0;
	float acc_y=0;
	float str_z=0;
		

	for(int i=0;i<4;i++){
		acc_x+=target_motor[i]*cos(target_steer[i]);
		acc_y+=target_motor[i]*sin(target_steer[i]);
	}
	float sum_r=target_steer[0]+target_steer[1]-target_steer[2]-target_steer[3];
	str_z=acc_x*sin(sum_r/4)/3.1415/WHEEL_BASE;


	pos_x+=(acc_x*cos(rot_z)-acc_y*sin(rot_z))/4*dt;
	pos_y+=(acc_y*cos(rot_z)+acc_x*sin(rot_z))/4*dt;
	rot_z+=str_z*dt;
	jp();
	float pos_xyz[3]={0,0,0};
	float pos_rpy[3]={0,0,0};
	pos_xyz[0]=pos_x;
	pos_xyz[1]=pos_y;
	pos_rpy[2]=rot_z;
	set_robot(pos_xyz,pos_rpy);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rough_sim");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	pn.getParam("WHEEL_BASE", WHEEL_BASE);
	
	//publish
	joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1000);

	//subscribe
	//ros::Subscriber joy_sub   = n.subscribe("joy", 10, joy_callback);
	ros::Subscriber width0_sub   = n.subscribe("width0", 10, width0_callback); 
	ros::Subscriber width1_sub   = n.subscribe("width1", 10, width1_callback); 
	ros::Subscriber steer0_sub   = n.subscribe("steer0", 10, steer0_callback); 
	ros::Subscriber steer1_sub   = n.subscribe("steer1", 10, steer1_callback); 
	ros::Subscriber steer2_sub   = n.subscribe("steer2", 10, steer2_callback); 
	ros::Subscriber steer3_sub   = n.subscribe("steer3", 10, steer3_callback); 
	ros::Subscriber motor0_sub   = n.subscribe("motor0", 10, motor0_callback); 
	ros::Subscriber motor1_sub   = n.subscribe("motor1", 10, motor1_callback); 
	ros::Subscriber motor2_sub   = n.subscribe("motor2", 10, motor2_callback); 
	ros::Subscriber motor3_sub   = n.subscribe("motor3", 10, motor3_callback); 
			
	float dt=1.0/20;
	ros::Rate loop_rate(20); 
	while (ros::ok()){
		move_robot(dt);
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

