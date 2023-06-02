#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Empty.h"
#include "tf/transform_datatypes.h"
//#include "sensor_msgs/Imu.h"
//#include "px4_control_cfg.h"
#include "geometry_msgs/Twist.h" 
//#include "nlink_parser/LinktrackAnchorframe0.h"
//#include "nlink_parser/LinktrackTagframe0.h"
#include "geometry_msgs/PoseStamped.h"
//#include "tello_driver/TelloStatus.h"
#include <math.h>
#include <iostream>
#define PI 3.1415926
//#include <Eigen/Dense>
//#include <Eigen/Core>
using namespace std;
//using namespace Eigen;

double t=0,tstart=0;
int i,j,flag=0;
double A=0,B=1,B_hat=0,B_tlide=1,K=-3.3181,u1[2][1],u2[2][1],u3[2][1],u4[2][1],u5[2][1], u6[2][1], u7[2][1],u8[2][1];
double tello_pos1[2][1],tello_pos2[2][1],tello_pos3[2][1],tello_pos4[2][1],tello_pos5[2][1], tello_pos6[2][1],tello_pos7[2][1],tello_pos8[2][1];//位置
double h1[2][1],h2[2][1],h3[2][1],h4[2][1], h5[2][1], h6[2][1];//编队项
double v1[2][1],v2[2][1],v3[2][1],v4[2][1], v5[2][1], v6[2][1];//补偿项
double F1[2][1],F2[2][1],F3[2][1],F4[2][1], F5[2][1], F6[2][1];//避障
//Matrix4d L[4][4]={{1,0,0,-1},{-1,1,0,0},{0,-1,1,0},{0,0,-1,1}};
double delta=0.6;
double lamda=-1;
double K_rep=0.4;//斥力场系数
double ro=0.4;//作用范围
double r=1;//编队半径
int ans=0;
double X[10];
double Y[10];
double D[10][10],Fx[10][10],Fy[10][10],cgemaFx[10],cgemaFy[10];

geometry_msgs::Twist cmd_vel1;
geometry_msgs::Twist cmd_vel2;
geometry_msgs::Twist cmd_vel3;
geometry_msgs::Twist cmd_vel4;
geometry_msgs::Twist cmd_vel5;
geometry_msgs::Twist cmd_vel6;
geometry_msgs::Twist cmd_vel7;
geometry_msgs::Twist cmd_vel8;


/*S_PID_ITEM PidItemX;
S_PID_ITEM PidItemY;
S_PID_ITEM PidItemZ;*/

int command;


/*geometry_msgs::Twist VelPIDController(double selfPos[3],double leaderPos[3],double expectPos[3],double expectyaw)
{
	geometry_msgs::Twist cmd_vel;

	//cmd.linear和cmd.angular范围都是-1到1，模拟的是摇杆最上和最下。tellopy自带限幅
	//但是tello的速度还有两档，fastmode?

	// tello飞机的平动机体坐标系是右前上坐标系,yaw也是朝上为正
	PidItemX.difference = leaderPos[0]+expectPos[0]-selfPos[0];       //当前偏差
	PidItemX.differential = PidItemX.difference - PidItemX.tempDiffer;//微分项 当前偏差-上一刻偏差
	PidItemX.integral += PidItemX.difference;                         //偏差累计 积分项
	PidItemX.tempDiffer = PidItemX.difference;                        
	cmd_vel.linear.x = vel_P*PidItemX.difference + vel_D*PidItemX.differential + vel_I*PidItemX.integral;	

	// vy 
	PidItemY.difference = leaderPos[1]+expectPos[1]-selfPos[1];
	PidItemY.differential = PidItemY.difference - PidItemY.tempDiffer;
	PidItemY.integral += PidItemY.difference;
	PidItemY.tempDiffer = PidItemY.difference;
	cmd_vel.linear.y = vel_P*PidItemY.difference + vel_D*PidItemY.differential + vel_I*PidItemY.integral;

	// vz
	PidItemZ.difference = leaderPos[2]+expectPos[2]-selfPos[2];
	PidItemZ.differential = PidItemZ.difference - PidItemZ.tempDiffer;
	PidItemZ.integral += PidItemZ.difference;
	PidItemZ.tempDiffer = PidItemZ.difference;
	cmd_vel.linear.z = vel_P*PidItemZ.difference + vel_D*PidItemZ.differential + vel_I*PidItemZ.integral;

	cmd_vel.angular.x = 0;
	cmd_vel.angular.y = 0;
	cmd_vel.angular.z = 0;
	cmd_vel.angular.z = w_P*(expectyaw-yaw_angle);
	//注意！tello航向角朝下为正

	return cmd_vel;
}*/


//void cb_readPos(const nlink_parser::LinktrackAnchorframe0::ConstPtr& msg)
void cb_readPos_1(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::PoseStamped pos;
	pos = *msg;

	tello_pos1[0][0] = pos.pose.position.x;
	tello_pos1[1][0] = pos.pose.position.y;

}

void cb_readPos_2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{	
	geometry_msgs::PoseStamped pos;
	pos = *msg;
	
	tello_pos2[0][0] =  pos.pose.position.x;
	tello_pos2[1][0] =  pos.pose.position.y;
	
}

void cb_readPos_3(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::PoseStamped pos;
	pos = *msg;

	tello_pos3[0][0] = pos.pose.position.x;
	tello_pos3[1][0] = pos.pose.position.y;

}
void cb_readPos_4(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::PoseStamped pos;
	pos = *msg;

	tello_pos4[0][0] = pos.pose.position.x;
	tello_pos4[1][0] = pos.pose.position.y;

}
void cb_readPos_5(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::PoseStamped pos;
	pos = *msg;

	tello_pos5[0][0] = pos.pose.position.x;
	tello_pos5[1][0] = pos.pose.position.y;

}
void cb_readPos_6(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::PoseStamped pos;
	pos = *msg;

	tello_pos6[0][0] = pos.pose.position.x;
	tello_pos6[1][0] = pos.pose.position.y;

}
void cb_readPos_7(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::PoseStamped pos;
	pos = *msg;

	tello_pos7[0][0] = pos.pose.position.x;
	tello_pos7[1][0] = pos.pose.position.y;

}
void cb_readPos_8(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::PoseStamped pos;
	pos = *msg;

	tello_pos8[0][0] = pos.pose.position.x;
	tello_pos8[1][0] = pos.pose.position.y;

}
/*void cb_status(const tello_driver::TelloStatus::ConstPtr& msg)
{
	tello_driver::TelloStatus Tellodata;
	Tellodata = *msg;
	Battery_percentage = Tellodata.battery_percentage;
	tello_pos[2] = Tellodata.height_m;	
	//cout << "Battery_percentage: "<< Battery_percentage << endl;
	//cout << "tello_height: "<< tello_pos[0][2] << endl;
}*/


void cb_command(const std_msgs::Int32::ConstPtr& msg)
{
	std_msgs::Int32 cmd_data;
	cmd_data = *msg;
	command = cmd_data.data;
	cout << "receive command: " << command << endl;
	if(command == 3 && flag == 0)
	{
		tstart=ros::Time::now().toSec();
		flag=1;
	}
	if(command == 2 && flag == 0)
	{
		tstart=ros::Time::now().toSec();
		flag=1;
	}
}


/*void cb_imu(const sensor_msgs::Imu::ConstPtr& msg)
{
	double Roll,Pitch,Yaw;
	sensor_msgs::Imu IMUdata;
	IMUdata = *msg;
	tf::Quaternion quat;
      	tf::quaternionMsgToTF(IMUdata.orientation,quat);
      	tf::Matrix3x3(quat).getRPY(Roll,Pitch,Yaw);

	Roll = Roll*180/3.14159;
	Pitch = Pitch*180/3.14159;
	Yaw = Yaw*180/3.14159;

	//cout << "IMU angle RPY: "<< Roll << " " << Pitch <<" " << Yaw << endl;
	//tello的yaw是以开机时的角度为0的

	yaw_angle = Yaw; 
}*/

/*void cb_expectPos(const geometry_msgs::Twist::ConstPtr& msg)
{
	geometry_msgs::Twist posdata;
	posdata = *msg;
	expectPos[0] = posdata.linear.x; //meter
	expectPos[1] = posdata.linear.y; //meter 
	expectPos[2] = posdata.linear.z; //meter
	expectYawAngle = posdata.angular.z + initialYawAngle; //degree 这个是在起飞航向角的基础上增加的值
	cout << "expectPos_x: "<< expectPos[0] << endl;
	cout << "expectPos_y: "<< expectPos[1] << endl;
	cout << "expectPos_z: "<< expectPos[2] << endl;
	cout << "expectYawAngle: "<< expectYawAngle << endl;
}*/




int main(int argc,char **argv)
{
	ros::init(argc,argv,"tello0");
	ros::NodeHandle nh("");

	ros::Subscriber cmd_sub = nh.subscribe("/command",10,cb_command); 
	//ros::Subscriber pos_sub = nh.subscribe("/nlink_linktrack_anchorframe0",10,cb_readPos); //cb means Callback
	ros::Subscriber pos_sub1 = nh.subscribe("/tello1/ground_truth_to_tf/pose", 10, cb_readPos_1);
	ros::Subscriber pos_sub2 = nh.subscribe("/tello2/ground_truth_to_tf/pose", 10, cb_readPos_2);
	ros::Subscriber pos_sub3 = nh.subscribe("/tello3/ground_truth_to_tf/pose", 10, cb_readPos_3);
	ros::Subscriber pos_sub4 = nh.subscribe("/tello4/ground_truth_to_tf/pose", 10, cb_readPos_4);
	ros::Subscriber pos_sub5 = nh.subscribe("/tello5/ground_truth_to_tf/pose", 10, cb_readPos_5);
	ros::Subscriber pos_sub6 = nh.subscribe("/tello6/ground_truth_to_tf/pose", 10, cb_readPos_6);
	ros::Subscriber pos_sub7 = nh.subscribe("/tello7/ground_truth_to_tf/pose", 10, cb_readPos_7);
	ros::Subscriber pos_sub8 = nh.subscribe("/tello8/ground_truth_to_tf/pose", 10, cb_readPos_8);
	//ros::Subscriber expectPos_sub = nh.subscribe("expectPos",10,cb_expectPos);
	//ros::Subscriber status_sub = nh.subscribe("status",10,cb_status); 
	//ros::Subscriber imu_sub = nh.subscribe("imu",10,cb_imu); 

	ros::Publisher vel_pub1 = nh.advertise<geometry_msgs::Twist>("/tello1/cmd_vel",10); 
	ros::Publisher vel_pub2 = nh.advertise<geometry_msgs::Twist>("/tello2/cmd_vel",10); 
	ros::Publisher vel_pub3 = nh.advertise<geometry_msgs::Twist>("/tello3/cmd_vel",10); 
	ros::Publisher vel_pub4 = nh.advertise<geometry_msgs::Twist>("/tello4/cmd_vel",10); 
	ros::Publisher vel_pub5 = nh.advertise<geometry_msgs::Twist>("/tello5/cmd_vel",10); 
	ros::Publisher vel_pub6 = nh.advertise<geometry_msgs::Twist>("/tello6/cmd_vel",10); 
	ros::Publisher vel_pub7 = nh.advertise<geometry_msgs::Twist>("/tello7/cmd_vel",10); 
	ros::Publisher vel_pub8 = nh.advertise<geometry_msgs::Twist>("/tello8/cmd_vel",10);

	ros::Publisher takeoff_pub1 = nh.advertise<std_msgs::Empty>("/tello1/takeoff",10);
	ros::Publisher takeoff_pub2 = nh.advertise<std_msgs::Empty>("/tello2/takeoff",10); 
	ros::Publisher takeoff_pub3 = nh.advertise<std_msgs::Empty>("/tello3/takeoff",10); 
	ros::Publisher takeoff_pub4 = nh.advertise<std_msgs::Empty>("/tello4/takeoff",10); 
	ros::Publisher takeoff_pub5 = nh.advertise<std_msgs::Empty>("/tello5/takeoff",10); 
	ros::Publisher takeoff_pub6 = nh.advertise<std_msgs::Empty>("/tello6/takeoff",10);
 
	ros::Publisher land_pub1 = nh.advertise<std_msgs::Empty>("/tello1/land",10); 
	ros::Publisher land_pub2 = nh.advertise<std_msgs::Empty>("/tello2/land",10); 
	ros::Publisher land_pub3 = nh.advertise<std_msgs::Empty>("/tello3/land",10); 
	ros::Publisher land_pub4 = nh.advertise<std_msgs::Empty>("/tello4/land",10); 
	ros::Publisher land_pub5 = nh.advertise<std_msgs::Empty>("/tello5/land",10); 
	ros::Publisher land_pub6 = nh.advertise<std_msgs::Empty>("/tello6/land",10); 


	//ros::Publisher ready = nh.advertise<std_msgs::Int32>("ready",10);

	/*ros::param::get("~vel_P",vel_P);
	ros::param::get("~vel_D",vel_D);
	ros::param::get("~vel_I",vel_I);
	ros::param::get("~w_P",w_P);
	ros::param::get("~ID",ID);

	cout << "vel_P: "<< vel_P << endl;
	cout << "vel_D: "<< vel_D << endl;
	cout << "vel_I: "<< vel_I << endl;
	cout << "w_P : "<< w_P << endl;
	cout << "ID : "<< ID << endl;*/


	/*yaw_angle = 0;
	expectYawAngle = 0;
	initialYawAngle = 0;
	Battery_percentage = 0;
	cmd_vel.linear.x = 0;
	cmd_vel.linear.y = 0;
	cmd_vel.linear.z = 0;
	cmd_vel.angular.x = 0;
	cmd_vel.angular.y = 0;
	cmd_vel.angular.z = 0;
	PidItemX.tempDiffer = 0;
	PidItemX.integral = 0;
	PidItemY.tempDiffer = 0;
	PidItemY.integral = 0;
	PidItemZ.tempDiffer = 0;
	PidItemZ.integral = 0;*/
	command = 0;


	ros::Rate loop_rate(100);

	while(ros::ok())
	{	
		double t=ros::Time::now().toSec()-tstart;
		//cout<<"t:"<<t<<endl;
		double w=0.2;//w=0.1成功
		
		/*if(command == 0) //确定期望航向角为此时的角度，且指令速度给0
			{
				initialYawAngle = yaw_angle;	
				expectYawAngle = yaw_angle;			
				//cout << "expectYawAngle: " << expectYawAngle << endl;
				PidItemX.tempDiffer = 0;
				PidItemX.integral = 0;
				PidItemY.tempDiffer = 0;
				PidItemY.integral = 0;	
				PidItemZ.tempDiffer = 0;
				PidItemZ.integral = 0;	
				cmd_vel.linear.x = 0;
				cmd_vel.linear.y = 0;
				cmd_vel.linear.z = 0;
				cmd_vel.angular.x = 0;
				cmd_vel.angular.y = 0;
				cmd_vel.angular.z = 0;
				vel_pub.publish(cmd_vel);
			}*/		
		if(command == 1)  //起飞
			{	
				std_msgs::Empty takeoff_cmd;
				takeoff_pub1.publish(takeoff_cmd);
				takeoff_pub2.publish(takeoff_cmd);
				takeoff_pub3.publish(takeoff_cmd);
				takeoff_pub4.publish(takeoff_cmd);
				takeoff_pub5.publish(takeoff_cmd);
			}	
		if(command == 2)  //进行编队
			{	
					h1[0][0] = r * sin(w * t);
					h1[1][0] = r * cos(w * t);
					h2[0][0] = r * sin(w * t + PI * 2 / 3);
					h2[1][0] = r * cos(w * t + PI * 2 / 3);
					h3[0][0] = r * sin(w * t + PI * 4 / 3);
					h3[1][0] = r * cos(w * t + PI * 4 / 3);


					v1[0][0] = r * w * cos(w * t);
					v1[1][0] = -r * w * sin(w * t);
					v2[0][0] = r * w * cos(w * t + PI * 2 / 3);
					v2[1][0] = -r * w * sin(w * t + PI * 2 / 3);
					v3[0][0] = r * w * cos(w * t + PI * 4 / 3);
					v3[1][0] = -r * w * sin(w * t + PI * 4 / 3);
					//cout<<"v1x"<<v1[0][0]<<endl;
					//cout<<"v1y"<<v1[1][0]<<endl;
					X[0]=tello_pos1[0][0];
					X[1]=tello_pos2[0][0];
					X[2]=tello_pos3[0][0];
					X[3]=tello_pos5[0][0];
					X[4]=tello_pos6[0][0];
					X[5]=tello_pos7[0][0];
					X[6]=tello_pos4[0][0];
					X[7]=tello_pos8[0][0];
					X[8]=2;

					Y[0]=tello_pos1[1][0];
					Y[1]=tello_pos2[1][0];
					Y[2]=tello_pos3[1][0];
					Y[3]=tello_pos5[1][0];
					Y[4]=tello_pos6[1][0];
					Y[5]=tello_pos7[1][0];
					Y[6]=tello_pos4[1][0];
					Y[7]=tello_pos8[1][0];
					Y[8]=3;

		for(i=0;i<9;i++)
		{
			for(j=0;j<9;j++)
			{
				D[i][j]=sqrt((X[i] - X[j])*(X[i] - X[j]) + (Y[i] - Y[j])*(Y[i] - Y[j]));	//distance
				//cout<<"D:"<<D[i][j]<<endl;			
			}
		}
			//cout<<"D1:"<<D[4][5]<<endl;
			//cout<<"D2:"<<D[5][4]<<endl;
		for(i=0;i<6;i++)
		{
			for(j=0;j<9;j++)
			{
				if((D[i][j]!=0) && (D[i][j]<=ro))
				{
					Fx[i][j]=K_rep*(1/D[i][j]-1/ro)*((1/D[i][j])*(1/D[i][j])*(1/D[i][j]))*(X[i]-X[j]);
					Fy[i][j]=K_rep*(1/D[i][j]-1/ro)*((1/D[i][j])*(1/D[i][j])*(1/D[i][j]))*(Y[i]-Y[j]);
					ans++;
					//cout<<"ans:"<<ans<<endl;
				}
				else
				{
					Fx[i][j]=0;
					Fy[i][j]=0;
				}
			}		
		}
		for(i=0;i<6;i++)
		{
			cgemaFx[i]=0;
			cgemaFy[i]=0;		
		}
		
		for(i=0;i<6;i++)
		{
			for(j=0;j<9;j++)
			{
				cgemaFx[i]+=Fx[i][j];
				cgemaFy[i]+=Fy[i][j];			
			}
		}
		F1[0][0]	=	cgemaFx[0];
		F1[1][0]	=	cgemaFy[0];
		F2[0][0]	=	cgemaFx[1];
		F2[1][0]	=	cgemaFy[1];
		F3[0][0]	=	cgemaFx[2];
		F3[1][0]	=	cgemaFy[2];
		F4[0][0]	=	cgemaFx[3];
		F4[1][0]	=	cgemaFy[3];
		F5[0][0]	=	cgemaFx[4];
		F5[1][0]	=	cgemaFy[4];
		F6[0][0]	=	cgemaFx[5];
		F6[1][0]	=	cgemaFy[5];
		
		//cout<<"F1x:"<<F1[0][0]<<endl;
		//cout<<"F1y:"<<F1[1][0]<<endl;
		if(t>0)
			{	
				u4[0][0]=0.6;
				u4[1][0]=0;
				u8[0][0]=-0.6;
				u8[1][0]=0;
			}
		if(t>=10)
			{
				u4[0][0]=0;
				u4[1][0]=0.2;
				u8[0][0]=0;
				u8[1][0]=-0.2;
			}
		if(t>=20)
			{
				u4[0][0]=-0.6;
				u4[1][0]=0;
				u8[0][0]=0.6;
				u8[1][0]=0;
			}
		if(t>=30)
			{
				u4[0][0]=0;
				u4[1][0]=0;
				u8[0][0]=0;
				u8[1][0]=0;
			}
		u1[0][0] = B * K * (tello_pos1[0][0] - h1[0][0] - tello_pos4[0][0])  + B * v1[0][0] + B * u4[0][0] + 0.5 * B * F1[0][0];
		u2[0][0] = B * K * (tello_pos2[0][0] - h2[0][0] - tello_pos4[0][0])  + B * v2[0][0] + B * u4[0][0] + 0.5 * B * F2[0][0];  
		u3[0][0] = B * K * (tello_pos3[0][0] - h3[0][0] - tello_pos4[0][0])  + B * v3[0][0] + B * u4[0][0] + 0.5 * B * F3[0][0];  
		u5[0][0] = B * K * (tello_pos5[0][0] - h1[0][0] - tello_pos8[0][0])  + B * v1[0][0] + B * u8[0][0] + 0.5 * B * F4[0][0];
		u6[0][0] = B * K * (tello_pos6[0][0] - h2[0][0] - tello_pos8[0][0])  + B * v2[0][0] + B * u8[0][0] + 0.5 * B * F5[0][0];
		u7[0][0] = B * K * (tello_pos7[0][0] - h3[0][0] - tello_pos8[0][0])  + B * v3[0][0] + B * u8[0][0] + 0.5 * B * F6[0][0];

		u1[1][0] = B * K * (tello_pos1[1][0] - h1[1][0] - tello_pos4[1][0])  + B * v1[1][0] + B * u4[1][0] + 0.5 * B * F1[1][0];
		u2[1][0] = B * K * (tello_pos2[1][0] - h2[1][0] - tello_pos4[1][0])  + B * v2[1][0] + B * u4[1][0] + 0.5 * B * F2[1][0];  
		u3[1][0] = B * K * (tello_pos3[1][0] - h3[1][0] - tello_pos4[1][0])  + B * v3[1][0] + B * u4[1][0] + 0.5 * B * F3[1][0];   
		u5[1][0] = B * K * (tello_pos5[1][0] - h1[1][0] - tello_pos8[1][0])  + B * v1[1][0] + B * u8[1][0] + 0.5 * B * F4[1][0];
		u6[1][0] = B * K * (tello_pos6[1][0] - h2[1][0] - tello_pos8[1][0])  + B * v2[1][0] + B * u8[1][0] + 0.5 * B * F5[1][0];
		u7[1][0] = B * K * (tello_pos7[1][0] - h3[1][0] - tello_pos8[1][0])  + B * v3[1][0] + B * u8[1][0] + 0.5 * B * F6[1][0];
 
		cout<<"u1:"<<u1[0][0]<<"	"<<u1[1][0]<<endl;
		cout<<"u2:"<<u2[0][0]<<"	"<<u2[1][0]<<endl;
		cout<<"u3:"<<u3[0][0]<<"	"<<u3[1][0]<<endl;
		cout<<"u4:"<<u4[0][0]<<"	"<<u4[1][0]<<endl;
		cout<<"u5:"<<u5[0][0]<<"	"<<u5[1][0]<<endl;
				cmd_vel1.linear.x=u1[0][0];
				cmd_vel1.linear.y=u1[1][0];
				cmd_vel2.linear.x=u2[0][0];
				cmd_vel2.linear.y=u2[1][0];
				cmd_vel3.linear.x=u3[0][0];
				cmd_vel3.linear.y=u3[1][0];
				cmd_vel4.linear.x=u4[0][0];
				cmd_vel4.linear.y=u4[1][0];
				cmd_vel5.linear.x=u5[0][0];
				cmd_vel5.linear.y=u5[1][0];
				cmd_vel6.linear.x=u6[0][0];
				cmd_vel6.linear.y=u6[1][0];
				cmd_vel7.linear.x=u7[0][0];
				cmd_vel7.linear.y=u7[1][0];
				cmd_vel8.linear.x=u8[0][0];
				cmd_vel8.linear.y=u8[1][0];
				//cout<<"error:"<<tello_pos1[0][0] - h1[0][0] - tello_pos5[0][0]<<endl;
				vel_pub1.publish(cmd_vel1);
				vel_pub2.publish(cmd_vel2);
				vel_pub3.publish(cmd_vel3);
				vel_pub4.publish(cmd_vel4);
				vel_pub5.publish(cmd_vel5);
				vel_pub6.publish(cmd_vel6);
				vel_pub7.publish(cmd_vel7);
				vel_pub8.publish(cmd_vel8);
			}	
		if (command == 3)  // 巡逻编队
			{		h1[0][0]=0;
					h1[1][0]=-2;
					h2[0][0]=0;
					h2[1][0]=-1.5;
					h3[0][0]=0;
					h3[1][0]=-1;
					h4[0][0]=0;
					h4[1][0]=-0.5;		

					v1[0][0]=0;
					v1[1][0]=0;
					v2[0][0]=0;
					v2[1][0]=0;
					v3[0][0]=0;
					v3[1][0]=0;
					v4[0][0]=0;
					v4[1][0]=0;
					//cout<<"v1x"<<v1[0][0]<<endl;
					//cout<<"v1y"<<v1[1][0]<<endl;
					X[0]=tello_pos1[0][0];
					X[1]=tello_pos2[0][0];
					X[2]=tello_pos3[0][0];
					X[3]=tello_pos4[0][0];
					X[4]=1;
					X[5]=2;

					Y[0]=tello_pos1[1][0];
					Y[1]=tello_pos2[1][0];
					Y[2]=tello_pos3[1][0];
					Y[3]=tello_pos4[1][0];
					Y[4]=0.6;
					Y[5]=2.8;

		for(i=0;i<6;i++)
		{
			for(j=0;j<6;j++)
			{
				D[i][j]=sqrt((X[i] - X[j])*(X[i] - X[j]) + (Y[i] - Y[j])*(Y[i] - Y[j]));	//distance
				//cout<<"D:"<<D[i][j]<<endl;			
			}
		}
			//cout<<"D1:"<<D[4][5]<<endl;
			//cout<<"D2:"<<D[5][4]<<endl;
		for(i=0;i<4;i++)
		{
			for(j=0;j<6;j++)
			{
				if((D[i][j]!=0) && (D[i][j]<=ro))
				{
					Fx[i][j]=K_rep*(1/D[i][j]-1/ro)*((1/D[i][j])*(1/D[i][j])*(1/D[i][j]))*(X[i]-X[j]);
					Fy[i][j]=K_rep*(1/D[i][j]-1/ro)*((1/D[i][j])*(1/D[i][j])*(1/D[i][j]))*(Y[i]-Y[j]);
					ans++;
					//cout<<"ans:"<<ans<<endl;
				}
				else
				{
					Fx[i][j]=0;
					Fy[i][j]=0;
				}
			}		
		}
		for(i=0;i<4;i++)
		{
			cgemaFx[i]=0;
			cgemaFy[i]=0;		
		}
		
		for(i=0;i<4;i++)
		{
			for(j=0;j<6;j++)
			{
				cgemaFx[i]+=Fx[i][j];
				cgemaFy[i]+=Fy[i][j];			
			}
		}
		F1[0][0]	=	cgemaFx[0];
		F1[1][0]	=	cgemaFy[0];
		F2[0][0]	=	cgemaFx[1];
		F2[1][0]	=	cgemaFy[1];
		F3[0][0]	=	cgemaFx[2];
		F3[1][0]	=	cgemaFy[2];
		F4[0][0]	=	cgemaFx[3];
		F4[1][0]	=	cgemaFy[3];
		
		//cout<<"F1x:"<<F1[0][0]<<endl;
		//cout<<"F1y:"<<F1[1][0]<<endl;
		if(t>0)
			{	
				u5[0][0]=0.3;
				u5[1][0]=0;
			}
		if(t>=10)
			{
				u5[0][0]=0;
				u5[1][0]=0.4;
			}
		if(t>=15)
			{
				u5[0][0]=-0.3;
				u5[1][0]=0;
			}
		if(t>=25)
			{
				u5[0][0]=0;
				u5[1][0]=0;
			}
		u1[0][0] = B * K * (tello_pos1[0][0] - h1[0][0] - tello_pos5[0][0])  + B * v1[0][0] + B * u5[0][0] + 0.5 * B * F1[0][0];//
		u2[0][0] = B * K * (tello_pos2[0][0] - h2[0][0] - tello_pos5[0][0])  + B * v2[0][0] + B * u5[0][0] + 0.5 * B * F2[0][0];//+ B * K * (tello_pos2[0][0] - h2[0][0] - tello_pos5[0][0])  
		u3[0][0] = B * K * (tello_pos3[0][0] - h3[0][0] - tello_pos5[0][0])  + B * v3[0][0] + B * u5[0][0] + 0.5 * B * F3[0][0];//+ B * K * (tello_pos3[0][0] - h3[0][0] - tello_pos5[0][0])  
		u4[0][0] = B * K * (tello_pos4[0][0] - h4[0][0] - tello_pos5[0][0])  + B * v4[0][0] + B * u5[0][0] + 0.5 * B * F4[0][0];//+ B * K * (tello_pos4[0][0] - h4[0][0] - tello_pos5[0][0])  
		u1[1][0] = B * K * (tello_pos1[1][0] - h1[1][0] - tello_pos5[1][0])  + B * v1[1][0] + B * u5[1][0] + 0.5 * B * F1[1][0];//
		u2[1][0] = B * K * (tello_pos2[1][0] - h2[1][0] - tello_pos5[1][0])  + B * v2[1][0] + B * u5[1][0] + 0.5 * B * F2[1][0];//+ B * K * (tello_pos2[1][0] - h2[1][0] - tello_pos5[1][0])  
		u3[1][0] = B * K * (tello_pos3[1][0] - h3[1][0] - tello_pos5[1][0])  + B * v3[1][0] + B * u5[1][0] + 0.5 * B * F3[1][0];//+ B * K * (tello_pos3[1][0] - h3[1][0] - tello_pos5[1][0])   
		u4[1][0] = B * K * (tello_pos4[1][0] - h4[1][0] - tello_pos5[1][0])  + B * v4[1][0] + B * u5[1][0] + 0.5 * B * F4[1][0];//+ B * K * (tello_pos4[1][0] - h4[1][0] - tello_pos5[1][0])  
		cout<<"u1:"<<u1[0][0]<<"	"<<u1[1][0]<<endl;
		cout<<"u2:"<<u2[0][0]<<"	"<<u2[1][0]<<endl;
		cout<<"u3:"<<u3[0][0]<<"	"<<u3[1][0]<<endl;
		cout<<"u4:"<<u4[0][0]<<"	"<<u4[1][0]<<endl;
		cout<<"u5:"<<u5[0][0]<<"	"<<u5[1][0]<<endl;
				cmd_vel1.linear.x=u1[0][0];
				cmd_vel1.linear.y=u1[1][0];
				cmd_vel2.linear.x=u2[0][0];
				cmd_vel2.linear.y=u2[1][0];
				cmd_vel3.linear.x=u3[0][0];
				cmd_vel3.linear.y=u3[1][0];
				cmd_vel4.linear.x=u4[0][0];
				cmd_vel4.linear.y=u4[1][0];
				cmd_vel5.linear.x=u5[0][0];
				cmd_vel5.linear.y=u5[1][0];
				//cout<<"error:"<<tello_pos1[0][0] - h1[0][0] - tello_pos5[0][0]<<endl;
				vel_pub1.publish(cmd_vel1);
				vel_pub2.publish(cmd_vel2);
				vel_pub3.publish(cmd_vel3);
				vel_pub4.publish(cmd_vel4);
				vel_pub5.publish(cmd_vel5);
			}
		if(command == 9)  //降落
			{	
				std_msgs::Empty land_cmd;
				land_pub1.publish(land_cmd);
				land_pub2.publish(land_cmd);	
				land_pub3.publish(land_cmd);	
				land_pub4.publish(land_cmd);	
				land_pub5.publish(land_cmd);								
			}	
		ros::spinOnce();
		loop_rate.sleep(); 
	}
	return 0;
}
