#include <ros/ros.h>
#include <iostream>
#include <can_msgs/Frame.h>
#include <can_msgs/FrameArray.h>
#include <transmiter/AreasInfo.h>
#include <transmiter/EnvInfo.h>

#define __NAME__ "transmiter_node"

class Transmiter
{
public:
	Transmiter()
	{
		areas_info_frames_.header.frame_id = "ch1";
		areas_info_frames_.frames.resize(1);
		areas_info_frames_.frames[0].id = 0x400;
		areas_info_frames_.frames[0].len = 8;

		env_info_frames_.header.frame_id = "ch1";
		env_info_frames_.frames.resize(1);
		env_info_frames_.frames[0].id = 0x401;
		env_info_frames_.frames[0].len = 8;
	}
	~Transmiter(){};
	bool init()
	{
		ros::NodeHandle nh,nh_private("~");

		std::string to_can_topic = nh_private.param<std::string>("to_can_topic","to_can_topic");
		std::string from_can_topic = nh_private.param<std::string>("from_can_topic","from_can_topic");

		sub_area_info_ = nh.subscribe("sweeper/area_info",1,&Transmiter::areasInfoCallback,this);
		sub_env_info_  = nh.subscribe("sweeper/env_info", 1,&Transmiter::envInfoCallback,this);

		pub_can_msgs_  = nh.advertise<can_msgs::FrameArray>(to_can_topic,1);
	}

	void areasInfoCallback(const transmiter::AreasInfo::ConstPtr& areas_info)
	{
		can_msgs::Frame& frame = areas_info_frames_.frames[0];

		for(auto& byte:frame.data) byte = 0;

		for(size_t i=0; i< areas_info->infos.size(); ++i)
		{
			const transmiter::AreaInfo& areaInfo = areas_info->infos[i];
			if(areaInfo.area_id <=0 || areaInfo.area_id>8)
			{
				ROS_ERROR("[%s] area id %d is invalid.",__NAME__, areaInfo.area_id);
				continue;
			}
			if(areaInfo.rubbish_grade > 8)
			{
				ROS_ERROR("[%s] rubbish grade %d is invalid, in area %d.",__NAME__,areaInfo.rubbish_grade, areaInfo.area_id);
				continue;
			}
			if(areaInfo.vegetation_type >3)
			{
				ROS_ERROR("[%s] vegetation type %d is invalid, in area %d.",__NAME__,areaInfo.vegetation_type, areaInfo.area_id);
				continue;
			}
			
			if(areaInfo.has_person)
				frame.data[4] |= (1<<(areaInfo.area_id-1));
			frame.data[(areaInfo.area_id-1)/2] |= areaInfo.rubbish_grade << (4*((areaInfo.area_id-1)%2));
			frame.data[(areaInfo.area_id-1)/4+5] |= areaInfo.vegetation_type << 2*((areaInfo.area_id-1)%4);
 
		}
		pub_can_msgs_.publish(areas_info_frames_);
	}

	void envInfoCallback(const transmiter::EnvInfo::ConstPtr& env_info)
	{
		can_msgs::Frame& frame = env_info_frames_.frames[0];
		//frame.data[0]
		pub_can_msgs_.publish(env_info_frames_);
	}

private:
	ros::Subscriber sub_area_info_;
	ros::Subscriber sub_env_info_;
	ros::Publisher  pub_can_msgs_;

	can_msgs::FrameArray areas_info_frames_;

	can_msgs::FrameArray env_info_frames_;
};


int main(int argc, char**argv)
{
	ros::init(argc,argv,"transmiter_node");
	Transmiter app;
	if(app.init())
		ros::spin();
	return 0;
}
