#include <ros/ros.h>
#include <iostream>
#include <can_msgs/Frame.h>
#include <can_msgs/FrameArray.h>
#include <transmiter/AreasInfo.h>
#include <transmiter/EnvInfo.h>
#include <transmiter/Objects.h>

#define __NAME__ "transmiter_node"

class Transmiter
{
public:
	Transmiter()
	{
		can_channel = "ch1";
		area_info_can_id = 0x18FFF061;
		env_info_can_id = 0x18FFF161;
		obstacles_can_ids.push_back(0x18FFF361);
		obstacles_can_ids.push_back(0x18FFF461);
		obstacles_can_ids.push_back(0x18FFF561);
		obstacles_can_ids.push_back(0x18FFF661);
		obstacles_can_ids.push_back(0x18FFF761);
		obstacles_can_ids.push_back(0x18FFF861);
		obstacles_can_ids.push_back(0x18FFF961);
		obstacles_can_ids.push_back(0x18FFFA61);
		
		areas_info_frames_.header.frame_id = can_channel;
		areas_info_frames_.frames.resize(1);
		areas_info_frames_.frames[0].id = area_info_can_id;
		areas_info_frames_.frames[0].len = 8;

		env_info_frames_.header.frame_id = can_channel;
		env_info_frames_.frames.resize(1);
		env_info_frames_.frames[0].id = env_info_can_id;
		env_info_frames_.frames[0].len = 8;
		
		obstacle_frames_.header.frame_id = can_channel;
	}
	~Transmiter(){};
	bool init()
	{
		ros::NodeHandle nh,nh_private("~");

		std::string to_can_topic = nh_private.param<std::string>("to_can_topic","to_can_topic");
		std::string from_can_topic = nh_private.param<std::string>("from_can_topic","from_can_topic");

		sub_area_info_ = nh.subscribe("sweeper/area_info",1,&Transmiter::areasInfoCallback,this);
		sub_env_info_  = nh.subscribe("sweeper/env_info", 1,&Transmiter::envInfoCallback,this);
		sub_obstacle_  = nh.subscribe("sweeper/obstacles", 1, &Transmiter::obstaclesCallback, this);

		pub_can_msgs_  = nh.advertise<can_msgs::FrameArray>(to_can_topic,1);
	}
	
	void obstaclesCallback(const transmiter::Objects::ConstPtr& obstacles)
	{
		obstacle_frames_.header.stamp = obstacles->header.stamp;
		
		size_t cnt = obstacles->objects.size();
		if(cnt > 8) //障碍个数过多
		{
			alarmCode_ = 0x1; //warning
			cnt = 8;
		}
		else
		{
			alarmCode_ = 0x0;
		}
		
		obstacle_frames_.frames.resize(cnt);
		for(size_t i=0; i<cnt; ++i)
		{
			can_msgs::Frame& frame = obstacle_frames_.frames[i];
			frame.id = obstacles_can_ids[i];
			
			float x = obstacles->objects[i].x;
			float y = obstacles->objects[i].y;
			float w = obstacles->objects[i].w;
			float h = obstacles->objects[i].h;
			
			*(uint16_t *)(&frame.data[0]) = uint16_t(x*100);
			*(uint16_t *)(&frame.data[2]) = uint16_t(y*100);
			*(uint16_t *)(&frame.data[4]) = uint16_t(h*100);
			*(uint16_t *)(&frame.data[6]) = uint16_t(w*100);
			
		}
		pub_can_msgs_.publish(obstacle_frames_);
	}
	
	void areasInfoCallback(const transmiter::AreasInfo::ConstPtr& areas_info)
	{
		areas_info_frames_.header.stamp = areas_info->header.stamp;
		
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
			
			//行人
			if(areaInfo.has_person)
				frame.data[4] |= (1<<(areaInfo.area_id-1));
			//垃圾等级
			frame.data[(areaInfo.area_id-1)/2] |= areaInfo.rubbish_grade << (4*((areaInfo.area_id-1)%2));
			//植被类型
			frame.data[(areaInfo.area_id-1)/4+5] |= areaInfo.vegetation_type << 2*((areaInfo.area_id-1)%4);
			//报警信息
			frame.data[7] |= (alarmCode_&0x3);
 
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
	ros::Subscriber sub_obstacle_;
	
	ros::Publisher  pub_can_msgs_;

	can_msgs::FrameArray areas_info_frames_;
	can_msgs::FrameArray env_info_frames_;
	can_msgs::FrameArray obstacle_frames_;
	
	uint32_t area_info_can_id;
	uint32_t env_info_can_id;
	std::vector<uint32_t> obstacles_can_ids;
	std::string can_channel;
	uint8_t alarmCode_;
};


int main(int argc, char**argv)
{
	ros::init(argc,argv,"transmiter_node");
	Transmiter app;
	if(app.init())
		ros::spin();
	return 0;
}
