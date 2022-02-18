#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <simple_layers/Polygon_array.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <queue>
#include <vector>
#include <string>
#include <limits>
#include "filters/filter_base.h"
#include <sensor_msgs/LaserScan.h>
#include <limits>

namespace delete_platform_namespace{
	
	struct line_segment{//mx + b [x1:x2, y1:y2]
		double m;//slope
		double b;//constant
		double range_x1;//min x value
		double range_x2;//max x value
		double range_y1;//min y value
		double range_y2;//max y value
	};

class PlatformFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
	public:
		PlatformFilter();
		bool update(const sensor_msgs::LaserScan& data_in, sensor_msgs::LaserScan& data_out);
		virtual bool configure();
  
	private:
		void PlatformCallBack(const simple_layers::Polygon_array::ConstPtr& msg);
		bool isIntersection(float angle, double scan_data__range_index);
		bool isOnPlatform();
		line_segment calculateLine(double, double, double , double);
		bool exactlyPlatfrom(line_segment* scan, line_segment* platform);

		//ros::Subscriber laser_scan_sub_;//filtered scanning data
		ros::Subscriber platfrom_sub_;
		//ros::Subscriber robot_position_sub_;
		tf::TransformListener tf_listener_;//for finding laser's position 

		
		double laser_x_, laser_y_, laser_z_, robot_yaw_;//coordinate of laser according to map
		std::vector<geometry_msgs::Polygon> platform_array_;
		bool data_read_, platforms_ready_;
		std::queue<std::vector<float>::iterator> platform_angle_range_;
};

}
