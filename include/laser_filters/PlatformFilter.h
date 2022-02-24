#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <laser_filters/Polygon_array.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <queue>
#include <vector>
#include <string>
#include <limits>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include "filters/filter_base.h"
#include <sensor_msgs/LaserScan.h>

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
		void PlatformCallBack(const laser_filters::Polygon_array::ConstPtr& msg);
		bool isIntersection(float angle, double range);
		bool isOnPlatform();
		line_segment calculateLine(double, double, double , double);
		bool exactlyPlatform(line_segment* scan, std::string polygon, std::vector<geometry_msgs::Point32>* points_of_platform);
		void tf_update();
		bool boost_intersection(std::vector<geometry_msgs::Point32> *points_of_platform, double l_end_x, double l_end_y);
		void carry_lines();
    void calculate_direction(double*, line_segment, double);

		ros::Subscriber platfrom_sub_;
		tf::TransformListener tf_listener_;//for finding laser's position 

		
		double laser_x_, laser_y_, laser_z_, laser_yaw_;//coordinate of laser according to map
    std::vector<double> pitches_;
		std::vector<geometry_msgs::Polygon> platform_array_;
    std::vector<std::string> strings_of_polygons_;
		
		bool platforms_ready_;
		std::queue<std::vector<float>::iterator> platform_angle_range_;
};

}
