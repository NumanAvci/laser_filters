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
	const double PI = std::acos(-1);

	struct line_segment{//mx + b [x1:x2, y1:y2]
		double m;//slope
		double b;//constant
		double range_x1;//min x value
		double range_x2;//max x value
		double range_y1;//min y value
		double range_y2;//max y value
	};

  struct polygons{
    std::string string_of_polygon[2];//[1] on the zone, [0] on the platform
    std::string string_of_zone;//platform's own polygon
    double transport[2];//delta x and delta y for carrying polygon
  };

class PlatformFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
	public:
		PlatformFilter();
		bool update(const sensor_msgs::LaserScan& data_in, sensor_msgs::LaserScan& data_out);
		virtual bool configure();
  
	private:
		void PlatformZoneCallBack(const laser_filters::Polygon_array::ConstPtr& msg);
		bool isIntersection(float angle, double range);
		line_segment calculateLine(double, double, double , double);
		bool exactlyPlatform(line_segment* scan, std::string polygon);
    bool CloseEnough(line_segment* scan, std::vector<geometry_msgs::Point32> *);
    bool isOnGround(std::string);
		void tf_update();
		void CarryPolygons();
    void calculate_direction(double*, double, double);
    void calculate_yaw(double*, std::vector<geometry_msgs::Point32>*);

		ros::Subscriber platfrom_sub_;
		tf::TransformListener tf_listener_;//for finding laser's position 

		
		double laser_x_, laser_y_, laser_z_, laser_yaw_;//coordinate of laser according to map
    std::vector<double> pitches_;
		std::vector<geometry_msgs::Polygon> platform_array_;
    std::vector<polygons> polygons_data_;
		
		bool platforms_ready_;//when platform data came, it is true
    bool is_on_ground_;// if robot not on the upside of platform or on the platform, it is true
		std::queue<float*> platform_angle_range_;
    std::string platforms_id_;
};

}
