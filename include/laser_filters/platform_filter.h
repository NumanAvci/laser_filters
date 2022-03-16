#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <laser_filters/polygon_array.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
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

namespace laser_filters{
	constexpr double PI = std::acos(-1);//pi value
  
  struct scan_data{
    uint16_t index;
    double range;
    float angle;
  };

  struct polygons{
    std::string string_of_polygon[2];//[1] on the ground, [0] on the platform
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
		void PlatformZoneCallBack(const laser_filters::polygon_array::ConstPtr& msg);
		const std::string&  isIntersection(float angle, double range);
		bool exactlyPlatform(double, double, std::string polygon);
    bool CloseEnough(std::vector<geometry_msgs::Point32> *);
    bool isOnGround(std::string);
		void tfUpdate();
		void CarryPolygons();
    void calculateDirection(double*, double, double);
    void calculateYaw(double*, std::vector<geometry_msgs::Point32>*);
    void visualizePlatforms();
    double calculateDistance(double , double , double , double );

		ros::Subscriber platfrom_sub_;
    ros::Publisher marker_pub_;
		tf::TransformListener tf_listener_;//for finding laser's position 

		
		double laser_x_, laser_y_, laser_z_, laser_yaw_;//coordinate of laser according to map
    std::vector<double> pitches_;//store the pitch angles of platforms
		std::vector<geometry_msgs::Polygon> platform_array_;//store the values coming from message
    std::vector<polygons> polygons_data_;//store the calculated and creating values of platforms
		
		bool platforms_ready_;//when platform data came, it is true
    bool is_on_ground_;// if robot not on the upside of platform or on the platform, it is true
		std::map<std::string, std::vector<scan_data>> platform_lines_;//holding that platforms'
    std::string platforms_id_;//message name coming from outside
};

}
