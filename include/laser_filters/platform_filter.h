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

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet.h>
#include <tf2_ros/buffer.h>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <dynamic_reconfigure/server.h>
#include <laser_filters/PlatformFilterConfig.h>
#include "filters/filter_base.h"
#include <sensor_msgs/LaserScan.h>

namespace laser_filters{
	constexpr double PI = std::acos(-1);//pi value
  
  /*struct scan_data{
    uint16_t index;
    double range;
    float angle;
  };*/
  struct platform_cloud{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    int count = 0;
    std::vector<int> index_array;
    double sum_of_distance = 0;
    bool is_first_it = true;
  };

  struct polygons{
    std::string string_of_polygon[2];//[1] on the ground, [0] on the platform
    std::string string_of_zone;//platform's own polygon
    double transport[2];//delta x[0] and delta y[1] for carrying polygon
  };

class PlatformFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
	public:
		PlatformFilter();
		bool update(const sensor_msgs::LaserScan& data_in, sensor_msgs::LaserScan& data_out);
		virtual bool configure();
  
	private:
		void PlatformZoneCallBack(const laser_filters::polygon_array::ConstPtr& msg);
    void reconfigureCB(laser_filters::PlatformFilterConfig& config, uint32_t level);
		//int isOnPlatform(float angle, double range);
		int isOnPlatform(float scan_x, float scan_y);
    bool exactlyPlatform(double, double, std::string polygon);
    bool CloseEnough(std::vector<geometry_msgs::Point32> *);
    bool isOnGround(std::string);
		void tfUpdate(ros::Time);
    void indexBaseCountNAN(std::vector<int>&, const sensor_msgs::LaserScan&);
		bool CarryPolygons();
    void calculateDirection(double*, double, double);
    void calculateYaw(double*, std::vector<geometry_msgs::Point32>*);
    void visualizePlatforms();
    void visualizePlatforms(int, Eigen::VectorXf&);
    void calculateLine(double* , double , double , double , double );
    void calculateCommonPoint(geometry_msgs::Point& , double , double , double , double );
    double calculateDistance(double , double , double , double );

		ros::Subscriber platform_sub_;
    ros::Subscriber scan_sub_;
    ros::Publisher marker_pub_;//for publishing platforms and their calculated space
		tf::TransformListener tf_listener_;//for finding laser's position 
    std::shared_ptr<dynamic_reconfigure::Server<laser_filters::PlatformFilterConfig>> dyn_server_;
    boost::recursive_mutex own_mutex_;
		
		double laser_x_, laser_y_, laser_z_, laser_yaw_;//coordinate of laser according to map
    double tolerance_;
    double max_distance_;
    int skipped_angle_;
    double threshold_coef_;
    std::vector<double> pitches_;//store the pitch angles of platforms
		std::vector<geometry_msgs::Polygon> platform_array_;//store the values coming from message
    std::vector<polygons> polygons_data_;//store the calculated and creating values of platforms
		
		bool platforms_ready_;//when platform data came, it is true
    bool is_on_ground_;// if robot not on the upside of platform or on the platform, it is true
		//std::map<std::string, std::vector<scan_data>> platform_lines_;//holding that platforms'
    std::map<std::string, platform_cloud> platform_lines_;//holding that platforms'
    std::string platforms_id_;//message name coming from outside
};

}
