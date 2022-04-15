#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "filters/filter_base.h"
#include "milvus_msgs/MapFeaturesStamped.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <cmath>
#include <vector>
#include <string>
#include <limits>
//boost library
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
//pcl library
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
//dynamic reconfiguretion needed
#include <dynamic_reconfigure/server.h>
#include <laser_filters/PlatformFilterConfig.h>

namespace laser_filters{
	constexpr double PI = std::acos(-1);//pi value
  

  struct platform_cloud{//points data on the one platform 
    pcl::PointCloud<pcl::PointXYZ> cloud;//points on the platform
    int count = 0;//hold how many points space .<->.
    std::vector<int> index_array;//points' indexes in the general cloud msg
    double sum_of_distance = 0;//sum of distance between points
    bool is_first_it = true;//it it the first iteration on this platform
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
    void platformZoneCallBack(const milvus_msgs::MapFeaturesStamped::ConstPtr& msg);
    void reconfigureCB(laser_filters::PlatformFilterConfig& config, uint32_t level);
		int isOnPlatform(float scan_x, float scan_y);
    bool exactlyPlatform(double, double, std::string polygon);//if platform close enough this uses exactly platform
    bool closeEnough(std::vector<geometry_msgs::Pose2D> &);//control the is robots' laser frame close enough to beginning of the platform
    bool isOnGround(std::string);//where the robot's laser frame
		bool tfUpdate(ros::Time); 
		bool carryPolygons();//calculate and carry the expected intersection points of platforms
    void calculateDirection(double*, double, double);//calculate the expected points carrying amount on the x and y axis
    void calculateYaw(double&, std::vector<geometry_msgs::Pose2D>&);//according to middle and beg of platform, calculate the yaw of platform
    void visualizePlatforms();//visualize platforms' themself and expected spaces
    void visualizePlatforms(int, Eigen::VectorXf&);//visualize fitting line
    void calculateLine(double& coef_y, double& line_m,  double& line_n,double beg_x, double beg_y, double end_x, double end_y);//calculate m and n values according to input
    void calculateCommonPoint(geometry_msgs::Point& , double m_1, double n_1, double coef_y1, double m_2, double n_2, double coef_y2);//calculate and return the point according to m and n values
    double calculateDistance(double beg_x, double beg_y, double end_x, double end_y);

    ros::Subscriber platform_sub_;
    ros::Publisher marker_pub_;//for publishing platforms and their calculated space
    tf::TransformListener tf_listener_;//for finding laser's position 
    std::shared_ptr<dynamic_reconfigure::Server<laser_filters::PlatformFilterConfig>> dyn_server_;
    boost::recursive_mutex own_mutex_;
    bool conf_;//have parameters got succesfully
		/*parameters*/
    double tolerance_;
    double max_distance_;
    int skipped_angle_;
    double threshold_coef_;//for ransac
    double angle_threshold_;
    std::string laser_frame_;//coming as a parameter
    std::string map_frame_;//coming as a parameter

    double laser_x_, laser_y_, laser_z_, laser_yaw_;//coordinate of laser according to map
    std::vector<milvus_msgs::MapFeature> platform_array_;//store the values coming from message
    std::vector<polygons> polygons_data_;//store the calculated and creating values of platforms
    std::string platforms_id_;//message name coming from outside
    bool platforms_ready_;//when platform data came, it is true

    bool is_on_ground_;// if robot not on the upside of platform or on the platform, it is true
    std::map<int, std::shared_ptr<platform_cloud>> platform_lines_;//holding that platforms' platform cloud value
};

}
