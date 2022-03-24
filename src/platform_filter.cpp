#include <laser_filters/platform_filter.h>

namespace laser_filters{

  bool PlatformFilter::configure(){
    ros::NodeHandle nh;
    platform_sub_ = nh.subscribe("/platform_zone", 1000, &PlatformFilter::PlatformZoneCallBack, this);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("/vis_platforms", 1000);
    marker_line_pub_ = nh.advertise<visualization_msgs::Marker>("/vis_fitting_line", 1000);
    platforms_ready_ = false;
    is_on_ground_ = true;
    platforms_id_ = "";
    tfUpdate();
    ros::spinOnce();
    
    bool conf = getParam("tolerance", tolerance_) && getParam("max_distance", max_distance_) && getParam("number_skipped_angle", skipped_angle_);
    if(conf)
    {
      ROS_INFO("Configuration completed.");
      ROS_INFO("Platforms are waiting");
      return true;
    }
    ROS_WARN("Configuration could not completed. Platform parameters are not reachable");
    return false;
  }

  PlatformFilter::PlatformFilter(){//I may need to carry that transformation to the configure method (so I did:) but it was also wrong)
   
  }

  /*marking the ranges of intersection of scan data to the platform*/
  bool PlatformFilter::update(const sensor_msgs::LaserScan& data_in, sensor_msgs::LaserScan& data_out)
  {
    tfUpdate();//taking tf data
    data_out = data_in;
    platform_lines_.clear();//because of not filling again and again
    if(platforms_ready_ )//is published the platforms by the user
    {
      visualizePlatforms();
      float angle = data_in.angle_min;
      for (uint16_t i=0; i < data_out.ranges.size(); i++)//iteration all scan data
      {
        if(data_out.ranges[i] >= data_out.range_min && data_out.ranges[i] <= data_out.range_max)
        {
          int index_of_polygons = isOnPlatform(angle, data_out.ranges[i]);
          if(index_of_polygons != -1)//questions is really coming from platform
          {
            scan_data scan;
            scan.index = i;
            scan.range = data_out.ranges[i];
            scan.angle = angle;
            platform_lines_[polygons_data_[index_of_polygons].string_of_zone].push_back(scan);
          }
        }
        angle+=data_out.angle_increment;
        if(angle > data_out.angle_max)
          angle = data_out.angle_max;
      }

      int index_of_platform = 0;// for finding platforms' index
      for(polygons pol : polygons_data_)
      {
         if(!platform_lines_[pol.string_of_zone].empty())
         {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            cloud->height = 1;//values are unorganized
            cloud->is_dense = true;//values are valid (not included nan values)
            double sum_of_distance = 0, last_x = 0, last_y = 0;
            float last_angle = data_out.angle_min;
            bool is_first_it = true;//is first iteration
            int count = 0;
            //ROS_INFO("polygons_data_size:%i\nplatform_line size:%i\n", polygons_data_.size(), platform_lines_[pol.string_of_zone].size());
            for(scan_data scan : platform_lines_[pol.string_of_zone])//copy the data to pcl::cloud
            {
              double angle_of_alfa = scan.angle + laser_yaw_;
              float scanning_point_y = laser_y_ + (scan.range * std::sin(angle_of_alfa));//according to map
              float scanning_point_x = laser_x_ + (scan.range * std::cos(angle_of_alfa));
             //ROS_INFO("scan data\tx:%f,y:%f\n", scanning_point_x, scanning_point_y);
              pcl::PointXYZ point(scanning_point_x, scanning_point_y, 0);//creating a new point
              if(!is_first_it)
              {
                if( (scan.angle - last_angle) < (data_out.angle_increment * (skipped_angle_+1)) )//can have a tolerance
                {
                  sum_of_distance += calculateDistance(last_x, last_y, scanning_point_x, scanning_point_y);
                  count+= (int)std::round( std::abs( (scan.angle - last_angle) / data_out.angle_increment) );
                  //xROS_INFO("scanning angle:%f\n%f", scan.angle, std::abs( (scan.angle - last_angle) / data_out.angle_increment) );
                }
              }
              last_x = scanning_point_x;
              last_y = scanning_point_y;
              last_angle = scan.angle;
              is_first_it = false;
              cloud->push_back(point);
            }
            if(count == 0)
              continue;
            sum_of_distance /= count;
            //ROS_INFO("cloud size:%i\ndistance:%f\ncount:%i\nangle_increment:%f", cloud->size(), sum_of_distance, count, data_out.angle_increment);
            std::vector<int> inliers, indices;// will hold exception index of line
            Eigen::VectorXf vec;
            pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud->makeShared()));
            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p, sum_of_distance*2);//threshold is sum_of_distance
            
            ransac.computeModel();//compute the regression of line
            ransac.getInliers(inliers);//get line points' indexes
            ransac.getModel(indices);//not using now
            ransac.getModelCoefficients(vec);// it is 6d vector (last 3 element is direction vector on 3d)

            //std::cout << vec.x() << "\n"<< vec.y() << "\n"<<  vec.z() << "\n"<<vec.w() << "\n"<< "\n" << vec << "\n" << vec.rows() << "\n" << vec.cols();
            
            /*//not needed now. it is needed for looking final values
            pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*cloud, inliers, *final);
            //ROS_INFO("final size:%i", final->size());
            */
           
            visualizePlatforms(index_of_platform, vec);//show the last points of linestrıp on the calculated line on the rviz
            /*
            ROS_INFO("inlier size%i\t indices size:%i", inliers.size(), indices.size());
            ROS_INFO("indices1:%i, indices2:%i\n\n", indices[0], indices[1]);
            */
            int i = 0, j = 0;
            for(scan_data scan : platform_lines_[pol.string_of_zone])
            {
              if(inliers.size() != 0 && inliers[i] == j)
              {
                //ROS_INFO("index:%i     inliers:\t%i\nscan_index:%i", i, inliers[i],scan.index);
                data_out.ranges[scan.index] = std::numeric_limits<float>::quiet_NaN();
                i++;
              }

              j++;//scanning index
            }
         }
         index_of_platform++;
      }
       

    }
    return true;
  }

  /*memorize the platforms position*/
  void PlatformFilter::PlatformZoneCallBack(const laser_filters::polygon_array::ConstPtr& msg)
  {
    /*if it is a new call or first call*/
    if(platforms_id_.compare(msg->id.c_str()) != 0 || platforms_id_.compare("") == 0 )
    {
      platform_array_ = msg->points_to_points;
      pitches_ = msg->angles;
      platforms_id_ = msg->id;
      polygons_data_.clear();
      platforms_ready_ = CarryPolygons();//creating polygon strings that describe places where scan data is expected to come
      //CarryPolygon function will return bool because of controlling data accuracy
    }
    if(msg->id.compare("") == 0)//if id is not given, all platforms will not be executed
    {
      ROS_INFO("platforms ID did not given");
      platforms_ready_ = false;
    }
  }

  /*control the intersection of reading scanning data to the any of the platforms*/
  int PlatformFilter::isOnPlatform(float angle, double range)
  {
    double angle_of_alfa = angle + laser_yaw_;
    double scanning_point_y = laser_y_ + (range * std::sin(angle_of_alfa));//according to map
    double scanning_point_x = laser_x_ + (range * std::cos(angle_of_alfa));
    /*now we know that reading point, laser point, platforms' points*/
    //ROS_INFO("laser_x:%f,laser_y:%f\tlaser_yaw:%f\nscan point angle:%f\t(%f,%f)",laser_x_, laser_y_, laser_yaw_
    //                                                      , angle, scanning_point_x, scanning_point_y);

    std::vector<polygons>::iterator it_polygons = polygons_data_.begin();//has to be platform_array_ is same size 
    for (geometry_msgs::Polygon platform : platform_array_)//iteration all platforms
    {
      std::vector<geometry_msgs::Point32> points_of_platform = platform.points;

      std::string s_polygon;

      if(CloseEnough(&points_of_platform))//if it is far we do not control
      {
        if(isOnGround(it_polygons->string_of_zone) )
       	  s_polygon = it_polygons->string_of_polygon[0];//control the polygon that is on the zone
     	  else
          s_polygon = it_polygons->string_of_polygon[1];//control the polygon that is on the ground

        if(exactlyPlatform(scanning_point_x, scanning_point_y, s_polygon))
        {
          //ROS_INFO("DELETING...(%f,%f)", scanning_point_x, scanning_point_y);
          return it_polygons - polygons_data_.begin();
        }
      }
    it_polygons++;
    }
    return -1;
  }

  /*robots position(actually lidar positions) is on the platform or on the ground*/
  bool PlatformFilter::isOnGround(std::string str_polygon)//investigate
  {
    boost::geometry::model::d2::point_xy<double> point_2d(laser_x_, laser_y_);
    typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > polygon;
    polygon platform;
    boost::geometry::read_wkt(str_polygon, platform);
    return boost::geometry::disjoint(platform, point_2d);//if it is outside of the platform, it is on the ground
  }

  /*check is it close enough to control that platform*/
  bool PlatformFilter::CloseEnough(std::vector<geometry_msgs::Point32> *points_of_platform)//can be develop
  {
    double mid_beg_plat_x = (points_of_platform->front().x + (points_of_platform->begin() + 1)->x) / 2;
    double mid_beg_plat_y = (points_of_platform->front().y + (points_of_platform->begin() + 1)->y) / 2;
    double distance = calculateDistance(laser_x_, laser_y_, mid_beg_plat_x, mid_beg_plat_y); 
    return distance <= max_distance_;
  }

  /*control that is scan data coming from expected points that named str_polygon*/
  bool PlatformFilter::exactlyPlatform(double scan_x, double scan_y, std::string str_polygon)
  {
    typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > polygon;
    polygon platform;
    boost::geometry::model::d2::point_xy<double> point_2d(scan_x, scan_y);
    std::string scan_string;
    boost::geometry::read_wkt(str_polygon, platform);
    return !boost::geometry::disjoint(platform, point_2d);
  }

  /*collect the new positions data*/
  void PlatformFilter::tfUpdate()
  {
    tf::StampedTransform tf_transform_laser;
    bool tf_not_ready = true;
    while(tf_not_ready && ros::ok())
    {
      try{
        tf_listener_.lookupTransform("/map", "/base_scan", ros::Time(0), tf_transform_laser);//laser's position according to map
        tf_not_ready = false;
      }
      catch (tf::TransformException &ex){
        ROS_ERROR("%s",ex.what()); 
        ros::Duration(0.5).sleep();
        tf_not_ready = true;
      }
    }
      
    laser_x_ = tf_transform_laser.getOrigin().x();//coordinate of laser according to map
    laser_y_ = tf_transform_laser.getOrigin().y();
    laser_z_ = tf_transform_laser.getOrigin().z();
    laser_yaw_ = tf::getYaw(tf_transform_laser.getRotation());

  }

  /*platforms' lines taken carry to the zone and ground(named polygon)*/
  bool PlatformFilter::CarryPolygons()
  {
    if(pitches_.size() != platform_array_.size())
    {
      ROS_WARN("ANGLE VALUES AND PLATFORM VALUES NOT MATCHING");
      return false;
    }

    std::vector<double>::iterator pitch_angles_it = pitches_.begin();
    
    for (geometry_msgs::Polygon msg_pol : platform_array_ )//iteration all the platforms
    {
      polygons temp_pol;
      double yaw;
      std::vector<geometry_msgs::Point32> points_of_platform = msg_pol.points;
      if(points_of_platform.size() != 4)
      {
        ROS_WARN("PLATFORMS MUST HAVE 4 POINTS");
        return false;
      }
      if( *pitch_angles_it > 90.0)
      {
        ROS_WARN("ANGLE VALUES CAN NOT HIGHER THAN 90 degree");
        return false;
      }
      calculateYaw(&yaw, &points_of_platform);// calculate direction angle(yaw)
      calculateDirection(temp_pol.transport, *pitch_angles_it, yaw);//calculate the amount of displacement and assign to the temp.transport
      temp_pol.string_of_polygon[0] = "POLYGON((";//on the zone
      temp_pol.string_of_polygon[1] = "POLYGON((";//on the ground
      temp_pol.string_of_zone = "POLYGON((";//platforms' itself
      
      temp_pol.string_of_polygon[0] += std::to_string(points_of_platform[0].x + temp_pol.transport[0] + (std::cos(PI*yaw/180) * tolerance_)) + " "
                             + std::to_string(points_of_platform[0].y + temp_pol.transport[1] + (std::sin(PI*yaw/180) * tolerance_)) + ","
                             + std::to_string(points_of_platform[1].x + temp_pol.transport[0] + (std::cos(PI*yaw/180) * tolerance_)) + " "
                             + std::to_string(points_of_platform[1].y + temp_pol.transport[1] + (std::sin(PI*yaw/180) * tolerance_)) + ","
                             + std::to_string(points_of_platform[1].x + temp_pol.transport[0] - (std::cos(PI*yaw/180) * tolerance_)) + " "
                             + std::to_string(points_of_platform[1].y + temp_pol.transport[1] - (std::sin(PI*yaw/180) * tolerance_)) + ","
                             + std::to_string(points_of_platform[0].x + temp_pol.transport[0] - (std::cos(PI*yaw/180) * tolerance_)) + " "
                             + std::to_string(points_of_platform[0].y + temp_pol.transport[1] - (std::sin(PI*yaw/180) * tolerance_)) + ","
                             + std::to_string(points_of_platform[0].x + temp_pol.transport[0] + (std::cos(PI*yaw/180) * tolerance_)) + " "
                             + std::to_string(points_of_platform[0].y + temp_pol.transport[1] + (std::sin(PI*yaw/180) * tolerance_)) + "))";
      temp_pol.string_of_polygon[1] += std::to_string(points_of_platform[0].x - temp_pol.transport[0] + (std::cos(PI*yaw/180) * tolerance_)) + " "
                             + std::to_string(points_of_platform[0].y - temp_pol.transport[1] + (std::sin(PI*yaw/180) * tolerance_)) + ","
                             + std::to_string(points_of_platform[1].x - temp_pol.transport[0] + (std::cos(PI*yaw/180) * tolerance_)) + " "
                             + std::to_string(points_of_platform[1].y - temp_pol.transport[1] + (std::sin(PI*yaw/180) * tolerance_)) + ","
                             + std::to_string(points_of_platform[1].x - temp_pol.transport[0] - (std::cos(PI*yaw/180) * tolerance_)) + " "
                             + std::to_string(points_of_platform[1].y - temp_pol.transport[1] - (std::sin(PI*yaw/180) * tolerance_)) + ","
                             + std::to_string(points_of_platform[0].x - temp_pol.transport[0] - (std::cos(PI*yaw/180) * tolerance_)) + " "
                             + std::to_string(points_of_platform[0].y - temp_pol.transport[1] - (std::sin(PI*yaw/180) * tolerance_)) + ","
                             + std::to_string(points_of_platform[0].x - temp_pol.transport[0] + (std::cos(PI*yaw/180) * tolerance_)) + " "
                             + std::to_string(points_of_platform[0].y - temp_pol.transport[1] + (std::sin(PI*yaw/180) * tolerance_)) + "))";
      temp_pol.string_of_zone += std::to_string(points_of_platform[0].x) + " " + std::to_string(points_of_platform[0].y) + ","
                             + std::to_string(points_of_platform[1].x) + " " + std::to_string(points_of_platform[1].y) + ","
                             + std::to_string(points_of_platform[2].x) + " " + std::to_string(points_of_platform[2].y) + ","
                             + std::to_string(points_of_platform[3].x) + " " + std::to_string(points_of_platform[3].y) + ","
                             + std::to_string(points_of_platform[0].x) + " " + std::to_string(points_of_platform[0].y) + "))";

      polygons_data_.push_back(temp_pol);
      pitch_angles_it++;

      ROS_INFO("%ith polygon was carried. polygon on the zone:%s \npolygon on the ground:%s\npolygon zones itself:%s\ndirection_x:%f\tdirection_y:%f", (int)(pitch_angles_it - pitches_.begin())
      , temp_pol.string_of_polygon[0].c_str(), temp_pol.string_of_polygon[1].c_str(), temp_pol.string_of_zone.c_str(), temp_pol.transport[0], temp_pol.transport[1]);
    }
    return true;
  }

  /*calculate the displacement direction for the beginning line according to pitch and yaw ,and assign to variable*/
  void PlatformFilter::calculateDirection(double *direction, double pitch, double yaw)//pitch and yaw must be degree
  {
    direction[1] = std::sin(PI*yaw/180) * laser_z_ * (1.0/std::tan(PI*pitch/180));//y direction
    direction[0] = std::cos(PI*yaw/180) * laser_z_ * (1.0/std::tan(PI*pitch/180));
  }

  /*calculate middle of the polygon and set the yaw as a vector that is through beginning line's middle to that point*/
  void PlatformFilter::calculateYaw(double *yaw, std::vector<geometry_msgs::Point32> *vec)//return the degree of yaw
  {
    double middle_x, middle_y;
    for(geometry_msgs::Point32 point : *vec)//find middle of the platform
    {
      middle_x += point.x;
      middle_y += point.y;
    }
    middle_x /= vec->size();
    middle_y /= vec->size();
    double mid_beg_plat_x = ( vec->begin()->x + (vec->begin()+1)->x ) / 2;//middle of the beginning of platform
    double mid_beg_plat_y = ( vec->begin()->y + (vec->begin()+1)->y ) / 2;
    //ROS_INFO("org_plat_x:%forg_plat_y:%f beg_plat_x:%f\tbeg_plat_y:%f",middle_x, middle_y, mid_beg_plat_x, mid_beg_plat_y);
    if(middle_x - mid_beg_plat_x == 0)
    {
      if(middle_y - mid_beg_plat_y < 0)
        *yaw = 270;
      else if(middle_y - mid_beg_plat_y > 0)
        *yaw = 90;
    }
    else
      if(middle_y - mid_beg_plat_y == 0 && middle_x - mid_beg_plat_x < 0)
        *yaw = 180;
      else
        *yaw = std::atan((middle_y - mid_beg_plat_y) / (middle_x - mid_beg_plat_x))*180/PI;
  }

  void PlatformFilter::visualizePlatforms()//at the same Hz as update
  {
    int index = 0;
    std::vector<polygons>::iterator it_polygons = polygons_data_.begin();
    for (geometry_msgs::Polygon platform : platform_array_)//iteration all platforms
    {
      visualization_msgs::Marker marking_plat, marking_line;
      marking_plat.header.frame_id = "/map";
      marking_plat.header.stamp = ros::Time();
      marking_plat.ns = "platform";
      marking_plat.id = index++;
      marking_plat.type = visualization_msgs::Marker::CUBE;
      marking_plat.action = visualization_msgs::Marker::ADD;
      marking_plat.pose.position.x = (platform.points[0].x + platform.points[1].x + platform.points[2].x + platform.points[3].x) / 4;
      marking_plat.pose.position.y = (platform.points[0].y + platform.points[1].y + platform.points[2].y + platform.points[3].y) / 4;
      marking_plat.pose.position.z = 0;
      tf2::Quaternion q;
      double yaw;
      calculateYaw(&yaw, &platform.points);
      yaw = (yaw-90)*PI/180;
      q.setRPY(0, 0, yaw);
      q = q.normalize();
      marking_plat.pose.orientation.x = q.getX();
      marking_plat.pose.orientation.y = q.getY();
      marking_plat.pose.orientation.z = q.getZ();
      marking_plat.pose.orientation.w = q.getW();
      marking_plat.scale.x = calculateDistance(platform.points[0].x, platform.points[0].y, platform.points[1].x, platform.points[1].y);//weight of platform
      marking_plat.scale.y = calculateDistance(platform.points[0].x, platform.points[0].y, platform.points[3].x, platform.points[3].y);//length of platform
      marking_plat.scale.z = 0.001;
      marking_plat.color.a = 0.3; // Don't forget to set the alpha!
      marking_plat.color.r = 0.8*index;
      marking_plat.color.g = 0.1*index;
      marking_plat.color.b = 0.0;
      marker_pub_.publish(marking_plat);

      /*create line of expected points*/
      marking_line.header.frame_id = "/map";
      marking_line.header.stamp = ros::Time();
      marking_line.ns = "line";
      marking_line.id = index++;
      marking_line.type = visualization_msgs::Marker::LINE_STRIP;
      marking_line.action = visualization_msgs::Marker::ADD;
      geometry_msgs::Point point_1, point_2;
      if(isOnGround(it_polygons->string_of_zone))
      {
        point_1.x = platform.points[0].x + it_polygons->transport[0];
        point_1.y = platform.points[0].y + it_polygons->transport[1];
        point_2.x = platform.points[1].x + it_polygons->transport[0];
        point_2.y = platform.points[1].y + it_polygons->transport[1];
      }
      else
      {
        point_1.x = platform.points[0].x - it_polygons->transport[0];
        point_1.y = platform.points[0].y - it_polygons->transport[1];
        point_2.x = platform.points[1].x - it_polygons->transport[0];
        point_2.y = platform.points[1].y - it_polygons->transport[1];
      }
      marking_line.points.push_back(point_1);
      marking_line.points.push_back(point_2);
      marking_line.pose.orientation.w = 1.0;
      marking_line.scale.x = tolerance_*2;
      marking_line.scale.y = tolerance_*2;
      marking_line.scale.z = 0;
      marking_line.color.a = 0.25;
      marking_line.color.r = 0.0;
      marking_line.color.b = 1.0;
      marker_pub_.publish(marking_line);
      it_polygons++;
    }
  }

  /*visualize fitting line for every platform individually*/
  void PlatformFilter::visualizePlatforms(int index_of_platform, Eigen::VectorXf& vec)
  {
    visualization_msgs::Marker marking_line;
    double fitting_line[2];
    fitting_line[0] = (vec[4]/vec[3]);//std::tan(tf::getYaw())
    fitting_line[1] = vec[1] - fitting_line[0] * vec[0];
    //ROS_INFO("yaw:%f ", tf::getYaw(qua1)*180/PI);
    //ROS_INFO("fit m: %f, fit n: %f", fitting_line[0], fitting_line[1]);
    std::vector<geometry_msgs::Point32> points_of_platform = platform_array_[index_of_platform].points;
    double line_1[2];//mx + n(m, n)
    calculateLine(line_1, points_of_platform[0].x, points_of_platform[0].y, points_of_platform[3].x, points_of_platform[3].y);
    double line_2[2];//mx + n(m, n)
    calculateLine(line_2, points_of_platform[1].x, points_of_platform[1].y, points_of_platform[2].x, points_of_platform[2].y);
    //ROS_INFO("n_1:%f n_2%f", line_1[1], line_2[1]);
    marking_line.header.frame_id = "/map";
    marking_line.header.stamp = ros::Time();
    marking_line.ns = "line";
    marking_line.id = 55;
    marking_line.type = visualization_msgs::Marker::LINE_STRIP;
    marking_line.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point point_msg;
    calculateCommonPoint(point_msg, fitting_line[0], fitting_line[1], line_1[0], line_1[1]);
    marking_line.points.push_back(point_msg);
    calculateCommonPoint(point_msg, fitting_line[0], fitting_line[1], line_2[0], line_2[1]);
    marking_line.points.push_back(point_msg);
    
    marking_line.pose.orientation.w = 1.0;
    marking_line.scale.x = 0.1;
    marking_line.scale.y = 0.1;
    marking_line.scale.z = 0;
    marking_line.color.a = 1;
    marking_line.color.r = 1;
    marking_line.color.g = 1;
    marking_line.color.b = 0.0;

    marker_line_pub_.publish(marking_line);
  }
  
  /*calculate the line according to given points and return slope value [0](m) and constant value [1](n)*/
  void PlatformFilter::calculateLine(double* line, double beg_x, double beg_y, double end_x, double end_y)
  {
    if(end_x - beg_x == 0)
    {
      if(end_y - beg_y < 0)
        line[0] = std::tan(270*PI/180);
      else if(end_y - beg_y > 0)
        line[0] = std::tan(90*PI/180);
    }
    else
      if(end_y - beg_y == 0 && end_x - beg_x < 0)
        line[0] = std::tan(180*PI/180);
      else
        line[0] = (end_y - beg_y) / (end_x - beg_x);
    line[1] = end_y - line[0] * end_x;
  }
  

  /*calculate the common solution for two lines given*/
  void PlatformFilter::calculateCommonPoint(geometry_msgs::Point& out, double m_1, double n_1, double m_2, double n_2)
  {
    out.x = 0;
    if(m_1 != m_2)
      out.x = (n_2 - n_1) / (m_1 - m_2);
    else
      ROS_ERROR("INAPPROPRIATE LINE VALUES ARE REACHED");
    out.y = m_2 * out.x + n_2;
  }

  double PlatformFilter::calculateDistance(double beg_x, double beg_y, double end_x, double end_y)
  {
    double change_x = beg_x - end_x;
    double change_y = beg_y - end_y;
    return std::sqrt((change_y*change_y) + (change_x*change_x)); 
  }
}
