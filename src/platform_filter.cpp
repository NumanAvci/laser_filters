#include <laser_filters/platform_filter.h>

namespace laser_filters{

  bool PlatformFilter::configure(){
    ros::NodeHandle private_nh("~" + getName());
    ros::NodeHandle nh("/laser_filter");//for looking parent parameter 
    platform_sub_ = private_nh.subscribe("/platform_zone", 1000, &PlatformFilter::PlatformZoneCallBack, this);
    marker_pub_ = private_nh.advertise<visualization_msgs::Marker>("/vis_platforms", 1000);
    platforms_ready_ = false;
    conf_ = false;
    is_on_ground_ = true;
    platforms_id_ = "";
    PlatformFilterConfig config;
    dyn_server_.reset(new dynamic_reconfigure::Server<laser_filters::PlatformFilterConfig>(own_mutex_, private_nh));
    dynamic_reconfigure::Server<laser_filters::PlatformFilterConfig>::CallbackType f;
    f = boost::bind(&laser_filters::PlatformFilter::reconfigureCB, this, _1, _2);
    dyn_server_->setCallback(f);
    ros::spinOnce();
    conf_ = getParam("tolerance", tolerance_) && getParam("max_distance", max_distance_)
     && getParam("number_skipped_angle", skipped_angle_) && getParam("threshold_coefficient", threshold_coef_)
     && nh.getParam("laser_frame", laser_frame_) && nh.getParam("map_frame", map_frame_);
    config.max_distance = max_distance_;
    config.number_skipped_angle = skipped_angle_;
    config.threshold_coefficient = threshold_coef_;
    config.tolerance = tolerance_;
    tfUpdate(ros::Time(0));
    ROS_INFO("\nthreshold:\t%f\nmax_distance:\t%f\nskipped_angle:\t%d\nthreshold_coef:\t%f\nlaser_f:\t%s\nmap_f:\t%s"
    , tolerance_, max_distance_, skipped_angle_, threshold_coef_, laser_frame_.c_str(), map_frame_.c_str());
    if(conf_)
    {
      ROS_INFO("Configuration completed.");
      ROS_INFO("Platforms are waiting");
      return conf_;
    }
    ROS_WARN("Configuration could not completed. Platform parameters are not reachable");
    return conf_;
  }

  PlatformFilter::PlatformFilter(){//I may need to carry that transformation to the configure method (so I did:) but it was also wrong)
   
  }

  /*marking the ranges of intersection of scan data to the platform*/
  bool PlatformFilter::update(const sensor_msgs::LaserScan& data_in, sensor_msgs::LaserScan& data_out)
  {
    tfUpdate(data_in.header.stamp);//taking tf data
    data_out = data_in;
    if(platforms_ready_ )//is published the platforms by the user
    {
      visualizePlatforms();
      laser_geometry::LaserProjection projector;
      sensor_msgs::PointCloud2 cloud_msg;
      std::vector<int> number_of_NAN;
      indexBaseCountNAN(number_of_NAN, data_in);
      try{
        projector.transformLaserScanToPointCloud(map_frame_, data_in, cloud_msg, tf_listener_);
      }
      catch (tf::TransformException &ex){
        ROS_WARN("TransformException!");
        ROS_ERROR("%s",ex.what()); 
        return true;
      }
      
      sensor_msgs::PointCloud2ConstIterator<float> const_iter_x(cloud_msg, "x"),
                                                const_iter_y(cloud_msg, "y");
      
      int index_of_cloud = 0, last_index_of_cloud = 0;
      sensor_msgs::PointCloud2ConstIterator<float> const_last_iter_x(const_iter_x), const_last_iter_y(const_iter_y);
      
      for(int i=0;i < data_in.ranges.size();i++)//find points that are on the platforms
      {
        if( data_in.range_min < data_in.ranges[i]  && data_in.range_max > data_in.ranges[i] )
        {
          int index_of_platform = isOnPlatform(*const_iter_x, *const_iter_y);
          if(index_of_platform != -1)//questions is really coming from platform
          {
            platform_cloud* pl_struct = &platform_lines_[index_of_platform];//fill inside
            (*pl_struct).cloud.height = 1;//values are unorganized
            (*pl_struct).cloud.is_dense = true;//values are valid (not included nan values)
            pcl::PointXYZ point(*const_iter_x, *const_iter_y, 0);
            (*pl_struct).cloud.push_back(point);
            (*pl_struct).index_array.push_back(index_of_cloud);
            point_to_scan_index_map_[index_of_cloud] = i;
            if(!(*pl_struct).is_first_it)
            {
              if( ( index_of_cloud - last_index_of_cloud ) < (skipped_angle_+1) )
              {
                (*pl_struct).sum_of_distance += calculateDistance(*const_last_iter_x, *const_last_iter_y, *const_iter_x, *const_iter_y);
                (*pl_struct).count+= index_of_cloud - last_index_of_cloud;
              }
            }

            (*pl_struct).is_first_it = false;
            last_index_of_cloud = index_of_cloud;
            const_last_iter_x = const_iter_x;
            const_last_iter_y = const_iter_y;
          }
          index_of_cloud++;
          ++const_iter_x;
          ++const_iter_y;
        }
      }
      //ROS_INFO("\nsize scanner:%i\nsize cloud%i\nnan size:%i",data_in.ranges.size(), index_of_cloud, number_of_NAN.back());

      int index_of_platform = 0;// for finding platforms' index
      for(polygons pol : polygons_data_)//according to point fit line and delete points on the line
      {
        if(!platform_lines_[index_of_platform].cloud.empty())
        {

          platform_cloud cl = platform_lines_[index_of_platform];
          //ROS_INFO("cloud size:%i", cl.cloud.size());
          if(cl.count == 0)
            continue;
          cl.sum_of_distance /= cl.count;
          std::vector<int> inliers, indices;// will hold exception index of line
          Eigen::VectorXf vec;
          pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cl.cloud.makeShared()));
          pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p, cl.sum_of_distance*threshold_coef_);//threshold is sum_of_distance
          ransac.computeModel();//compute the regression of line
          ransac.getInliers(inliers);//get line points' indexes
          ransac.getModel(indices);//not using now
          ransac.getModelCoefficients(vec);// it is 6d vector (last 3 element is direction vector on 3d)
          pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
          pcl::copyPointCloud(cl.cloud, inliers, *final);
          //ROS_INFO("final size:%i\n", final->size());
          //ROS_INFO("inlier size%i indices size:%i", inliers.size(), indices.size());
          double angle_value_of_platforms_edge = atan2(pol.transport[1], pol.transport[0]);
          if(angle_value_of_platforms_edge < 0)
            angle_value_of_platforms_edge += PI;
          double angle_value_of_fitting_line = atan2(vec[4], vec[3]);
          if(angle_value_of_fitting_line < 0)
            angle_value_of_fitting_line += PI;
          double differ_angle = std::abs( (angle_value_of_platforms_edge - angle_value_of_fitting_line)*180/PI);
          if(differ_angle < 10.0)//if fitting line and platform edge angle is too close 
            cl.index_array.clear();//for not entering the for loop below
          //ROS_INFO("%i\tdiffer angle:%f", index_of_platform, differ_angle);
          
          visualizePlatforms(index_of_platform, vec);//show the last points of linestrıp on the calculated line on the rviz
            
          int count=0, inliers_index = 0;
              
          for(int index : cl.index_array)//delete points on the line
          {
            if(inliers[inliers_index] == count)//then it is on the line
            {
              int scan_index = point_to_scan_index_map_[index];
              data_out.ranges[scan_index] = std::numeric_limits<float>::quiet_NaN();
              inliers_index++;
            }
            count++;
          }
          //ROS_INFO("%i\t%i", index_of_platform, count);
        }
        platform_cloud cl;
        platform_lines_[index_of_platform] = cl;//clear inside because of not override
        index_of_platform++;
      }
      
    }
    return true;
  }
  /*count NAN values until coming that index and inizilize the vector that have same size with scan data*/
  void PlatformFilter::indexBaseCountNAN(std::vector<int>& vec, const sensor_msgs::LaserScan& data)
  {
    int count = 0;
    for(int i=0;i < data.ranges.size();i++)
    {
      if( !(data.range_min < data.ranges[i]  && data.range_max > data.ranges[i]) )
        count++;
      vec.push_back(count);
    }
  }

  /*memorize the platforms position*/
  void PlatformFilter::PlatformZoneCallBack(const milvus_msgs::MapFeaturesStamped::ConstPtr& msg)
  {
    if(conf_)
    {
      //if it is a new call or first call
      if(platforms_id_.compare(msg->map_features[0].name.c_str()) != 0 || platforms_id_.compare("") == 0 )
      {
        platform_array_ = msg->map_features;
        platforms_id_ = msg->map_features[0].name;
        platforms_ready_ = CarryPolygons();//creating polygon strings that describe places where scan data is expected to come
        //CarryPolygon function will return bool because of controlling data accuracy
      }
      if(msg->map_features[0].id.compare("") == 0)//if id is not given, all platforms will not be executed
      {
        ROS_INFO("platforms ID did not given");
        platforms_ready_ = false;
      }
    }
  }

  void PlatformFilter::reconfigureCB(laser_filters::PlatformFilterConfig& config, uint32_t level)
  {
    tolerance_ = config.tolerance;
    max_distance_ = config.max_distance;
    skipped_angle_ = config.number_skipped_angle;
    threshold_coef_ = config.threshold_coefficient;
    if(platforms_ready_)
      platforms_ready_ = CarryPolygons();//do it again according to new tolerance
  }

  int PlatformFilter::isOnPlatform(float scan_x, float scan_y)
  {
    std::vector<polygons>::iterator it_beg = polygons_data_.begin();
    std::vector<polygons>::iterator it_polygons = it_beg;//has to be platform_array_ is same size 
    for (milvus_msgs::MapFeature platform : platform_array_)//iteration all platforms
    {
      std::vector<geometry_msgs::Pose2D> points_of_platform = platform.geometries[0].points;

      std::string str_polygon;

      if(CloseEnough(&points_of_platform))//if it is far we do not control
      {

        if(isOnGround(it_polygons->string_of_zone) )
       	  str_polygon = it_polygons->string_of_polygon[0];//control the polygon that is on the zone
     	  else
          str_polygon = it_polygons->string_of_polygon[1];//control the polygon that is on the ground

        if(exactlyPlatform(scan_x, scan_y, str_polygon))
        {
          //ROS_INFO("DELETING...(%f,%f)\n%s", scan_x, scan_y, s_polygon.c_str());
          //ROS_INFO("DELETable...%f", calculateDistance(scan_x, scan_y, laser_x_, laser_y_)); 
          return it_polygons - it_beg;
        }
      }
    it_polygons++;
    }
    return -1;
  }
  
  /*robots position(actually lidar positions) is on the platform or on the ground*/
  bool PlatformFilter::isOnGround(std::string str_polygon)
  {
    boost::geometry::model::d2::point_xy<double> point_2d(laser_x_, laser_y_);
    typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > polygon;
    polygon platform;
    boost::geometry::read_wkt(str_polygon, platform);
    return boost::geometry::disjoint(platform, point_2d);//if it is outside of the platform, it is on the ground
  }

  /*check is it close enough to control that platform*/
  bool PlatformFilter::CloseEnough(std::vector<geometry_msgs::Pose2D> *points_of_platform)//can be develop
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
    boost::geometry::read_wkt(str_polygon, platform);
    return !boost::geometry::disjoint(platform, point_2d);
  }

  /*collect the new positions data*/
  void PlatformFilter::tfUpdate(ros::Time time)
  {
    tf::StampedTransform tf_transform_laser;
    bool tf_not_ready = true;
    int count = 0;
    while(tf_not_ready && ros::ok())
    {
      //ROS_INFO("%f",t.toSec());
      try{
        //ros::Duration(t.toSec()).sleep();
        tf_listener_.lookupTransform(map_frame_, laser_frame_, time , tf_transform_laser);//laser's position according to map        tf_not_ready = false;
        tf_not_ready = false;
      }
      catch (tf::TransformException &ex){
        ROS_ERROR("%s",ex.what()); 
        ros::Duration(0.5).sleep();
        tf_not_ready = true;
        if(++count >= 2)
          time = ros::Time(0);
      }
    }

    laser_x_ = tf_transform_laser.getOrigin().x();//coordinate of laser according to map
    laser_y_ = tf_transform_laser.getOrigin().y();
    laser_z_ = tf_transform_laser.getOrigin().z();
    laser_yaw_ = tf::getYaw(tf_transform_laser.getRotation());

  }

  /*platforms' lines taken and carried to on the zone and ground(named polygon) also initilize the platforms data*/
  bool PlatformFilter::CarryPolygons()
  {
    polygons_data_.clear();

    int index = 0;

    for (milvus_msgs::MapFeature msg_pol : platform_array_ )//iteration all the platforms
    {
      polygons temp_pol;
      double pitch_angle = std::stod(msg_pol.payload);
      if(msg_pol.geometries.size() != 1)
      {
        ROS_WARN("EVERY GEOMETRIES MUST HAVE 1 ELEMENT");
        return false;
      }

      if(!(msg_pol.type.compare("zone") == 0 && msg_pol.subtype.compare("ramp_zone") == 0))
      {
        return false;
      }

      if( pitch_angle > 90.0)
      {
        ROS_WARN("ANGLE VALUES CAN NOT HIGHER THAN 90 degree");
        return false;
      }
      
      std::vector<geometry_msgs::Pose2D> points_of_platform = msg_pol.geometries[0].points;
      if(points_of_platform.size() != 4)
      {
        ROS_WARN("MAP FEATUERS POİNTS SİZE MUST BE EXACTLY 4");
        return false;
      }

      double yaw;
      calculateYaw(&yaw, &points_of_platform);// calculate direction angle(yaw)
      calculateDirection(temp_pol.transport, pitch_angle, yaw);//calculate the amount of displacement and assign to the temp.transport
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
      platform_cloud cl;
      platform_lines_[index++] = cl;//declaration

      ROS_INFO("%ith polygon was carried. polygon on the zone:%s \npolygon on the ground:%s\npolygon zones itself:%s\ndirection_x:%f\tdirection_y:%f\nyaw:%f\ntolerance:%f\n", index
      , temp_pol.string_of_polygon[0].c_str(), temp_pol.string_of_polygon[1].c_str(), temp_pol.string_of_zone.c_str(), temp_pol.transport[0], temp_pol.transport[1], yaw, tolerance_);
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
  void PlatformFilter::calculateYaw(double *yaw, std::vector<geometry_msgs::Pose2D> *vec)//return the degree of yaw
  {
    double middle_x, middle_y;
    for(geometry_msgs::Pose2D point : *vec)//find middle of the platform
    {
      middle_x += point.x;
      middle_y += point.y;
    }
    middle_x /= vec->size();
    middle_y /= vec->size();
    double mid_beg_plat_x = ( vec->begin()->x + (vec->begin()+1)->x ) / 2;//middle of the beginning of platform
    double mid_beg_plat_y = ( vec->begin()->y + (vec->begin()+1)->y ) / 2;
    //ROS_INFO("org_plat_x:%forg_plat_y:%f beg_plat_x:%f\tbeg_plat_y:%f",middle_x, middle_y, mid_beg_plat_x, mid_beg_plat_y);
    if(middle_x - mid_beg_plat_x == 0.0)
    {
      if(middle_y - mid_beg_plat_y < 0.0)
        *yaw = 270;
      else if(middle_y - mid_beg_plat_y > 0.0)
        *yaw = 90;
    }
    else
      if(middle_y - mid_beg_plat_y == 0.0 && middle_x - mid_beg_plat_x < 0.0)
        *yaw = 180;
      else{
        double temp = std::atan((middle_y - mid_beg_plat_y) / (middle_x - mid_beg_plat_x))*180/PI;

        if(middle_y > mid_beg_plat_y)
          *yaw = std::abs(temp); 
        else
          *yaw = -std::abs(temp); 
      }
  }

  void PlatformFilter::visualizePlatforms()//at the same Hz as update
  {
    int index = 0;
    std::vector<polygons>::iterator it_polygons = polygons_data_.begin();
    for (milvus_msgs::MapFeature platform : platform_array_)//iteration all platforms
    {
      visualization_msgs::Marker marking_plat, marking_line;
      marking_plat.header.frame_id = map_frame_;
      marking_plat.header.stamp = ros::Time();
      marking_plat.ns = "platform";
      marking_plat.id = index++;
      marking_plat.type = visualization_msgs::Marker::CUBE;
      marking_plat.action = visualization_msgs::Marker::ADD;
      marking_plat.pose.position.x = (platform.geometries[0].points[0].x + platform.geometries[0].points[1].x + platform.geometries[0].points[2].x + platform.geometries[0].points[3].x) / 4;
      marking_plat.pose.position.y = (platform.geometries[0].points[0].y + platform.geometries[0].points[1].y + platform.geometries[0].points[2].y + platform.geometries[0].points[3].y) / 4;
      marking_plat.pose.position.z = 0;
      tf2::Quaternion q;
      double yaw;
      calculateYaw(&yaw, &platform.geometries[0].points);
      yaw = (yaw-90)*PI/180;
      q.setRPY(0, 0, yaw);
      q = q.normalize();
      marking_plat.pose.orientation.x = q.getX();
      marking_plat.pose.orientation.y = q.getY();
      marking_plat.pose.orientation.z = q.getZ();
      marking_plat.pose.orientation.w = q.getW();
      marking_plat.scale.x = calculateDistance(platform.geometries[0].points[0].x, platform.geometries[0].points[0].y, platform.geometries[0].points[1].x, platform.geometries[0].points[1].y);//weight of platform
      marking_plat.scale.y = calculateDistance(platform.geometries[0].points[0].x, platform.geometries[0].points[0].y, platform.geometries[0].points[3].x, platform.geometries[0].points[3].y);//length of platform
      marking_plat.scale.z = 0.001;
      marking_plat.color.a = 0.3; // Don't forget to set the alpha!
      marking_plat.color.r = 0.8*index;
      marking_plat.color.g = 0.1*index;
      marking_plat.color.b = 0.0;
      marker_pub_.publish(marking_plat);

      /*create line of expected points*/
      marking_line.header.frame_id = map_frame_;
      marking_line.header.stamp = ros::Time();
      marking_line.ns = "line";
      marking_line.id = index++;
      marking_line.type = visualization_msgs::Marker::LINE_STRIP;
      marking_line.action = visualization_msgs::Marker::ADD;
      geometry_msgs::Point point_1, point_2;
      if(isOnGround(it_polygons->string_of_zone))
      {
        point_1.x = platform.geometries[0].points[0].x + it_polygons->transport[0];
        point_1.y = platform.geometries[0].points[0].y + it_polygons->transport[1];
        point_2.x = platform.geometries[0].points[1].x + it_polygons->transport[0];
        point_2.y = platform.geometries[0].points[1].y + it_polygons->transport[1];
      }
      else
      {
        point_1.x = platform.geometries[0].points[0].x - it_polygons->transport[0];
        point_1.y = platform.geometries[0].points[0].y - it_polygons->transport[1];
        point_2.x = platform.geometries[0].points[1].x - it_polygons->transport[0];
        point_2.y = platform.geometries[0].points[1].y - it_polygons->transport[1];
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
    fitting_line[0] = (vec[4]/vec[3]);//delta y / delta x
    fitting_line[1] = vec[1] - fitting_line[0] * vec[0];
    //ROS_INFO("platform:%i\nfit m: %f, fit n: %f", index_of_platform, fitting_line[0], fitting_line[1]);
    std::vector<geometry_msgs::Pose2D> points_of_platform = platform_array_[index_of_platform].geometries[0].points;
    double line_1[2];//mx + n(m, n)
    calculateLine(line_1, points_of_platform[0].x, points_of_platform[0].y, points_of_platform[3].x, points_of_platform[3].y);
    double line_2[2];//mx + n(m, n)
    calculateLine(line_2, points_of_platform[1].x, points_of_platform[1].y, points_of_platform[2].x, points_of_platform[2].y);
    //ROS_INFO("m_1:%f n_1%f  m_2:%f n_2%f", line_1[0] , line_1[1], line_2[0], line_2[1]);
    marking_line.header.frame_id = map_frame_;
    marking_line.header.stamp = ros::Time();
    marking_line.ns = "line";
    marking_line.id = 55 + index_of_platform;
    marking_line.type = visualization_msgs::Marker::LINE_STRIP;
    marking_line.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point point_msg;
    calculateCommonPoint(point_msg, fitting_line[0], fitting_line[1], line_1[0], line_1[1]);
    //ROS_INFO("x: %f, y: %f", point_msg.x, point_msg.y);
    marking_line.points.push_back(point_msg);
    calculateCommonPoint(point_msg, fitting_line[0], fitting_line[1], line_2[0], line_2[1]);
    //ROS_INFO("x: %f, y: %f", point_msg.x, point_msg.y);
    marking_line.points.push_back(point_msg);
    
    marking_line.pose.orientation.w = 1.0;
    marking_line.scale.x = 0.1*threshold_coef_;
    marking_line.scale.y = 0.1*threshold_coef_;
    marking_line.scale.z = 0;
    marking_line.color.a = 1;
    marking_line.color.r = 1;
    marking_line.color.g = 1;
    marking_line.color.b = 0.0;
    marker_pub_.publish(marking_line);
  }
  
  /*calculate the line according to given points and return slope value [0](m) and constant value [1](n)*/
  void PlatformFilter::calculateLine(double* line, double beg_x, double beg_y, double end_x, double end_y)
  {
    if(end_x - beg_x == 0)
    {
        line[0] = 1000000000000.0;
    }
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
    else//parallel each other
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
