#include <laser_filters/platform_filter.h>
#include <pluginlib/class_list_macros.h>

//PLUGINLIB_REGISTER_CLASS(simple_layers_PlatformFilter, delete_platform_namespace::PlatformFilter, filters::FilterBase<sensor_msgs::LaserScan>)

namespace laser_filters{

  bool PlatformFilter::configure(){
    ros::NodeHandle nh;
    platfrom_sub_ = nh.subscribe("/platform_zone", 1000, &PlatformFilter::PlatformZoneCallBack, this);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("/vis", 1000);
    platforms_ready_ = false;
    is_on_ground_ = true;
    platforms_id_ = "";
    tfUpdate();
    ros::spinOnce();
    ROS_INFO("Configuration completed.");
  }

  PlatformFilter::PlatformFilter(){//I may need to carry that transformation to the configure method (so I did:) but it was also wrong)
   
  }

  /*marking the ranges of intersection of scan data to the platform*/
  bool PlatformFilter::update(const sensor_msgs::LaserScan& data_in, sensor_msgs::LaserScan& data_out)
  {
    tfUpdate();//taking tf data
    data_out = data_in;
    if(platforms_ready_ )//is published the platforms by the user
    {
      visualizePlatforms();
      float angle = data_in.angle_min;
      for (uint16_t i=0; i < data_out.ranges.size(); i++)//iteration all scan data
      {
        if(data_out.ranges[i] >= data_out.range_min && data_out.ranges[i] <= 2*PI)
        {
          const std::string& str_plat = isIntersection(angle, data_out.ranges[i]);
          if(&str_plat != NULL)//questions is really coming from platform
          {
            scan_data scan;
            scan.index = i;
            scan.range = data_out.ranges[i];
            scan.angle = angle;
            platform_lines_[str_plat].push_back(scan);
          }
        }
         angle+=data_in.angle_increment;//find another way
        /*angle = end - it - 1; -> give the angle
        angle > data_out.range_max - data_out.range_min*/
        if(angle > 2*PI)
          angle = 2*PI;
      }

      for(polygons pol : polygons_data_)
       {
         if(!platform_lines_[pol.string_of_zone].empty())
         {
            std::vector<double> x;
            std::vector<double> y;
            for(scan_data scan : platform_lines_[pol.string_of_zone])
            {
              double angle_of_alfa = scan.angle + laser_yaw_;
              double scanning_point_y = laser_y_ + (scan.range * std::sin(angle_of_alfa));//according to map
              double scanning_point_x = laser_x_ + (scan.range * std::cos(angle_of_alfa));
              x.push_back(scanning_point_x);
              y.push_back(scanning_point_y);
            }
            //auto [n, m] = boost::math::statistics::simple_ordinary_least_squares(x, y);//mx + n
            //oluşan doğrunun aralığı son ve ilk noktanın bu doğruya olan dik uzaklığı bulunarak ayarlanabilir.
            /*for(scan_data scan : platform_lines_[pol.string_of_zone])
            {
              std::string zone = "POLYGON((";
              //zone += 
              data_out.ranges[scan.index] = std::numeric_limits<float>::quiet_NaN();
            }*/
         }
       }
    }

    return true;
  }

  /*memorize the platforms position*/
  void PlatformFilter::PlatformZoneCallBack(const laser_filters::polygon_array::ConstPtr& msg)
  {
    /*if it is a new call or first call*/
    if(platforms_id_.compare(msg->id.c_str()) != 0 || platforms_id_.compare("") == 0 ){
      platform_array_ = msg->points_to_points;
      pitches_ = msg->angles;
      platforms_id_ = msg->id;
      polygons_data_.clear();
      CarryPolygons();//creating polygon strings that describe places where scan data can come from
      //CarryPolygon function will change the value of platforms_ready_ because of controlling data accurancy
    }
    if(msg->id.compare("") == 0)//if id is not given, all platforms are not executed
    {
      platforms_ready_ = false;
    }
  }

  /*control the intersection of reading scanning data to the any of the platforms*/
  const std::string& PlatformFilter::isIntersection(float angle, double range)
  {
    double angle_of_alfa = angle + ((laser_yaw_ < 0) ? (laser_yaw_+2*PI) : laser_yaw_);
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

        if(exactlyPlatform(scanning_point_x, scanning_point_y, s_polygon)){
          //ROS_INFO("DELETING...(%f,%f)", scanning_point_x, scanning_point_y);
          return (*it_polygons).string_of_zone;
        }
      }
    it_polygons++;
    }
    std::string *null_string = NULL;
    return *null_string;
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
    double distance = calculateDistance(points_of_platform->front().x, points_of_platform->front().y,
                                        (&points_of_platform->front() + 1)->x, (&points_of_platform->front() + 1)->y); 
    return distance <= 10; 
  }

  /*control that is scan data coming from expected points that named str_polygon*/
  bool PlatformFilter::exactlyPlatform(double scan_x, double scan_y, std::string str_polygon)
  {
    int constant = laser_z_;//depends on robots' height
    float tolerance = 0.1;

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
    while(tf_not_ready){
      try{
        tf_listener_.lookupTransform("/map", "/base_scan", ros::Time(0), tf_transform_laser);//laser's position according to map
        tf_not_ready = false;
      }
      catch (tf::TransformException &ex) {
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
  void PlatformFilter::CarryPolygons()
  {
    float tolerance = 0.1;
    std::vector<double>::iterator pitch_angles = pitches_.begin();
    
    for (geometry_msgs::Polygon i : platform_array_ )//iteration all the platforms
    {
      polygons temp;
      double yaw;
      std::vector<geometry_msgs::Point32> points_of_platform = i.points;
      if(points_of_platform.size() != 4)
      {
        platforms_ready_ = false;
        return;
      }
      geometry_msgs::Point32 *beginnning = &points_of_platform.front();
      geometry_msgs::Point32 *end = (&points_of_platform.back()) + 1;
      geometry_msgs::Point32 *index = beginnning;
      calculateYaw(&yaw, &points_of_platform);// calculate direction angle(yaw)
      calculateDirection(temp.transport, *pitch_angles, yaw);//calculate the amount of displacement and assign to the temp.transport
      temp.string_of_polygon[0] = "POLYGON((";
      temp.string_of_polygon[1] = "POLYGON((";
      temp.string_of_zone = "POLYGON((";
      while(index != end)/*first iterate beginnning to the end*/
      {
        if(index-beginnning < 2)//first two point is our displacment line
        {
          temp.string_of_polygon[0] += std::to_string((*index).x + temp.transport[0] + (std::cos(PI*yaw/180) * tolerance)) + " "
                             + std::to_string((*index).y + temp.transport[1] + (std::sin(PI*yaw/180) * tolerance));
          temp.string_of_polygon[1] += std::to_string((*index).x - temp.transport[0] + (std::cos(PI*yaw/180) * tolerance)) + " "
                             + std::to_string((*index).y - temp.transport[1] + (std::sin(PI*yaw/180) * tolerance));
          temp.string_of_polygon[1] += ",";
          temp.string_of_polygon[0] += ",";
        }
        temp.string_of_zone += std::to_string((*index).x) + " " + std::to_string((*index).y);
        temp.string_of_zone += ",";
        index++;      
      }
      temp.string_of_zone += std::to_string((*beginnning).x) + " " + std::to_string((*beginnning).y) + "))";
      index = beginnning + 2;//yes, it has extra one iteration

      //reverse order
      while(index != beginnning-1)/*it is not necessary that iteration of end to beginning, iterate just first two point*/
      {
      	if(index-beginnning < 2)
      	{
          temp.string_of_polygon[0] += std::to_string((*index).x + temp.transport[0] - (std::cos(PI*yaw/180) * tolerance)) + " "
                             + std::to_string((*index).y + temp.transport[1] - (std::sin(PI*yaw/180) * tolerance));
          temp.string_of_polygon[1] += std::to_string((*index).x - temp.transport[0] - (std::cos(PI*yaw/180) * tolerance)) + " "
                             + std::to_string((*index).y - temp.transport[1] - (std::sin(PI*yaw/180) * tolerance));
          temp.string_of_polygon[1] += ",";
          temp.string_of_polygon[0] += ",";
        }
        index--;
      }
      /*for creating closed polygon*/
      temp.string_of_polygon[0] += std::to_string((*beginnning).x + temp.transport[0] + (std::cos(PI*yaw/180) * tolerance)) + " "
                             + std::to_string((*beginnning).y + temp.transport[1] + (std::sin(PI*yaw/180) * tolerance)) + "))";
      temp.string_of_polygon[1] += std::to_string((*beginnning).x - temp.transport[0] + (std::cos(PI*yaw/180) * tolerance)) + " "
                             + std::to_string((*beginnning).y - temp.transport[1] + (std::sin(PI*yaw/180) * tolerance)) + "))";
      
      polygons_data_.push_back(temp);
      pitch_angles++;

    ROS_INFO("Polygons carried. polygon on the zone:%s \npolygon on the ground:%s\npolygon zones itself:%s\ndirection_x:%f\tdirection_y:%f",
      temp.string_of_polygon[0].c_str(), temp.string_of_polygon[1].c_str(), temp.string_of_zone.c_str(), temp.transport[0], temp.transport[1]);
    }
    platforms_ready_ = true;
  }
  /*calculate the displacement direction for the beginning line according to pitch and yaw ,and assign to variable*/
  void PlatformFilter::calculateDirection(double *direction, double pitch, double yaw)
  {
    direction[1] = std::sin(PI*yaw/180) * laser_z_ * (1.0/std::tan(PI*pitch/180));//y direction
    direction[0] = std::cos(PI*yaw/180) * laser_z_ * (1.0/std::tan(PI*pitch/180));
  }
  /*calculate middle of the polygon and set the yaw as a vector that is through beginning line's middle to that point*/
  void PlatformFilter::calculateYaw(double *yaw, std::vector<geometry_msgs::Point32> *vec)
  {
    double middle_x, middle_y;
    for(geometry_msgs::Point32 point : *vec)
    {
      middle_x += point.x;
      middle_y += point.y;
    }
    middle_x /= vec->size();
    middle_y /= vec->size();
    double mid_beg_plat_x = ( vec->begin()->x + (vec->begin()+1)->x ) / 2;
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
      marking_plat.color.a = 0.5; // Don't forget to set the alpha!
      marking_plat.color.r = 100.0*index;
      marking_plat.color.g = 1.0*index/2;
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
      marking_line.scale.x = 0.2;
      marking_line.scale.y = 0.2;
      marking_line.scale.z = 0;
      marking_line.color.a = 0.7;
      marking_line.color.r = 0.0;
      marking_line.color.b = 1.0;
      marker_pub_.publish(marking_line);
      it_polygons++;
    }
  }

  double PlatformFilter::calculateDistance(double beg_x, double beg_y, double end_x, double end_y)
  {
    double change_x = beg_x - end_x;
    double change_y = beg_y - end_y;
    return std::sqrt((change_y*change_y) + (change_x*change_x)); 
  }
}
