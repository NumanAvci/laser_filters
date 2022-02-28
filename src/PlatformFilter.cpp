#include <laser_filters/PlatformFilter.h>
#include <pluginlib/class_list_macros.h>

//PLUGINLIB_REGISTER_CLASS(simple_layers_PlatformFilter, delete_platform_namespace::PlatformFilter, filters::FilterBase<sensor_msgs::LaserScan>)

namespace delete_platform_namespace{

  bool PlatformFilter::configure(){
    ros::NodeHandle nh;
    platfrom_sub_ = nh.subscribe("/platforms", 1000, &PlatformFilter::PlatformCallBack, this);
    platforms_ready_ = false;
    ros::spinOnce();
  }

  PlatformFilter::PlatformFilter(){//I may need to carry that transformation to the configure method (so I did:) but it was also wrong)
   
  }

  /*marking the ranges of intersection of scan data to the platform*/
  bool PlatformFilter::update(const sensor_msgs::LaserScan& data_in, sensor_msgs::LaserScan& data_out)
  {
    tf_update();//taking tf data
    data_out = data_in;
    if(platforms_ready_ )//is published the platforms by the user
    {
      bool in_range = false;
      float beginnning_of_range, end_of_range;
      std::vector<float>::iterator beginnning_it, end_it;
      float angle = data_in.angle_min;
      for(std::vector<float>::iterator it = data_out.ranges.begin();&(*it) != &(*data_out.ranges.end()); it++)
      {
        if(*it != std::numeric_limits<float>::quiet_NaN() && isIntersection(angle, *it))//questions is really coming from platfrom
        {
          if(in_range)//possible end of the intersection
          {
            end_it = it;
          }
          else
          {
            beginnning_it = it;
            platform_angle_range_.push(beginnning_it);
            in_range = true;
          }
        }
        else
        {
          if(in_range)//if intersection was started it is over anymore
          {
            //push the end of the range
            platform_angle_range_.push(end_it);

            //pop the range
            std::vector<float>::iterator end = platform_angle_range_.back();
            platform_angle_range_.pop();
            std::vector<float>::iterator begin = platform_angle_range_.back();
            platform_angle_range_.pop();
            /*delete the intersection range*/
            for(std::vector<float>::iterator index = begin; &(*index) != &(*end); index++)
            {
            *index = std::numeric_limits<float>::quiet_NaN();
            }
          in_range = false;
          }
        }
        angle+=data_in.angle_increment;//find another way
        /*angle = end - it - 1; -> give the angle
        angle > data_out.range_max - data_out.range_min*/
        if(angle > data_out.range_max)
          angle = data_out.range_max;
      }
    }

    return true;
  }

  /*memorize the platforms position*/
  void PlatformFilter::PlatformCallBack(const laser_filters::Polygon_array::ConstPtr& msg)
  {
    platform_array_ = msg->points_to_points;
    pitches_ = msg->angles;
    platform_yaws_ = msg->directions;
    carry_lines();//creating polygon strings that describe places where scan data can come from
    platforms_ready_ = true;
  }

  /*control the intersection of reading scanning data to the one of the platforms*/
  bool PlatformFilter::isIntersection(float angle, double range)
  {
  
    double angle_of_alfa = angle + laser_yaw_;
    double scanning_point_y = laser_y_ + range * std::sin(angle_of_alfa);//according to map
    double scanning_point_x = laser_x_ + range * std::cos(angle_of_alfa);
    /*now we know that reading point, laser point, platforms' points*/
    
    line_segment scanning_line = calculateLine(laser_x_, laser_y_, scanning_point_x, scanning_point_y);

    std::vector<geometry_msgs::Polygon>::iterator it_platform;
    std::vector<polygons>::iterator it_polygons = polygons_data_.begin();
    std::vector<double>::iterator yaws_it = platform_yaws_.begin();

    for (it_platform = platform_array_.begin(); &(*it_platform) != &(*platform_array_.end());
     ++it_platform, it_polygons++, yaws_it++)
    {
      std::vector<geometry_msgs::Point32> points_of_platform = it_platform->points;
      std::vector<geometry_msgs::Point32>::iterator beginnning = points_of_platform.begin();
      std::vector<geometry_msgs::Point32>::iterator end = points_of_platform.end();
      std::vector<geometry_msgs::Point32>::iterator index = beginnning;
      std::string *s_polygons;
      s_polygons = it_polygons->string_of_polygon;
      std::string s_polygon;
      double change = abs(angle_of_alfa - *yaws_it);
      if( (change > 90 && change < 270))
        s_polygon = s_polygons[0];
      else
        s_polygon = s_polygons[1];

      if(exactlyPlatform(&scanning_line, s_polygon));
        return farEnoughAway(&scanning_line, it_polygons->transport);
    }

    return false;
  }

  bool PlatformFilter::farEnoughAway(line_segment* scan, double *transport)
  {
  	double change_x, change_y;
  	change_x = scan->range_x2 - scan->range_x2;
  	change_y = scan->range_y2 - scan->range_y2; 
  	return transport[1] < change_y && transport[0] < change_x; 
  }

  bool PlatformFilter::isOnPlatform()
  {
    return false;
  }

  line_segment PlatformFilter::calculateLine(double beginning_x, double beginning_y, double end_x, double end_y)
  {
    line_segment line;
    line.m = (end_y - beginning_y) / (end_x - beginning_x);
    line.b = end_y - line.m * end_x;
    line.range_x1 = beginning_x;
    line.range_x2 = end_x;
    line.range_y1 = beginning_y;
    line.range_y2 = end_y;
    return line;
  }

  /*control that scan is data coming from expected points (assume that lines are intersection each other)*/
  bool PlatformFilter::exactlyPlatform(line_segment* scan, std::string str_polygon)
  {
    int constant = laser_z_;//depends on robots' height
    float tolerance = 0.1;

    typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > polygon;
    polygon platform;
    typedef boost::geometry::model::d2::point_xy<double> P; 
    boost::geometry::model::linestring<P> scan_line;
    std::string scan_string;

    scan_string = "linestring(" + std::to_string(scan->range_x1) + " " + std::to_string(scan->range_y1) 
                           + "," + std::to_string(scan->range_x2) + " " + std::to_string(scan->range_y2) + ")";
    boost::geometry::read_wkt(scan_string, scan_line);
    boost::geometry::read_wkt(str_polygon, platform);
    return boost::geometry::intersects(scan_line, platform);
  }

  void PlatformFilter::tf_update()
  {
    tf::StampedTransform tf_transform_laser;
    try{
      tf_listener_.lookupTransform("/map", "/base_scan", ros::Time(0), tf_transform_laser);//laser's position according to map
      }
      catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what()); 
      }
    laser_x_ = tf_transform_laser.getOrigin().x();//coordinate of laser according to map
    laser_y_ = tf_transform_laser.getOrigin().y();
    laser_z_ = tf_transform_laser.getOrigin().z();
  
    laser_yaw_ = tf_transform_laser.getRotation().getAngle();
  }

  /*control that the given platform and scan are intersect*/
  bool PlatformFilter::boost_intersection(std::vector<geometry_msgs::Point32> *points_of_platform, double laser_end_x, double laser_end_y)
  {
    typedef boost::geometry::model::d2::point_xy<double> P; 
    boost::geometry::model::linestring<P> line_p, line_s;
    std::string p_string, s_string;//platfrom string , scanner string

    std::vector<geometry_msgs::Point32>::iterator beginnning = points_of_platform->begin();
    std::vector<geometry_msgs::Point32>::iterator end = points_of_platform->end();
    std::vector<geometry_msgs::Point32>::iterator index = beginnning;
    p_string = "linestring(";
    while(&(*index) != &(*end)){
      p_string += std::to_string((*index).x) + " " + std::to_string((*index).y);
      index++;
      if(&(*index) != &(*end))
        p_string += ",";
    }
    p_string += ")";

    s_string = "linestring(" + std::to_string(laser_x_) + " " + std::to_string(laser_y_) 
                           + "," + std::to_string(laser_end_x) + " " + std::to_string(laser_end_y) + ")";
    boost::geometry::read_wkt(p_string, line_p);
    boost::geometry::read_wkt(s_string, line_s);
    return boost::geometry::intersects(line_s, line_p);
  }

  /*taken platforms' lines carry */
  void PlatformFilter::carry_lines()
  {
    std::string platform_string[2];
    float tolerance = 0.1;
    std::vector<double>::iterator pitch_angles = pitches_.begin();
    std::vector<double>::iterator yaws = platform_yaws_.begin();
    for (std::vector<geometry_msgs::Polygon>::iterator i = platform_array_.begin(); &(*i) != &(*platform_array_.end()); ++i, 
        pitch_angles++, yaws++)
    {
      std::vector<geometry_msgs::Point32> points_of_platform = i->points;
      std::vector<geometry_msgs::Point32>::iterator beginnning = points_of_platform.begin();
      std::vector<geometry_msgs::Point32>::iterator end = points_of_platform.end();
      std::vector<geometry_msgs::Point32>::iterator index = beginnning;
      platform_string[0] = "POLYGON(";
      platform_string[1] = "POLYGON(";
      double a[2];
      while(&(*index) != &(*end))
      {
        if(&(*(index+1)) != &(*end) )
          calculate_direction(a, *pitch_angles, *yaws);
        platform_string[1] += std::to_string((*index).x + a[0] + tolerance) + " " + std::to_string((*index).y + a[1] + tolerance);
        platform_string[0] += std::to_string((*index).x - a[0] + tolerance) + " " + std::to_string((*index).y - a[1] + tolerance);;
        index++;
        platform_string[1] += ",";
        platform_string[0] += ",";
      }
      index--;
      //reverse order
      while(&(*index) != &(*beginnning))
      {
        calculate_direction(a, *pitch_angles, *yaws);
        platform_string[1] += std::to_string((*index).x + a[0] - tolerance) + " " + std::to_string((*index).y + a[1] - tolerance);
        platform_string[0] += std::to_string((*index).x - a[0] - tolerance) + " " + std::to_string((*index).y - a[1] - tolerance);
        index--;
        if(&(*index) != &(*beginnning))
        {
          platform_string[1] += ",";
          platform_string[0] += ",";
        }
        else
        {
        	platform_string[1] += std::to_string((*index).x + a[0] - tolerance) + " " + std::to_string((*index).y + a[1]- tolerance);
        	platform_string[0] += std::to_string((*index).x - a[0] - tolerance) + " " + std::to_string((*index).y - a[1]- tolerance);
        	break;
        }
      }
      platform_string[1] += "," + std::to_string((*beginnning).x + a[0] + tolerance) + " " + std::to_string((*beginnning).y + a[1] + tolerance) + ")";
      platform_string[0] += "," + std::to_string((*beginnning).x - a[0] + tolerance) + " " + std::to_string((*beginnning).y - a[1] + tolerance) + ")";

      polygons temp;
      temp.string_of_polygon = platform_string;
      temp.transport = a;
      polygons_data_.push_back(temp);
    }
  }
/*calculate the direction to the line according to pitch and assign to a variable*/
  void PlatformFilter::calculate_direction(double* a, double pitch, double yaw)
  {
    a[0] = yaw * laser_z_ * (1.0/std::tan(pitch));//x direction
    a[1] = (yaw > 0) ? (yaw * laser_z_ * (1.0/std::tan(pitch))) : (yaw * laser_z_ * (1.0/std::tan(pitch)) * -1);//y direction
  }
}
