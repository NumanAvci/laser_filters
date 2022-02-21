#include <laser_filters/PlatformFilter.h>
#include <pluginlib/class_list_macros.h>

//PLUGINLIB_EXPORT_CLASS(delete_platform_namespace::PlatformFilter, filters::FilterBase<sensor_msgs::LaserScan>)
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
  	tf_update();
    data_out = data_in;
    if(platforms_ready_ )//is published the platforms by the user
    {
      bool in_range = false;
      float beginnning_of_range, end_of_range;
      std::vector<float>::iterator beginnning_it, end_it;
      float angle = data_in.angle_min;
      for(std::vector<float>::iterator it = data_out.ranges.begin();&(*it) != &(*data_out.ranges.end()); it++)
      {
        if(*it != std::numeric_limits<float>::quiet_NaN() && isIntersection(angle, *it))//controls the intersection of scan and
                                              //platfroms' beginnning and also questions 
                                             //is really coming from platfrom
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
    platforms_ready_ = true;
  }


  /*control the intersection of reading scanning data to the one of the platforms*/
  bool PlatformFilter::isIntersection(float angle, double range)
  {
  	
    double angle_of_alfa = angle + laser_yaw_;//135 is robot specific angle
    double scanning_point_y = laser_y_ + range * std::sin(angle_of_alfa);//according to map
    double scanning_point_x = laser_x_ + range * std::cos(angle_of_alfa);
    /*now we know that reading point, laser point, platforms' points*/
    
    line_segment scanning_line = calculateLine(laser_x_, laser_y_, scanning_point_x, scanning_point_y);
    for (std::vector<geometry_msgs::Polygon>::iterator i = platform_array_.begin(); &(*i) != &(*platform_array_.end()); ++i)
    {
      std::vector<geometry_msgs::Point32> points_of_platform = i->points;
      std::vector<geometry_msgs::Point32>::iterator beginnning = points_of_platform.begin();
      std::vector<geometry_msgs::Point32>::iterator end = points_of_platform.end();
      line_segment platforms_line = calculateLine((*beginnning).x, (*beginnning).y, (*end).x, (*end).y);

      double difference_slope = platforms_line.m - scanning_line.m;//m1-m2=m_new
      double difference_bValue = platforms_line.b - scanning_line.b;//b1-b2=b_new
      if(difference_slope <= 0.05 && difference_slope >= -0.05)//platform and scanning vector are parallel or coincident
        continue;

      double intersection_point_x = (0-difference_bValue)/difference_slope;//-b_new/m_new
      double intersection_point_y = platforms_line.m * intersection_point_x + platforms_line.b;//m1x+b1

      /*controling range of intersection_point*/
      if(intersection_point_x > platforms_line.range_x1 && intersection_point_x < platforms_line.range_x2
        && intersection_point_x > scanning_line.range_x1 && intersection_point_x < scanning_line.range_x2)
        if(intersection_point_y > platforms_line.range_y1 && intersection_point_y < platforms_line.range_y2
        && intersection_point_y > scanning_line.range_y1 && intersection_point_y < scanning_line.range_y2)
          return exactlyPlatfrom(&scanning_line, &platforms_line);
    }

    return false;
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

  /*control is scan data coming from expected points (assume that lines are intersection each other)*/
  bool PlatformFilter::exactlyPlatfrom(line_segment* scan, line_segment* platform)
  {
    int constant = laser_z_;//depends on robots' height
    float tolarance = 0.1;
    platform->b += constant;
    double old_range_x1 = platform->range_x1;
    double old_range_x2 = platform->range_x2;
    double old_range_y1 = platform->range_y1;
    double old_range_y2 = platform->range_y2;

    /*add or substract according to where comes the laser data, (behind or front)*/
    platform->range_x1 = old_range_x1 + std::cos(platform->m/180)*constant;
    platform->range_x2 = old_range_x2 + std::cos(platform->m/180)*constant;
    platform->range_y1 = old_range_y1 + std::sin(platform->m/180)*constant;
    platform->range_y2 = old_range_y2 + std::sin(platform->m/180)*constant;
    if( (platform->m * scan->range_x2 + platform->b + constant + tolarance) <= scan->range_y2 &&
      (platform->m * scan->range_x2 + platform->b + constant - tolarance) >= scan->range_y2)
      if(scan->range_x2 > platform->range_x1 && scan->range_x2 < platform->range_x2)
        if(scan->range_y2 > platform->range_y1 && scan->range_y2 < platform->range_y2)
          return true;

    platform->range_x1 = old_range_x1 - std::cos(platform->m/180)*constant;
    platform->range_x2 = old_range_x2 - std::cos(platform->m/180)*constant;
    platform->range_y1 = old_range_y1 - std::sin(platform->m/180)*constant;
    platform->range_y2 = old_range_y2 - std::sin(platform->m/180)*constant;
    if( (platform->m * scan->range_x2 + platform->b + constant + tolarance) <= scan->range_y2 &&
      (platform->m * scan->range_x2 + platform->b + constant - tolarance) >= scan->range_y2)
      if(scan->range_x2 > platform->range_x1 && scan->range_x2 < platform->range_x2)
        if(scan->range_y2 > platform->range_y1 && scan->range_y2 < platform->range_y2)
          return true;

    return false;
  }

  void PlatformFilter::tf_update(){
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
}
