#include <laser_filters/PlatformFilter.h>
#include <pluginlib/class_list_macros.h>

//PLUGINLIB_REGISTER_CLASS(simple_layers_PlatformFilter, delete_platform_namespace::PlatformFilter, filters::FilterBase<sensor_msgs::LaserScan>)

namespace delete_platform_namespace{

  bool PlatformFilter::configure(){
    ros::NodeHandle nh;
    platfrom_sub_ = nh.subscribe("/platform_zone", 1000, &PlatformFilter::PlatformZoneCallBack, this);
    platforms_ready_ = false;
    is_on_ground_ = true;
    platforms_id_ = "";
    tf_update();
    ros::spinOnce();
    ROS_INFO("Configuration completed.");
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
      float angle = data_in.angle_min;
      for(float *it = &data_out.ranges.front();it != (&data_out.ranges.back())+1; it++)//iteration all scan data
      {
        if(*it >= data_out.range_min && *it <= data_out.range_max && isIntersection(angle, *it))//questions is really coming from platfrom
        {
        	*it = std::numeric_limits<float>::quiet_NaN();
          
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
  void PlatformFilter::PlatformZoneCallBack(const laser_filters::Polygon_array::ConstPtr& msg)
  {
  	/*if it is a new call or first call*/
    if(platforms_id_.compare(msg->id.c_str()) != 0 || platforms_id_.compare("") == 0 ){
    	platform_array_ = msg->points_to_points;
      pitches_ = msg->angles;
  	  platforms_id_ = msg->id;
      CarryPolygons();//creating polygon strings that describe places where scan data can come from
      //CarryPolygon function will change the value of platforms_ready_ because of controlling data accurancy
    }
    if(msg->id.compare("") == 0)//if id is not given, all platforms are not executed
    {
    	platforms_ready_ = false;
    }
  }

  /*control the intersection of reading scanning data to the any of the platforms*/
  bool PlatformFilter::isIntersection(float angle, double range)
  {
    double angle_of_alfa = angle + (laser_yaw_);
    double scanning_point_y = laser_y_ + range * std::sin(angle_of_alfa);//according to map
    double scanning_point_x = laser_x_ + range * std::cos(angle_of_alfa);
    /*now we know that reading point, laser point, platforms' points*/
    
    line_segment scanning_line = calculateLine(laser_x_, laser_y_, scanning_point_x, scanning_point_y);
    geometry_msgs::Polygon *ptr_platform, *ptr_platform_end;
    ptr_platform_end = &platform_array_.back()+1;//end of vector
    std::vector<polygons>::iterator it_polygons = polygons_data_.begin();
    for (ptr_platform = &platform_array_.front(); ptr_platform != ptr_platform_end;
     ++ptr_platform, it_polygons++)//iteration all platforms
    {
      std::vector<geometry_msgs::Point32> points_of_platform = ptr_platform->points;

      std::string s_polygon;

      if(CloseEnough(&scanning_line, &points_of_platform)){//if it is far we do not control
        //isOnGround((*it_polygons).string_of_zone)
        if(isOnGround((*it_polygons).string_of_zone) )
       	  s_polygon = (*it_polygons).string_of_polygon[0];//control the polygon that is on the zone
     	  else
          s_polygon = (*it_polygons).string_of_polygon[1];//control the polygon that is on the ground      
        //ROS_INFO("isOnGround:%i", isOnGround((*it_polygons).string_of_zone));
        if(exactlyPlatform(&scanning_line, s_polygon)){
          ROS_INFO("DELETING point of:(%f,%f)", scanning_point_x, scanning_point_y);
          return true;
        }
      }
    }

    return false;
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
  bool PlatformFilter::CloseEnough(line_segment *scan, std::vector<geometry_msgs::Point32> *points_of_platform)//can be develop
  {
  	double change_x, change_y, mid_beg_plat_x, mid_beg_plat_y;
  	mid_beg_plat_x = points_of_platform->front().x  + (&points_of_platform->front() + 1)->x;
  	mid_beg_plat_x = points_of_platform->front().y  + (&points_of_platform->front() + 1)->y;
  	change_x = mid_beg_plat_x - scan->range_x1;
  	change_y = mid_beg_plat_y - scan->range_y1;
  	double distance = std::sqrt((change_y*change_y) + (change_x*change_x)); 
  	return distance <= 10; 
  }
  /*create a struct of line segment data according to input*/
  line_segment PlatformFilter::calculateLine(double beginning_x, double beginning_y, double end_x, double end_y)
  {
    line_segment line;
    if(end_x - beginning_x != 0)
      line.m = (end_y - beginning_y) / (end_x - beginning_x);
    else
    {
    	if(end_y - beginning_y > 0)
    		line.m = 90;
    	else if(end_y - beginning_y < 0)
    		line.m = 180;
    	else
    		line.m = 0;
    }
    line.b = end_y - line.m * end_x;
    line.range_x1 = beginning_x;
    line.range_x2 = end_x;
    line.range_y1 = beginning_y;
    line.range_y2 = end_y;
    return line;
  }

  /*control that is scan data coming from expected points that named str_polygon*/
  bool PlatformFilter::exactlyPlatform(line_segment *scan, std::string str_polygon)
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
  /*collect the new positions data*/
  void PlatformFilter::tf_update()
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
    laser_yaw_ = tf_transform_laser.getRotation().getAngle();

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
      calculate_yaw(&yaw, &points_of_platform);// calculate direction angle(yaw)
	    calculate_direction(temp.transport, *pitch_angles, yaw);//calculate the amount of displacement and assign to the temp.transport
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
  void PlatformFilter::calculate_direction(double *direction, double pitch, double yaw)
  {
    direction[1] = std::sin(PI*yaw/180) * laser_z_ * (1.0/std::tan(PI*pitch/180));//y direction
    direction[0] = std::cos(PI*yaw/180) * laser_z_ * (1.0/std::tan(PI*pitch/180));
  }
  /*calculate middle of the polygon and set the yaw as a vector that is through beginning line's middle to that point*/
  void PlatformFilter::calculate_yaw(double *yaw, std::vector<geometry_msgs::Point32> *vec)
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
    	if(middle_y - mid_beg_plat_y < 0)
        *yaw = 180;
    	else if(middle_y - mid_beg_plat_y > 0)
    	  *yaw = 90;
      else
    		*yaw = 0;
    else
      *yaw = std::atan((middle_y - mid_beg_plat_y) / (middle_x - mid_beg_plat_x))*180/PI;
  }
}
