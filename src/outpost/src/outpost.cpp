#include "outpost.hpp"
#include <libstatistics_collector/topic_statistics_collector/constants.hpp>

//前哨站数据
//装甲板底部到地面高度： 1166 装甲板顶部到地面的距离：1331
//装甲板圈半径：553 / 2
//旋转速度： 0.8pi rad/s
//两块装甲板到达同一位置间隔时间 0.833333s

Outpost::Outpost() : Node("outpost")
{
    tf2_init();

    left_armors_sub_ = this->create_subscription<auto_aim_interfaces::msg::Armors>(
                  "/left_camera/detector/armors", rclcpp::SensorDataQoS(), std::bind(&Outpost::left_armors_callback, this, std::placeholders::_1));

                 update_sub_ = this->create_subscription<std_msgs::msg::Int8>(
                  "/left_camera/update", rclcpp::SensorDataQoS(), std::bind(&Outpost::updateCallback, this, std::placeholders::_1));
    
    outpost_points_pub_ = this->create_publisher<auto_aim_interfaces::msg::Points>(
        "/outpost/points", 10);

    outpost_point_pub_ = this->create_publisher<auto_aim_interfaces::msg::Point>(
        "/outpost/point", 10);

        marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/outpost/marker", 10);

    start_time = std::chrono::steady_clock::now();

    // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  point_marker_.ns = "points";
  point_marker_.action = visualization_msgs::msg::Marker::ADD;
  point_marker_.type = visualization_msgs::msg::Marker::CUBE;
  point_marker_.scale.x = 0.05;
  point_marker_.scale.z = 0.125;
  point_marker_.color.a = 1.0;
  point_marker_.color.g = 0.5;
  point_marker_.color.b = 1.0;
  point_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
}

void Outpost::tf2_init()
{
  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
   timestamp_offset_ = 0.00;
}

void Outpost::left_armors_callback(const auto_aim_interfaces::msg::Armors msg)
{
    if(tf2_buffer_->canTransform("left_camera_optical_frame", "leftodom", tf2::TimePointZero)){
      //std::cout << "Frames can be transformed" << std::endl;
    }else{
      return;
    }

  if(msg.armors.size() == 0 )
  {
    auto_aim_interfaces::msg::Point point;
    point.x = 0;
    point.y =0;
    point.z = 0;
    point.yaw = 0;
    outpost_point_pub_->publish(point);
    return;
  }
  geometry_msgs::msg::PointStamped ps;
  geometry_msgs::msg::PointStamped armor;
  ps.header = msg.header;
  ps.header.frame_id = "left_camera_optical_frame";
  ps.header.stamp = msg.header.stamp;
  ps.point.x = msg.armors[0].pose.position.x;
  ps.point.y = msg.armors[0].pose.position.y;
  ps.point.z = msg.armors[0].pose.position.z;
  try {
    ps.point = tf2_buffer_->transform(ps, "leftodom").point;
  } catch (const tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
    return;
  }
    end_time = std::chrono::steady_clock::now();
    auto time = std::chrono::duration_cast<chrono::milliseconds>(end_time - start_time);
    double yaw = abs(orientationToYaw(msg.armors[0].pose.orientation));
    if(armor_points.size() !=0)
    {
     //std::cout << "not" <<std::endl;
     cv::Point3d last_armor_point = cv::Point3d(armor_points[armor_points.size()-1].x,armor_points[armor_points.size()-1].y,armor_points[armor_points.size()-1].z);
     cv::Point3d now_armor_point = cv::Point3d(ps.point.x,ps.point.y,ps.point.z);
        //if(abs(yaw - lastyaw) > (60 / 57.3f) && lastyaw != 0) //需更好的判断什么时候装甲板消失
        if(get_distance(last_armor_point,now_armor_point) > 0.3)
        {
            //std::cout << "armor jumping" <<std::endl;
            if(armor_points.size()>20)
            {
            if(save_armor_points.size()< armor_points.size() && save_armor_points.size() < 20)
              {
              //需误差修正
              //std::cout << "publish" <<std::endl;
                save_armor_points = armor_points;
                auto_aim_interfaces::msg::Points points;
                for(int i =0;i<armor_points.size();i++)
                {
                  auto_aim_interfaces::msg::Point point;
                  point.x = armor_points[i].x;
                  point.y = armor_points[i].y;
                    point.z = armor_points[i].z;
                    point.yaw = armor_points[i].yaw;
                    point.timetolast = armor_points[i].timetolast;
                    points.points.push_back(point);
                  }
                  outpost_points_pub_->publish(points);
              }
            if(fittingToCircle_needtoupdate(armor_points,save_armor_points))
            {
                save_armor_points = armor_points;
                auto_aim_interfaces::msg::Points points;
                for(int i =0;i<armor_points.size();i++)
                {
                  auto_aim_interfaces::msg::Point point;
                  point.x = armor_points[i].x;
                  point.y = armor_points[i].y;
                    point.z = armor_points[i].z;
                    point.yaw = armor_points[i].yaw;
                    point.timetolast = armor_points[i].timetolast;
                    points.points.push_back(point);
                  }
                  outpost_points_pub_->publish(points);
            }
            armor_points.clear();
            }
            else
            {
                armor_points.clear();
            }
        }
    }
    if(time.count() > 10)
    {
        cv::Point3d armor_point = cv::Point3d(ps.point.x,ps.point.y,ps.point.z);
        if(armor_points.size() != 0)
        {
          cv::Point3d saved_armor_point = cv::Point3d(armor_points[armor_points.size()-1].x,armor_points[armor_points.size()-1].y,armor_points[armor_points.size()-1].z);
          if(get_distance(armor_point,saved_armor_point) > 0.01 && get_distance(armor_point,saved_armor_point) < 0.2 )
          {
            Mypoint mypoint;
            mypoint.x = armor_point.x;
            mypoint.y = armor_point.y;
            mypoint.z = armor_point.z;
            mypoint.yaw = yaw;
            mypoint.timetolast = time.count();
            armor_points.push_back(mypoint);
            //std::cout << "push points" <<std::endl;
          }
        }
        else
        {
            Mypoint mypoint;
            mypoint.x = armor_point.x;
            mypoint.y = armor_point.y;
            mypoint.z = armor_point.z;
            mypoint.yaw = yaw;
            mypoint.timetolast = time.count();
            armor_points.push_back(mypoint);
            //std::cout << "push points" <<std::endl;
        }
        start_time = std::chrono::steady_clock::now();
    }
    lastyaw = yaw;
    auto_aim_interfaces::msg::Point point;
    point.x = ps.point.x;
    point.y = ps.point.y;
    point.z = ps.point.z;
    point.yaw = yaw;
    point.timetolast = time.count();
    outpost_point_pub_->publish(point);
}

  // 用于将给定的四元数表示的姿态（朝向）转换为偏航角（yaw）
double Outpost::orientationToYaw(const geometry_msgs::msg::Quaternion & q)
{
  // Get armor yaw
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

double Outpost::get_distance(cv::Point3d point_one,cv::Point3d point_two)
{
    double distance = sqrt(pow(point_one.x - point_two.x,2)+pow(point_one.y - point_two.y,2)+pow(point_one.z - point_two.z,2));
    return distance;
}

double Outpost::get_distance_2f(cv::Point2f point_one,cv::Point2f point_two)
{
    double distance = sqrt(pow(point_one.x - point_two.x,2)+pow(point_one.y - point_two.y,2));
    return distance;
}

void Outpost::updateCallback(const std_msgs::msg::Int8 msg)
{
  if(msg.data == 1)
  {
    save_armor_points.clear();
  }
}

bool Outpost::fittingToCircle_needtoupdate(std::vector<Mypoint> &new_armor_points,std::vector<Mypoint> &save_armor_points)
{
    std::vector<cv::Point2f> new_circle_points;
    std::vector<cv::Point2f> save_circle_points;
    for(int i = 0;i<new_armor_points.size();i++)
    {
        cv::Point2f circle_point;
        circle_point.x = new_armor_points[i].x;
        circle_point.y = new_armor_points[i].y;
        new_circle_points.push_back(circle_point);
    }
    for(int i = 0;i<save_armor_points.size();i++)
    {
        cv::Point2f circle_point;
        circle_point.x = save_armor_points[i].x;
        circle_point.y = save_armor_points[i].y;
        save_circle_points.push_back(circle_point);
    }
    cv::Point2f new_circle_center;
    cv::Point2f save_circle_center;
    float new_radius;
    float save_radius;
    //std::cout << "circle_points.size()" << circle_points.size() <<std::endl;
    cv::minEnclosingCircle(new_circle_points,new_circle_center, new_radius);
    cv::minEnclosingCircle(save_circle_points,save_circle_center, save_radius);
    std::cout << "get_distance_2f(new_circle_center,save_circle_center): " <<get_distance_2f(new_circle_center,save_circle_center) <<std::endl;
    if(get_distance_2f(new_circle_center,save_circle_center) > 0.2)
    {
      return true;
    }
    if(abs(new_radius - 0.2765) < abs(save_radius - 0.2765))
    {
      return true;
    }
    return false;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Outpost>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}