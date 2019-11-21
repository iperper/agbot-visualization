/**
 * @file visualization_node.cpp
 * @brief Visualize rosbags from agbot in rviz
 */

// ROS headers.
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Path.h>
#include <visualization/StampedInt.h>

#include <glog/logging.h>

using String = std_msgs::String;
using StampedInt = visualization::StampedInt;
using StampedIntConstPtr = visualization::StampedIntConstPtr;



/*****************GLOBAL VARS*********************/

std::vector<std::vector<double> > gps_measurements_;
std::vector<std::pair<double, double> > gps_xy_measurements_; // XY coordinates in meters
nav_msgs::Path gps_path_;
std::vector<std::pair<int, int> > encoder_measurements_;
std::vector<std::pair<int, int> > motor_current_measurement_;
std::vector<std::pair<int, int> > speed_measurements_;


/************************************************/

// Color for a point.
struct PointColor {
  PointColor(float r = 0.0, float g = 1.0, float b = 0.0, float a = 1.0)
    : r_(r), g_(g), b_(b), a_(a) {}
  float r_;
  float g_;
  float b_;
  float a_;
};

// Color for a line.
struct LineColor {
  LineColor(float r = 0.0, float g = 1.0, float b = 0.0, float a = 1.0)
    : r_(r), g_(g), b_(b), a_(a) {}
  float r_;
  float g_;
  float b_;
  float a_;
};

// Color for a trajectory.
struct TrajectoryColor {
  TrajectoryColor(const PointColor& point_color = PointColor(),
                  const LineColor& line_color = LineColor())
    : point_color_(point_color), line_color_(line_color) {}
  PointColor point_color_;
  LineColor line_color_;
};

// Example colors to use for the ground-truth trajectory (gt), etc.
TrajectoryColor color_gt_trajectory (PointColor(0.0, 1.0, 0.0, 1.0),
                                     LineColor(0.0, 1.0, 0.0, 1.0));
TrajectoryColor color_noisy_trajectory (PointColor(1.0, 0.0, 0.0, 0.7),
                                        LineColor(1.0, 1.0, 1.0, 0.7));
TrajectoryColor color_initial_trajectory (PointColor(1.0, 0.0, 0.0, 1.0),
                                          LineColor(1.0, 0.0, 0.0, 1.0));
TrajectoryColor color_optimal_trajectory (PointColor(0.0, 1.0, 1.0, 1.0),
                                          LineColor(0.0, 1.0, 1.0, 1.0));

// Draws a trajectory in Rviz, by publishing a set of lines and poses using
// the publishers passed as argument and the actual trajectory to plot.
// You can also pass what color you want the trajectory in, and the frame_id
// were the Rviz Markers should be plotted (generally set to 'world).

void drawTrajectory(
    const ros::Publisher& marker_pub,
    const ros::Publisher& pose_array_pub,
    const std::vector<geometry_msgs::PoseStamped>& trajectory,
    const TrajectoryColor& trajectory_color = TrajectoryColor(),
    const std::string& frame_id = "world") {
    // Create the vertices for the points and lines
    CHECK_GE(trajectory.size(), 0);
    geometry_msgs::PoseStamped prev_i = trajectory.at(0);

    // Create visual markers.
    visualization_msgs::Marker points, line_strip;
    points.header.frame_id = line_strip.header.frame_id = frame_id;
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "points_and_lines";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.01;
    points.scale.y = 0.01;

    // LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.01;

    // Points are green
    points.color.r = trajectory_color.point_color_.r_;
    points.color.g = trajectory_color.point_color_.g_;
    points.color.b = trajectory_color.point_color_.b_;
    points.color.a = trajectory_color.point_color_.a_;

    // Line strip is blue
    line_strip.color.r = trajectory_color.line_color_.r_;
    line_strip.color.g = trajectory_color.line_color_.g_;
    line_strip.color.b = trajectory_color.line_color_.b_;
    line_strip.color.a = trajectory_color.line_color_.a_;

    // Contains set of poses.
    geometry_msgs::PoseArray pose_array;
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = frame_id;

    // Loop over the trajectory.
    for (const geometry_msgs::PoseStamped& i: trajectory) {
      if (i.header.stamp.toNSec() < prev_i.header.stamp.toNSec()) {
        LOG(WARNING) << "Stamp of poses in trajectory should be increasing in value.\n"
                     << "Got timestamp: " << i.header.stamp.toNSec() << " for pose #" << i.header.seq << "\n"
                     << "Got timestamp: " << prev_i.header.stamp.toNSec() << " for pose #" << prev_i.header.seq;
      }
      prev_i = i;
      geometry_msgs::Point p;
      p.x = i.pose.position.x;
      p.y = i.pose.position.y;
      p.z = i.pose.position.z;

      // Create points and lines.
      points.points.push_back(p);
      line_strip.points.push_back(p);

      // Store pose axis.
      pose_array.poses.push_back(i.pose);
    }

    // Publish poses.
    pose_array_pub.publish(pose_array);

    // Publish lines and points.
    marker_pub.publish(points);
    marker_pub.publish(line_strip);
}

// void xy2pose(geometry_msgs::pose &pose, double x, double y, ros::Time stamp){

// }


/**
 * GPS long/lat distance to XY coordinates.
 * Taken from https://stackoverflow.com/questions/639695/how-to-convert-latitude-or-longitude-to-meters
**/
double haversine_dist(double long1, double long2, double lat1, double lat2){
  double R = 6378.137*1000;
  double dLat = lat2 * M_PI / 180.0 - lat1 * M_PI / 180;
  double dLong = long2 * M_PI / 180.0 - long1 * M_PI / 180;
  double a = sin(dLat/2)*sin(dLat/2) + cos(lat1*M_PI/180)*cos(lat2*M_PI/180)*sin(dLong/2)*sin(dLong/2);
  return 2*R*asin(sqrt(a));
}

std::pair<double, double> gps2XY(double long1, double long2, double lat1, double lat2){
  double R = 6378.137*1000;
  double dLat = lat2 * M_PI / 180.0 - lat1 * M_PI / 180;
  double dLong = long2 * M_PI / 180.0 - long1 * M_PI / 180;
  return std::pair<double, double> (R*dLat, R*dLong);
}

void gpsCallback(const sensor_msgs::NavSatFix &msg){
  gps_measurements_.push_back(std::vector<double> {msg.longitude, msg.latitude, msg.altitude});
  static double start_long = msg.longitude;
  static double start_lat = msg.latitude;
  std::pair<double, double> XY = gps2XY(start_long, msg.longitude, start_lat, msg.latitude);
  gps_xy_measurements_.push_back(XY);

  // Add a pose object to build trajectory
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = msg.header.stamp;
  pose.pose.position.x = XY.first;
  pose.pose.position.y = XY.second;
  geometry_msgs::Quaternion q;
  q.x = q.y = q.z = pose.pose.position.z  = 0;
  q.w = 1;
  pose.pose.orientation = q;
  gps_path_.poses.push_back(pose);
  gps_path_.header.stamp = ros::Time::now();
  gps_path_.header.frame_id = "/world";

}

void encoderCallback(const StampedIntConstPtr &encoder1, const StampedIntConstPtr &encoder2){
  encoder_measurements_.push_back(std::pair<int, int> (encoder1->data, encoder2->data));
}

void motorCurrentCallback(const StampedIntConstPtr &motor_current1, const StampedIntConstPtr &motor_current2){
  motor_current_measurement_.push_back(std::pair<int, int> (motor_current1->data, motor_current2->data));
}

void speedCallback(const StampedIntConstPtr &speed1, const StampedIntConstPtr &speed2){
  speed_measurements_.push_back(std::pair<int, int> (speed1->data, speed2->data));
}

template<typename headerless_type, typename headed_type> 
class convert_msg {

  public:
    convert_msg(ros::NodeHandle n, std::string msg_name) : nh(n) {
      pub = nh.advertise<headed_type>(msg_name+"_headed", 10, true);
      sub = nh.subscribe(msg_name, 10, &convert_msg::republish_header, this);
    };
    
    /**
     * Takes in a headless msg and republishes it with a header 
     * Assumes the republished msg has a header and data value 
    **/
    void republish_header(const headerless_type &msg){
      headed_type headed_msg;
      headed_msg.data = std::stoi(msg.data);
      headed_msg.header.stamp = ros::Time::now();
      pub.publish(headed_msg);
    };

  private:
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::NodeHandle nh;
};

class sync_msg {
  typedef message_filters::Subscriber<StampedInt> stampedInt_sub;
  typedef convert_msg<String, StampedInt> headerless_string;
  typedef message_filters::sync_policies::ApproximateTime<StampedInt, StampedInt> SyncPolicy;

  public:
    sync_msg(ros::NodeHandle nh, std::pair<std::string, std::string> msg_names, void (*callback_ptr)(const StampedIntConstPtr&, const StampedIntConstPtr&)) :
             msg1(nh, msg_names.first), msg2(nh, msg_names.second), 
             sub1(nh, msg_names.first+"_headed", 1), sub2(nh, msg_names.second+"_headed", 1),
             sync(SyncPolicy(10), sub1, sub2) {
      sync.registerCallback(boost::bind(callback_ptr, _1, _2));
    };
  
  private:
    headerless_string msg1, msg2;
    stampedInt_sub sub1, sub2;
    message_filters::Synchronizer<SyncPolicy> sync;
};


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  // Init ROS node.
  ros::init(argc, argv, "visualization");
  ros::NodeHandle local_nh("");

  std::cout << "Started visualization" << std::endl;

  ros::Subscriber gps_sub = local_nh.subscribe("/agbot_gps/fix", 10, gpsCallback);
  
  // Synchronize all of the paired msgs
  std::vector<std::pair<std::string, std::string> > coupled_msg_names = {{"/encoder1", "/encoder2"},
                                                                         {"/motorcurrent1", "/motorcurrent2"},
                                                                         {"/speed1", "/speed2"}};
  sync_msg synced_encoder(local_nh, coupled_msg_names[0], &encoderCallback);
  sync_msg synced_motor_current(local_nh, coupled_msg_names[1], &motorCurrentCallback);
  sync_msg synced_speed(local_nh, coupled_msg_names[2], &speedCallback);
  

  // ROS publishers for gps XY points.
  ros::Publisher gps_path_pub =
      local_nh.advertise<nav_msgs::Path>("/gps_xy_trajectory", 10, true);
  
  ros::Rate rate(50);

  while(ros::ok()){
    gps_path_pub.publish(gps_path_);
    ros::spinOnce();
    rate.sleep();
  }

  // ROS spin until killed.
  ros::spin();

  return 0;
}
