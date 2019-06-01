#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

static const double POS_THRESH    = 0.0001;
static const double PICKUP_POS_X  = 3.8;
static const double PICKUP_POS_Y  = 2.6;
static const double DROPOFF_POS_X = 3.8;
static const double DROPOFF_POS_Y = -0.5;

static bool carryMarker = false;
static ros::Publisher marker_pub;

void addMarker(double x, double y){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = "marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.3;
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;
  marker.lifetime = ros::Duration();

  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.orientation.w = 1.0;

  marker_pub.publish(marker);
}

void hideMarker(){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = "marker";
  marker.id = 0;
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
}

void onAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  if(!carryMarker && (abs(x - PICKUP_POS_X) < POS_THRESH) && (abs(y - PICKUP_POS_Y) < POS_THRESH)){
    ROS_INFO("Pick the object up!");
    hideMarker();
    carryMarker = true;
  }
  else if(carryMarker && (abs(x - DROPOFF_POS_X) < POS_THRESH) && (abs(y - DROPOFF_POS_Y) < POS_THRESH)){
    addMarker(DROPOFF_POS_X, DROPOFF_POS_Y);
    carryMarker = false;
    ROS_INFO("Drop off the object");
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "add_marker");
  ros::NodeHandle n;

  ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 1, onAmclPose);
  marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

  // Wait until at least one subscriber appears
  while (marker_pub.getNumSubscribers() < 1){
    if (!ros::ok()){
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    ros::Duration(1.0).sleep();
  }
  //ros::Duration(5.0).sleep();
  ROS_INFO("Show PICKUP");
  addMarker(PICKUP_POS_X, PICKUP_POS_Y); 
 
/*  ros::Duration(5.0).sleep();
  ROS_INFO("hide PICKUP");
  hideMarker();
  ros::Duration(5.0).sleep();
  ROS_INFO("Show DROPOFF");
  addMarker(DROPOFF_POS_X, DROPOFF_POS_Y);
  ros::Duration(5.0).sleep();
  ROS_INFO("hide DROPOFF");
  hideMarker();
**/

  ros::spin();

  return 0;
}




/*

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    // Cycle between different shapes
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }


    r.sleep();
  }
} 

**/
