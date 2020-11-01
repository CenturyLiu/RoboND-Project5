#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

using namespace std;

// public publisher and subscriber
ros::Publisher marker_pub;
ros::Subscriber odom_sub;

// public indicator
bool reached_pickup = false;
bool reached_drop_off = false;

// constants
double tolerance = 2.0;

double pick_odom_x = -12.0;
double pick_odom_y = -7.0;
double drop_odom_x = 6.0;
double drop_odom_y = 10.0;

double pick_marker_x = pick_odom_x;//-7.0;
double pick_marker_y = pick_odom_y;//12.0;
double drop_marker_x = drop_odom_x;//10.0;
double drop_marker_y = drop_odom_y;//-6.0;


double euc_distance(double x1,double y1,double x2,double y2){
    return sqrt(pow((x1 - x2), 2) + pow((y1 - y2),2));
}



void publish_marker(double map_x, double map_y){
    uint32_t shape = visualization_msgs::Marker::CUBE;

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "goal";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = map_x;
    marker.pose.position.y = map_y;
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

    marker.lifetime = ros::Duration(0.1);

    // Publish the marker
    marker_pub.publish(marker);

}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    // extract the x,y of odom
    double odom_x = msg->pose.pose.position.x;
    double odom_y = msg->pose.pose.position.y;

    if (!reached_pickup){
        // calculate the distance between the current position and
        // the pick up pose
        if(euc_distance(odom_x,odom_y,pick_odom_x,pick_odom_y) <= tolerance){
            reached_pickup = true; // reached the pick up point, indicating that the object has been picked up
        }
        else{
            // has not reach pick up point
            // display the marker
            publish_marker(pick_marker_x, pick_marker_y);
        }
    }
    else{
        if (!reached_drop_off){
            if(euc_distance(odom_x,odom_y,drop_odom_x,drop_odom_y) <= tolerance){
                // reached goal position, display the marker
                reached_drop_off = true;
                publish_marker(drop_marker_x,drop_marker_y);
            }
            // do nothing if not
        }
        else{
            // has reached the drop off zone
            // display marker
            publish_marker(drop_marker_x,drop_marker_y);
        }
    }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // define the listener of odom
  odom_sub = n.subscribe("/odom", 1, odomCallback);


// %EndTag(INIT)%

  // Set our initial shape type to be a cube


    ros::spin();

}


