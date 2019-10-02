#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int pick_up = 0;


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");	// Initialize ROS & define node name of "add_markers"
  ros::NodeHandle n;			// Main access point to ROS system & initialize node
  ros::Rate loop_rate(1);		// The frequency we would like to loop at, 1Hz
  // Advertise = what we want to publish and on what topic name
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // NOW the master node has this in its register as a publisher
  // We can now publish() out a message of type bisualization_msgs/Marker
  // on the topic called "visualization_marker"
  // Size of message publishing queue is 1 before beginning to throw away old ones

// Define all of the details of the marker and publish it

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // This is a message object.  Stuff it with data and publish it.

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type to CUBE
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 1.0;
    marker.pose.position.y = 2.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;	// Alpha = transparency; 1 = opaic, 2 = transparent

    marker.lifetime = ros::Duration(5.0);	// Marker should only last 5 seconds
						// countdown resets if sam marker is received

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
    marker_pub.publish(marker);		// Publish the marker to the world

    ros::spinOnce();			// Make sure callbacks get called if applicable

    ros::Duration(5.0).sleep();		// Leave marker posted for 5 seconds

    ROS_WARN_ONCE("5 seconds are up.  Removing marker from pick-up location!");

    marker.action = visualization_msgs::Marker::DELETE;		// Remove old marker
    marker_pub.publish(marker);					// Publish the removal


  while (ros::ok())  			// Make sure our node is up and running
  {
    ROS_WARN_ONCE("Now waiting 5 seconds before moving marker to drop-off location.");

    // Wait another 5 seconds
    ros::Duration(5.0).sleep();

    ROS_WARN_ONCE("Now moving marker to drop-off location and turning it on.");

    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = -3.0;
    marker.pose.position.y = -3.0;

    marker.lifetime = ros::Duration(); // Marker stays on indefinately

    // Publish the marker
    marker_pub.publish(marker);

    ros::spinOnce();			// Make sure callbacks get called if applicable

    ROS_WARN_ONCE("Marker now at drop-off location.");
    
    //ros::Duration(5.0).sleep();

    loop_rate.sleep();
  }
  return 0;
}



