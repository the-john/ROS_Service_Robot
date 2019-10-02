#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int8.h>

int8_t callback_message = 0;

// Below is the function that gets called whenever a message comes in on the "location" topic
// The message is passed in a "boost shared_ptr" which means I can store it, and I do for now
void callback(const std_msgs::Int8::ConstPtr& msg)
{
  ROS_INFO("Message Received: [%d]", msg->data);
  callback_message = msg->data;
}

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

  ros::Subscriber location_sub = n.subscribe("/int8", 1, callback);
  // As a subscriber, we are subscribing to the "uint8" topic with the master
  // The message queue size is only 1 (we will throw away old messages as new ones arive)
  // "callback" is the function called whenever a new message arrives on the "uint8" topic
  // NodeHandle::subscribe() returns a ros::Subscriber object (hold onto until unsubscribe)
  ros::spinOnce();			// calls message callbacks as fast as possible

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
    marker.scale.x = 0.33;
    marker.scale.y = 0.33;
    marker.scale.z = 0.33;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;	// Alpha = transparency; 1 = opaic, 2 = transparent

    //marker.lifetime = ros::Duration(5.0);	// Marker should only last 5 seconds
						// countdown resets is same marker is received
    marker.lifetime = ros::Duration();		// Turn on marker indefinitely

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


  while (ros::ok())  			// Make sure our node is up and running
  {
    ros::spinOnce();			// Make sure callbacks get called if applicable

    ROS_WARN_ONCE("Callback Message is: [%d]", callback_message);
    
    if (callback_message == 1){
      ROS_WARN_ONCE("Robot at pick-up location, wait 5 seconds for pick-up");
      ros::Duration(2.5).sleep();
      ROS_WARN_ONCE("Robot Has Package");
      marker.action = visualization_msgs::Marker::DELETE;		// Remove old marker
      marker_pub.publish(marker);	// Publish the marker removal
    }

    if (callback_message == 2){
      ROS_WARN_ONCE("Robot at drop-off destination");
      ROS_WARN_ONCE("Robot dropping off package!");
      marker.action = visualization_msgs::Marker::ADD;			// Add old marker
      marker.pose.position.x = -3.0;					// New location x
      marker.pose.position.y = -3.0;					// New location y
      marker.lifetime = ros::Duration();// Marker stays on indefinately
      marker_pub.publish(marker);	// Publish the marker at new location
    }

    loop_rate.sleep();
  }
  return 0;
}



