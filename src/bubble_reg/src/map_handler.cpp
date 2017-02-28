#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/TimeReference.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TwistStamped.h"
#include "geodesy/utility.h"
#include <iostream>
#include <math.h>

class Map
{
public:
    Map(){
        // Subscribers
        gpsFix_sub = node.subscribe("fix", 1, &Map::updateGpsFix, this);
//        gpsVel_sub = node.subscribe("vel", 1, &Map::updateGpsVel, this);
//        timeRef_sub = node.subscribe("time_reference", 1, &Map::updateTimeRef, this);

        // Publishers
        pose_pub = node.advertise<geometry_msgs::Pose>("cmd", 1);

        // Internal variables
    }

    void updateGpsFix(const sensor_msgs::NavSatFix::ConstPtr& msg){
        pose.position.x = latDeg2meters(msg->latitude, latOrigin);
        pose.position.y = longDeg2meters(msg->longitude, latOrigin, longOrigin);
        pose.position.z = msg->altitude;
        ROS_DEBUG("I received an estimated position: ([%f], [%f], [%f])", pose.position.x, pose.position.y, pose.position.z);
    }

//    void updateGpsVel(const geometry_msgs::TwistStamped::ConstPtr& msg){
//        ROS_DEBUG("I received an estimated position: ([%f], [%f], [%f])", x, y, z);
//    }

//    void updateTimeRef(const sensor_msgs::NavSatFix::ConstPtr& msg){
//        ROS_DEBUG("I received an estimated position: ([%f], [%f], [%f])", x, y, z);
//    }

    void updateCommand(){

    }

    void spin(){

        ros::Rate loop(10);

        while (ros::ok()){

            // call all waiting callbacks
            ros::spinOnce();

            updateCommand();
            // publish the command
            pose_pub.publish(pose);

            loop.sleep();

        }
    }

private:
    // Node
    ros::NodeHandle node;

    // Suscribers
    ros::Subscriber gpsFix_sub;
    ros::Subscriber gpsVel_sub;
    ros::Subscriber timeRef_sub;

    // Publishers
    ros::Publisher pose_pub;

    // Internal variables
    geometry_msgs::Pose pose;
    double latOrigin = 60;
    double longOrigin = 0;
    float x, y, z;
    float theta;
};


int main(int argc, char **argv)
{
    // Node initialization
    std::cout << "Node initialization " << std::endl;
    ros::init(argc, argv, "map_handler");

    Map map;

    map.spin();
    return 0;
}
