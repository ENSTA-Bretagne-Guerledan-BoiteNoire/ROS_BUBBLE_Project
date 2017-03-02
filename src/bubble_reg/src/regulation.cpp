#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geodesy/utility.h"
#include "tf/transform_datatypes.h"
#include "bubble_msgs/Line.h"
//#include <iostream>
//#include <math.h>

enum State{
    manual = 0,
    blackBoxResearch = 1,
    stationKeeping = 2
};

class Regulation
{
public:
    Regulation(){
        // Subscribers
        poseReal_sub = node.subscribe("pose_est", 1, &Regulation::updatePoseReal, this);
        twistReal_sub = node.subscribe("twist_est", 1, &Regulation::updateTwistReal, this);
        obj_sub = node.subscribe("objective", 1, &Regulation::updateObj, this);
        cmdState_sub = node.subscribe("cmd_state", 1, &Regulation::updateCmdState, this);

        // Publishers
        cmdVel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        // Internal variables
        cmd_state = manual;

        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = 0;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.linear.z = 0;

        pose_real.position.z = 0;
        pose_real.position.x = 0;
        pose_real.position.y = 0;

        followedLine.prevWaypoint.x = 0; //x en metres
        followedLine.prevWaypoint.y = 0; //y en metres
        followedLine.prevWaypoint.z = 0;
        followedLine.nextWaypoint.x = 0; //x en metres
        followedLine.nextWaypoint.y = 0; //y en metres
        followedLine.nextWaypoint.z = 0;

        angle_ping = -180; // -180 = non trouvé

        const tf::Quaternion q = tf::createQuaternionFromYaw(0);
        pose_real.orientation.z = q.z();
        pose_real.orientation.w = q.w();
        pose_real.orientation.x = q.x();
        pose_real.orientation.y = q.y();
    }

    void updatePoseReal(const geometry_msgs::Pose::ConstPtr& msg){
        pose_real.orientation = msg->orientation;
        pose_real.position = msg->position;
        ROS_DEBUG("I received an estimated position: ([%f], [%f], [%f])", pose_real.position.x, pose_real.position.y, pose_real.position.z);
    }

    void updateTwistReal(const geometry_msgs::Twist::ConstPtr& msg){
        twist_real.angular = msg->angular;
        twist_real.linear = msg->linear;
    }

    void updateObj(const bubble_msgs::Line::ConstPtr& msg){
        followedLine.nextWaypoint = msg->nextWaypoint;
        followedLine.prevWaypoint = msg->prevWaypoint;
    }

    void updateCmdState(const std_msgs::Int8::ConstPtr& msg){
        cmd_state = msg->data;
    }

    void updateCommand(){

        const double ax = followedLine.prevWaypoint.x;
        const double ay = followedLine.prevWaypoint.y;
        const double bx = followedLine.nextWaypoint.x;
        const double by = followedLine.nextWaypoint.y;
        const double x = pose_real.position.x;
        const double y = pose_real.position.y;
        const double head = ned2enu_yaw_rad(tf::getYaw(pose_real.orientation));

        const double headLine = atan2(by-ay,bx-ax); // ENU convention
        const double penteLine = (ay-by)/(ax-bx);
        const double offsetLine = ay - penteLine*ax;
        const double dist2Line = fabs(penteLine*x + y + offsetLine)/sqrt( pow(x,2) + pow(y,2) );
        const double wantedHead = angle_rad(headLine,- atan(dist2Line));

        const double twist = angle_rad(wantedHead,- head)/2.0;
        const double dist2Obj = distance(x,y,bx,by);

        cmd_vel.angular.z = twist;
        cmd_vel.linear.x = atan(1*dist2Obj); // Le *1 c'est pour que le bateau ralentisse à 1/1m
    }

    void spin(){

        ros::Rate loop(10);

        while (ros::ok()){

            // call all waiting callbacks
            ros::spinOnce();

            if(cmd_state!=manual){

                updateCommand();

                // publish the command
                cmdVel_pub.publish(cmd_vel);
            }

            loop.sleep();

        }
    }

private:
    // Node
    ros::NodeHandle node;

    // Suscribers
    ros::Subscriber cmdState_sub;
    ros::Subscriber twistReal_sub;
    ros::Subscriber anglePing_sub;
    ros::Subscriber obj_sub;
    ros::Subscriber poseReal_sub;

    // Publishers
    ros::Publisher cmdVel_pub;

    // Internal variables
    int cmd_state;
    double angle_ping;
    geometry_msgs::Twist pose;
    geometry_msgs::Twist twist_real;
    geometry_msgs::Pose pose_real;
    geometry_msgs::Twist cmd_vel;
    bubble_msgs::Line followedLine;
};


int main(int argc, char **argv)
{
    // Node initialization
    std::cout << "Node initialization " << std::endl;
    ros::init(argc, argv, "bubble_reg");

    Regulation regulation;

    regulation.spin();
    return 0;
}
