#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "pathfinder_ros/Path.h"
#include "pathfinder_ros/PathSegment.h"
#include "nav_msgs/Path.h"
using namespace ros;

#include <pathfinder.h>

#include <iostream>
#include <vector>
using namespace std;

#define PI 3.1415926535

int sampleCount;
float timeStep;
float maxVel;
float maxAccel;
float maxJerk;

Publisher pathPublisher;
Publisher segmentsPublisher;
Publisher pathSegmentsPublisher;

double quaternionToYaw(geometry_msgs::Quaternion quaternion) {
    float x = quaternion.x;
    float y = quaternion.y;
    float z = quaternion.z;
    float w = quaternion.w;
	tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 matrix(q);

    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);

    return yaw;
}

void waypointsUpdate(const geometry_msgs::PoseArray::ConstPtr& msg) {
    cout << "Generating Path" << endl;

	int NUM_POINTS = msg->poses.size();
    Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * NUM_POINTS);

    for(int i = 0; i < NUM_POINTS; i++) {
        float theta = quaternionToYaw(msg->poses[i].orientation);
        
        while(theta < 0.0) {
            theta += 2.0 * PI;
        }

        Waypoint pt = {msg->poses[i].position.x, msg->poses[i].position.y, theta};
        points[i] = pt;
    }

    TrajectoryCandidate candidate;
    pathfinder_prepare(points, NUM_POINTS, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_FAST,
                        timeStep, maxVel, maxAccel, maxJerk, &candidate);

    int trajectory_length = candidate.length;
    Segment *trajectory = (Segment*) malloc(trajectory_length * sizeof(Segment));
    
    pathfinder_generate(&candidate, trajectory);
   
    vector<geometry_msgs::PoseStamped> pathArr;
    vector<geometry_msgs::Pose> segmentsArr;
    vector<pathfinder_ros::PathSegment> pathSegmentsArr;

    for (int i = 0; i < trajectory_length; i++) {
        Segment s = trajectory[i];
        
        geometry_msgs::Pose seg;
        geometry_msgs::PoseStamped segStamped;
        pathfinder_ros::PathSegment pathSeg;

        seg.orientation = tf::createQuaternionMsgFromYaw(s.heading);
        seg.position.x = s.x;
        seg.position.y = s.y;
        seg.position.z = 0.0;

        segStamped.header = msg->header;
        segStamped.pose = seg;

        pathSeg.x = s.x;
        pathSeg.y = s.y;

        pathSeg.position = s.position;
        pathSeg.velocity = s.velocity;
        pathSeg.acceleration = s.acceleration;
        pathSeg.jerk = s.jerk;

        pathSeg.heading = s.heading;

        if(i == trajectory_length - 1) {
            pathSeg.angular_velocity = 0;
        } else {
            pathSeg.angular_velocity = (trajectory[i + 1].heading - s.heading) / s.dt;
        }

        pathSeg.dt = s.dt;

        // printf("Time Step: %f\n", s.dt);
        // printf("Coords: (%f, %f)\n", s.x, s.y);
        // printf("Position (Distance): %f\n", s.position);
        // printf("Velocity: %f\n", s.velocity);
        // printf("Acceleration: %f\n", s.acceleration);
        // printf("Jerk (Acceleration per Second): %f\n", s.jerk);
        // printf("Heading (radians): %f\n", s.heading);
        
        pathArr.push_back(segStamped);
        segmentsArr.push_back(seg);
        pathSegmentsArr.push_back(pathSeg);
    }

    geometry_msgs::PoseArray segments;
    nav_msgs::Path path;
    pathfinder_ros::Path pathObj;

    segments.header = msg->header;
    segments.poses = segmentsArr;

    path.header = msg->header;
    path.poses = pathArr;

    pathObj.header = msg->header;
    pathObj.path = pathSegmentsArr;

    cout << "Publishing path with length " << pathArr.size() << endl;
    pathPublisher.publish(path);
    segmentsPublisher.publish(segments);
    pathSegmentsPublisher.publish(pathObj);

    free(trajectory);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pathfinder_node");
    NodeHandle n;
    NodeHandle params("~");

    sampleCount = params.param("sample_count", 10000);
    timeStep = params.param("time_step", 0.05);
    maxVel = params.param("max_velocity", 0.8);
    maxAccel = params.param("max_acceleration", 100000.0);
    maxJerk = params.param("max_jerk", 100000.0);

    // TODO: replace with proper rosout print
    cout << "====================================================" << endl;
    cout << "Pathfinder Config: " << endl;
    cout << "Sample Count: " << sampleCount << endl;
    cout << "Time Step: " << timeStep << endl;
    cout << "Max Velocity: " << maxVel << endl;
    cout << "Max Acceleration: " << maxAccel << endl;
    cout << "Max Jerk: " << maxJerk << endl;
    cout << "====================================================" << endl;

    Subscriber sub = n.subscribe("/pathfinder_ros/waypoints", 1000, waypointsUpdate);

    // Primarily for visualization
    pathPublisher = n.advertise<nav_msgs::Path>("/pathfinder_ros/path", 1000);
    segmentsPublisher = n.advertise<geometry_msgs::PoseArray>("/pathfinder_ros/segments", 1000);
    pathSegmentsPublisher = n.advertise<pathfinder_ros::Path>("/pathfinder_ros/path_references", 1000);

    ros::spin();

    return 0;
}
