#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Path.h"
using namespace ros;

#include <pathfinder.h>

#include <iostream>
#include <vector>
using namespace std;

int sampleCount;
float timeStep;
float maxVel;
float maxAccel;
float maxJerk;

Publisher pathPublisher;

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

	int NUM_POINTS = 3;
    Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * NUM_POINTS);

    for(int i = 0; i < msg->poses.size(); i++) {
        Waypoint pt = {msg->poses[i].position.x, msg->poses[i].position.y, quaternionToYaw(msg->poses[i].orientation)};
        points[i] = pt;
    }

    TrajectoryCandidate candidate;
    pathfinder_prepare(points, NUM_POINTS, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_FAST,
                        timeStep, maxVel, maxAccel, maxJerk, &candidate);

    int trajectory_length = candidate.length;
    Segment *trajectory = (Segment*) malloc(trajectory_length * sizeof(Segment));
    
    pathfinder_generate(&candidate, trajectory);
   
    vector<geometry_msgs::PoseStamped> pathArr;

    for (int i = 0; i < trajectory_length; i++) {
        Segment s = trajectory[i];
        
        geometry_msgs::PoseStamped seg;
        seg.header = msg->header;
        seg.pose.orientation = tf::createQuaternionMsgFromYaw(s.heading);
        seg.pose.position.x = s.x;
        seg.pose.position.y = s.y;
        seg.pose.position.z = 0.0;

        // printf("Time Step: %f\n", s.dt);
        // printf("Coords: (%f, %f)\n", s.x, s.y);
        // printf("Position (Distance): %f\n", s.position);
        // printf("Velocity: %f\n", s.velocity);
        // printf("Acceleration: %f\n", s.acceleration);
        // printf("Jerk (Acceleration per Second): %f\n", s.jerk);
        // printf("Heading (radians): %f\n", s.heading);
        
        pathArr.push_back(seg);
    }
    nav_msgs::Path path;
    path.header = msg->header;
    path.poses = pathArr;

    cout << "Publishing path with length " << pathArr.size() << endl;
    pathPublisher.publish(path);

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
    pathPublisher = n.advertise<nav_msgs::Path>("/pathfinder_ros/path", 1);

    ros::spin();

    return 0;
}

// Pathfinder code
/*
	int POINT_LENGTH = 3;

    Waypoint *points = (Waypoint*)malloc(sizeof(Waypoint) * POINT_LENGTH);

    Waypoint p1 = { -4, -1, d2r(45) };      // Waypoint @ x=-4, y=-1, exit angle=45 degrees
    Waypoint p2 = { -1, 2, 0 };             // Waypoint @ x=-1, y= 2, exit angle= 0 radians
    Waypoint p3 = {  2, 4, 0 };             // Waypoint @ x= 2, y= 4, exit angle= 0 radians
    points[0] = p1;
    points[1] = p2;
    points[2] = p3;
    
    TrajectoryCandidate candidate;
    pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_FAST, 0.05, 15.0, 10000000.0, 1000000000.0, &candidate);

    int length = candidate.length;
    Segment *trajectory = (Segment*) malloc(length * sizeof(Segment));
    
    pathfinder_generate(&candidate, trajectory);
   
    int i;
    for (i = 0; i < 2; i++) {
        Segment s = trajectory[i];
        printf("Time Step: %f\n", s.dt);
        printf("Coords: (%f, %f)\n", s.x, s.y);
        printf("Position (Distance): %f\n", s.position);
        printf("Velocity: %f\n", s.velocity);
        printf("Acceleration: %f\n", s.acceleration);
        printf("Jerk (Acceleration per Second): %f\n", s.jerk);
        printf("Heading (radians): %f\n", s.heading);
    }

    free(trajectory);
*/
