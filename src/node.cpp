#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
using namespace ros;

#include <pathfinder.h>

#include <iostream>
using namespace std;

int sampleCount;
float timeStep;
float maxVel;
float maxAccel;
float maxJerk;

void waypointsUpdate(const geometry_msgs::PoseArray::ConstPtr& msg) {
    cout << msg->poses.size() << endl;
    cout << msg->poses[0].position.x << endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pathfinder_node");
    NodeHandle n;
    NodeHandle params("~");

    sampleCount = params.param("sample_count", 10000);
    timeStep = params.param("time_step", 0.02);
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
