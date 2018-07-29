#include <pathfinder.h>

int main(int argc, char **argv) {

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

    return 0;
}
