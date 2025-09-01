#include "geo_local_planner/obstacle_update.h"

using namespace std;
namespace geo_local_planner {



// Utility function to find orientation of ordered triplet (p, q, r).
// The function returns:
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r) 
{
    double val = (q.y - p.y) * (r.x - q.x) -
                (q.x - p.x) * (r.y - q.y);
    if (val == 0) return 0;  // collinear
    return (val > 0)? 1: 2; // clock or counterclockwise
}

// The function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool doIntersect(geometry_msgs::Point p1, geometry_msgs::Point q1, geometry_msgs::Point p2, geometry_msgs::Point q2) 
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are collinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and p2 are collinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

    // p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases
}

// // Function to check whether a point is on a segment
bool onSegment(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r) 
{
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
        q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
    return true;
    return false;
}

// Function to check if a given segment intersects with a polygon std::vector<geometry_msgs::Point32, std::allocator<geometry_msgs::Point32>> 
bool isSegmentIntersectingPolygon(const std::vector<geometry_msgs::Point>& polygon, geometry_msgs::Point p1, geometry_msgs::Point p2) 
{
    size_t n = polygon.size();
    for (size_t i = 0; i < n; i++) {
        geometry_msgs::Point p3 = polygon[i];
        geometry_msgs::Point p4 = polygon[(i + 1) % n];  // Ensuring the last vertex connects to the first
        if (doIntersect(p1, p2, p3, p4)) {
            return true;
        }
    }
    return false;
}


}