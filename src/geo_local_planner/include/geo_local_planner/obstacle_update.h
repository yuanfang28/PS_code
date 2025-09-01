#ifndef OBSTACLE_UPDATE_H_
#define OBSTACLE_UPDATE_H_


#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>

#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry.hpp>

namespace bg = boost::geometry;

namespace geo_local_planner
{
    typedef bg::model::point<double, 2, bg::cs::cartesian> b_point;
    typedef bg::model::segment<b_point> b_segment;
    typedef bg::model::linestring<b_point> b_lineString;
    typedef bg::model::polygon<b_point> b_polygon;
    typedef bg::model::multi_polygon<b_polygon> b_circle;

    struct circle {
        b_circle circ;
        b_point center;
        double radius;
    };


    bool isLineIntersectingPolygon(const b_polygon &poly, const b_lineString &lineString);
    
    int orientation(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r);
    bool doIntersect(geometry_msgs::Point p1, geometry_msgs::Point q1, geometry_msgs::Point p2, geometry_msgs::Point q2);
    bool onSegment(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r);
    bool isSegmentIntersectingPolygon(const std::vector<geometry_msgs::Point>& polygon, geometry_msgs::Point p1, geometry_msgs::Point p2);
    

}
#endif