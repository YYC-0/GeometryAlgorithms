#pragma once

#include "GeomDataStructure.h"
#include <vector>
#include <Eigen/Dense>
using namespace std;
using namespace Geometry;

class Geom
{
public:
	Geom() = default;

	static double squareDis(const Point2f& p1, const Point2f& p2);
	static double distance(const Point2f& p1, const Point2f& p2);
	static double dot(const Vector2f& v1, const Vector2f& v2);
	static double cross(const Vector2f& v1, const Vector2f& v2);
	static double angle(const Vector2f& v1, const Vector2f& v2, bool isRadian = true);
	// Compute distance from a point to a line
	static double distance(const Line &line, const Point2f &p);
	// Compute distance from a point to a segment
	static double distance(const Segment &segment, const Point2f &p);
	// If a point p is on the left of a line l1-l2
	static int isLeft(const Point2f &l1, const Point2f &l2, const Point2f &p);
	// If vector v2 is left of v1
	static int isLeft(const Vector2f &v1, const Vector2f &v2);
	// If a point is in a circle or not
	static bool IsInCircle(const Circle &circle, const Point2f &p);
	// If a poitn is in a ellipse or not
	static bool IsInEllipse(const Ellipse &ellipse, const Point2f &p);
	// Compute circumcircle of triangle
	static Circle circumcircleOfTriangle(Triangle2f triangle);
	// Compute the smallest circle
	static Circle minCircle(vector<Point2f> points);
	// Compute intersec points of a circle and segment line
	static vector<Point2f> intersectPoints(const Circle& circle, const Point2f& p1, const Point2f& p2);
	// Compute intersect points of two circles
	static vector<Point2f> intersectPoints(const Circle& circle1, const Circle &circle2);
	// Compute intersec points of an ellipse and segment line
	static vector<Point2f> intersectPoints(const Ellipse &e, const Point2f &p1, const Point2f &p2);
	// Compute intersec points of a polygon and segment line
	static vector<Point2f> intersectPoints(const Polygon &polygon, const Point2f &p1, const Point2f &p2);
	// Compute intersect area of circle and polygon
	static double intersectArea(const Polygon& polygon, const Circle& circle);
	// Compute intersect area of two circle
	static double intersectArea(const Circle& circle1, const Circle& circle2);
	// Triangulation
	static vector<Polygon> triangulation(Polygon polygon);
	// Convex Decomposition
	static vector<Polygon> convexDecomposition(const Polygon& polygon);
	// Finds the minimum volume enclsing ellipsoid (MVEE) of a set of points stored in matrix P
	static pair<Eigen::MatrixXd, Eigen::VectorXd> MinVolEllipse(Eigen::MatrixXd P, double tolerance);
	// Finds the minimum ellipse of a 2D points
	static Ellipse minEllipseCoverage(const vector<Point2f>& points);
	// Generate random simple polygon
	static Polygon generateRandomPolygon(int edgeNum, Point2f center, int maxRadius, unsigned int seed = 0);
	// Convex shape decomposition
	static vector<Polygon> ConvexShapeDecomposition(const Polygon &polygon, int t, double eps);
	// Compute shorest path
	static vector<int> ShorestPathDijkstra(Graph graph, int beginIdx, int endIdx);

private:
	// used for triangulation
	static void tri_recursive(Polygon polygon, vector<Polygon>& triangles);
	static bool edge_valid(const Polygon &polygon, int idx1, int idx2);
	static float antiClockAngle(Vector2f v1, Vector2f v2); // compute anticlock angle of v1 and v2
	// used for convexDecomposition
	struct SharedEdge {
		Point2f p1;
		Point2f p2;
		int edge1Idx1;
		int edge1Idx2;
		int edge2Idx1;
		int edge2Idx2;
		int polyIdx1;
		int polyIdx2;
	};
	static bool mergeLongestSharedEdge(vector<Polygon>& polygons);	
	static bool shared_edge(Polygon polygon1, Polygon polygon2, SharedEdge& edge);
	static bool mergeEdgeValid(Polygon polygon1, Polygon polygon2, SharedEdge& edge);
	static Polygon mergePolygon(Polygon polygon1, Polygon polygon2, SharedEdge& edge);

	// used for ConvexShapeDecomposition
	
};
