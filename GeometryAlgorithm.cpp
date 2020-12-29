#include "GeometryAlgorithm.h"
#include <random>
#include <cmath>
#include <cstdlib>
#include <time.h>
#include <iostream>
#include <set>
#include <assert.h>
const double EPS = 1.0e-8;
using namespace Geometry;

double Geom::squareDis(const Point2f & p1, const Point2f & p2)
{
	return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
}

double Geom::distance(const Point2f & p1, const Point2f & p2)
{
	return sqrt(squareDis(p1, p2));
}

double Geom::dot(const Vector2f & v1, const Vector2f & v2)
{
	return v1.x * v2.x + v1.y * v2.y;
}

double Geom::cross(const Vector2f & v1, const Vector2f & v2)
{
	return v1.x * v2.y - v1.y * v2.x;
}

// angle of two vector, [0, 180]
double Geom::angle(const Vector2f & v1, const Vector2f & v2, bool isRadian)
{
	double cos = dot(v1, v2) / (v1.length() * v2.length());
	if (isRadian)
		return acos(cos);
	else
		return acos(cos) * 180.0 / PI;
}

Circle Geom::circumcircleOfTriangle(Triangle2f triangle)
{
	Circle circumcircle;
	const Point2f& p0 = triangle.p[0];
	const Point2f& p1 = triangle.p[1];
	const Point2f& p2 = triangle.p[2];
	double a, b, c;
	a = distance(p0, p1);
	b = distance(p1, p2);
	c = distance(p0, p2);
	// compute r
	// S(tri) = a*b*c / 4R
	circumcircle.r = a * b * c / (4 * triangle.area());
	// compute c
	double tmp1 = (p0.x * p0.x + p0.y * p0.y - p1.x * p1.x - p1.y * p1.y) / 2.0;
	double tmp2 = (p0.x * p0.x + p0.y * p0.y - p2.x * p2.x - p2.y * p2.y) / 2.0;
	double x = (tmp1 * (p0.y - p2.y) - tmp2 * (p0.y - p1.y)) / ((p0.x - p1.x) * (p0.y - p2.y) - (p0.x - p2.x) * (p0.y - p1.y));
	double y = (tmp1 * (p0.x - p2.x) - tmp2 * (p0.x - p1.x)) / ((p0.y - p1.y) * (p0.x - p2.x) - (p0.y - p2.y) * (p0.x - p1.x));
	circumcircle.c = Point2f(x, y);
	return circumcircle;
}

// https://my.oschina.net/u/4389301/blog/3380446
/*

*/
Circle Geom::minCircle(vector<Point2f> points)
{
	if (points.size() == 0)
		return Circle();
	if (points.size() == 1)
		return Circle(points[0], 0);

	// random
	random_shuffle(points.begin(), points.end());

	Circle circle((points[0] + points[1]) / 2.0, distance(points[0], points[1]) / 2.0);
	for (int i = 2; i < points.size(); ++i)
	{
		if (distance(circle.c, points[i]) > circle.r)
		{
			circle.c = points[i];
			circle.r = 0;
			for (int j = 0; j < i; ++j)
			{
				if (distance(points[j], circle.c) > circle.r)
				{
					circle.c = (points[i] + points[j]) / 2.0;
					circle.r = distance(circle.c, points[j]);
					for (int k = 0; k < j; ++k)
					{
						if (distance(points[k], circle.c) > circle.r)
						{
							circle = circumcircleOfTriangle({ points[i], points[j], points[k] });
						}
					}
				}
			}
		}
	}

	return circle;
}

// https://www.cnblogs.com/lxglbk/archive/2012/08/12/2634192.html
// 输入多边形需要逆时针
double Geom::intersectArea(const Polygon & polygon, const Circle & circle)
{
	if (polygon.isClockwise())
	{
		cout << "Input polygon need anticlockwise" << endl;
		return -1;
	}
	double area = 0;
	for (int i = 0; i < polygon.size(); ++i)
	{
		double subArea = 0;
		Point2f p1 = polygon[i];
		Point2f p2 = polygon[(i + 1) % polygon.size()];
		int left = isLeft(p1, p2, circle.c);
		bool p1IsInCircle = IsInCircle(circle, p1);
		bool p2IsInCircle = IsInCircle(circle, p2);
		vector<Point2f> intersection = intersectPoints(circle, p1, p2);
		// p1 and p2 inside circle
		if (p1IsInCircle && p2IsInCircle)
		{
			Vector2f v1 = p1 - circle.c;
			Vector2f v2 = p2 - circle.c;
			subArea += cross(v1, v2) / 2.0;
		}
		// one inside circle and one not
		else if (((p1IsInCircle && !p2IsInCircle) || (!p1IsInCircle && p2IsInCircle)) && intersection.size()==1)
		{
			// triangle area + fan area
			Point2f intersectPoint;
			if (intersection.size() == 1)
				intersectPoint = intersection[0];

			Point2f Pin = p1, Pout = p2;
			if (p1IsInCircle)
			{
				Vector2f v1 = p1 - circle.c;
				Vector2f v2 = intersectPoint - circle.c;
				subArea += cross(v1, v2) / 2.0;
			}
			else if (p2IsInCircle)
			{
				Vector2f v1 = intersectPoint - circle.c;
				Vector2f v2 = p2 - circle.c;
				subArea += cross(v1, v2) / 2.0;
				Pin = p2;
				Pout = p1;
			}
			// fan area
			Vector2f v1 = intersectPoint - circle.c;
			Vector2f v2 = Pout - circle.c;
			double fanAngle = angle(v1, v2);
			if(left == 1)
				subArea += circle.r * circle.r * fanAngle / 2.0;
			else if(left == -1)
				subArea -= circle.r * circle.r * fanAngle / 2.0;
		}
		// two points outside circle
		else
		{
			if (intersection.size() < 2)
			{
				// fan area
				Vector2f v1 = p1 - circle.c;
				Vector2f v2 = p2 - circle.c;
				double fanAngle = angle(v1, v2);
				if (left == 1)
					subArea += circle.r * circle.r * fanAngle / 2.0;
				else if (left == -1)
					subArea -= circle.r * circle.r * fanAngle / 2.0;
			}
			else
			{
				// 2 fan area + 1 triangle area
				Vector2f v1 = intersection[0] - circle.c;
				Vector2f v2 = intersection[1] - circle.c;
				subArea += cross(v1, v2) / 2.0;

				v1 = p1 - circle.c;
				v2 = intersection[0] - circle.c;
				double fanAngle = angle(v1, v2);
				if (left == 1)
					subArea += circle.r * circle.r * fanAngle / 2.0;
				else if (left == -1)
					subArea -= circle.r * circle.r * fanAngle / 2.0;

				v1 = p2 - circle.c;
				v2 = intersection[1] - circle.c;
				fanAngle = angle(v1, v2);
				if (left == 1)
					subArea += circle.r * circle.r * fanAngle / 2.0;
				else if (left == -1)
					subArea -= circle.r * circle.r * fanAngle / 2.0;
			}
		}
		area += subArea;
	}

	return area;
}

// https://blog.csdn.net/elicococoo/article/details/41653687
double Geom::intersectArea(const Circle & circle1, const Circle & circle2)
{
	double dis = distance(circle1.c, circle2.c);
	// no intersect
	if (dis > circle1.r + circle2.r)
		return 0;
	else if (dis < fabs(circle1.r - circle2.r))
	{
		// 包含关系
		if (circle1.r < circle2.r)
			return circle1.getArea();
		else
			return circle2.getArea();
	}

	double angle1 = acos((circle1.r * circle1.r + dis * dis - circle2.r * circle2.r) / (2 * dis * circle1.r));
	double angle2 = acos((circle2.r * circle2.r + dis * dis - circle1.r * circle1.r) / (2 * dis * circle2.r));
	return circle1.r * circle1.r * angle1 + circle2.r * circle2.r * angle2 - sin(angle1) * circle1.r * dis;
}

// https://thecodeway.com/blog/?p=932
vector<Point2f> Geom::intersectPoints(const Circle & circle, const Point2f & p1, const Point2f & p2)
{
	double A = pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
	double B = 2 * ((p2.x - p1.x) * (p1.x - circle.c.x) + (p2.y - p1.y) * (p1.y - circle.c.y));
	double C = pow(circle.c.x, 2) + pow(circle.c.y, 2) + p1.x * p1.x + p1.y * p1.y - 2 * (circle.c.x * p1.x + circle.c.y * p1.y) - circle.r * circle.r;
	double b24ac = B * B - 4 * A * C;
	if (b24ac < -EPS)
		return {};

	double u1 = (-B + sqrt(fabs(b24ac))) / (2 * A);
	double u2 = (-B - sqrt(fabs(b24ac))) / (2 * A);

	if ((u1 < 0 || u1 >1) && (u2 < 0 || u2 >1)) // no intersect
		return {};
	if (u1 == u2 && u1 >= 0 && u1 <= 1)
	{
		// one intersect point
		return { p1 + u1 * (p2 - p1) };
	}
	if ((u1 >= 0 && u1 <= 1) && (u2 < 0 || u2 >1))
	{
		// one intersect point
		return { p1 + u1 * (p2 - p1) };
	}
	if ((u2 >= 0 && u2 <= 1) && (u1 < 0 || u1 >1))
	{
		// one intersect point
		return { p1 + u2 * (p2 - p1) };
	}
	if ((u1 >= 0 && u1 <= 1) && (u2 >= 0 && u2 <= 1))
	{
		// two intersect points
		if(u1 < u2)
			return{ p1 + u1 * (p2 - p1) , p1 + u2 * (p2 - p1) };
		else
			return{ p1 + u2 * (p2 - p1) , p1 + u1 * (p2 - p1) };
	}

	return {};
}

// https://www.cnblogs.com/AOQNRMGYXLMV/p/5098304.html
vector<Point2f> Geom::intersectPoints(const Circle & circle1, const Circle & circle2)
{
	double dis = distance(circle1.c, circle2.c);
	// no intersect
	if (dis > circle1.r + circle2.r ||
		dis < fabs(circle1.r - circle2.r))
		return vector<Point2f>(); 
	else if (fabs(dis - circle1.r - circle2.r) < EPS || 
		fabs(dis - fabs(circle1.r - circle2.r)) < EPS)
	{
		// one intersection
		Vector2f v = circle2.c - circle1.c;
		v.normalize();
		return vector<Point2f>{circle1.c + v * circle1.r};
	}

	double a = 2 * circle1.r * (circle1.c.x - circle2.c.x);
	double b = 2 * circle1.r * (circle1.c.y - circle2.c.y);
	double c = circle2.r * circle2.r - circle1.r * circle1.r - pow((circle1.c.x - circle2.c.x), 2) - pow((circle1.c.y - circle2.c.y), 2);

	double p = a * a + b * b;
	double q = -a * c * 2;
	double r = c * c - b * b;
	double delta = sqrt(q * q - p * r * 4);
	double cosa = (-q + delta) / (2 * p);
	double cosb = (-q - delta) / (2 * p);
	double sina = sqrt(1 - cosa * cosa);
	double sinb = sqrt(1 - cosb * cosb);
	Point2f p1(circle1.c.x + circle1.r * cosa, circle1.c.y + circle1.r * sina);
	Point2f p2(circle1.c.x + circle1.r * cosb, circle1.c.y + circle1.r * sinb);
	if (fabs(distance(p1, circle2.c) - circle2.r) > EPS)
		p1.y = circle1.c.y - circle1.r * sina;
	if (fabs(distance(p2, circle2.c) - circle2.r) > EPS)
		p2.y = circle1.c.y - circle1.r * sinb;
	if(p1 == p2)
		p1.y = circle1.c.y - circle1.r * sina;
	return vector<Point2f>{p1, p2};
}

vector<Point2f> Geom::intersectPoints(const Ellipse &e, const Point2f &p1, const Point2f &p2)
{
	// approximate
	vector<Point2f> points;
	Vector2f dir = p2 - p1;
	float step = 0.5;
	float lineLen = dir.length();
	//float stepNum = dir.length() / step;
	dir.normalize();
	Point2f lastPoint = p1;
	bool lastPointInE = IsInEllipse(e, p1);
	for (float len = step; len < lineLen; len += step)
	{
		Point2f p = p1 + len * dir;
		bool in = IsInEllipse(e, p);
		if(in != lastPointInE)
			points.push_back((lastPoint + p) / 2.0);
		lastPoint = p;
		lastPointInE = in;
	}
	bool in = IsInEllipse(e, p2);
	if (in != lastPointInE)
		points.push_back((lastPoint + p2) / 2.0);

	return points;
}

vector<Point2f> Geom::intersectPoints(const Polygon &polygon, const Point2f &p1, const Point2f &p2)
{
	vector<Point2f> points;
	for (int i = 0; i <= polygon.size(); ++i)
	{

	}
	return points;
}

// return 1 if p on the left of line l1-l2
// 0 if p is on the line
// -1 if p on the right
int Geom::isLeft(const Point2f & l1, const Point2f & l2, const Point2f & p)
{
	Vector2f v1 = l2 - p;
	Vector2f v2 = l2 - l1;
	double l = v1.x * v2.y - v1.y * v2.x;
	if (l > 0)
		return 1;
	if (l < 0)
		return -1;
	return 0;
}

int Geom::isLeft(const Vector2f & v1, const Vector2f & v2)
{
	int l = v1.x * v2.y - v1.y * v2.x;
	if (l > 0)
		return 1;
	if (l < 0)
		return -1;
	return 0;
}

double Geom::distance(const Line & line, const Point2f & p)
{
	return abs(cross(line.dir, p - line.p0) / line.dir.length());
}

double Geom::distance(const Segment & segment, const Point2f & p)
{
	Vector2f v = segment.p1 - segment.p0;
	Vector2f v1 = p - segment.p0;
	if (v.dot(v1) <= 0)
		return distance(p, segment.p0);
	Vector2f v2 = p - segment.p1;
	if (v.dot(v2) >= 0)
		return distance(p, segment.p1);
	return distance(Line(segment.p0, v), p);
}

vector<Polygon> Geom::triangulation(Polygon polygon)
{
	if (polygon[0] == polygon[polygon.size() - 1])
		polygon.points.erase(polygon.points.end() - 1);
	vector<Polygon> triangles;
	tri_recursive(polygon, triangles);

	return triangles;
}

// http://critterai.org/projects/nmgen_study/polygen.html
vector<Polygon> Geom::convexDecomposition(const Polygon& polygon)
{
	vector<Polygon> newPolygons = triangulation(polygon);
	while (mergeLongestSharedEdge(newPolygons))
		;
	return newPolygons;
}

pair<Eigen::MatrixXd, Eigen::VectorXd> Geom::MinVolEllipse(Eigen::MatrixXd P, double tolerance)
{
	//----------------------- Solving the Dual problem -----------------------
	//	 --------------------------------
	//	 data points
	//	 ----------------------------------
	int d = P.rows();
	int N = P.cols();
	Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(d + 1, N);
	Q.block(0, 0, d, N) = P.block(0, 0, d, N);
	Q.row(d) = Eigen::MatrixXd::Ones(1, N);

	// initializations
	//----------------------------------
	int count = 1;
	double err = 1.0;
	Eigen::MatrixXd u = (1.0 / N) * Eigen::MatrixXd::Ones(N, 1);

	// Khachiyan Algorithm
	//----------------------------------
	while (err > tolerance)
	{
		Eigen::MatrixXd X = Q * u.asDiagonal() * Q.transpose();
		Eigen::VectorXd M = (Q.transpose() * X.inverse() * Q).diagonal();
		Eigen::Index j;
		double maximum = M.maxCoeff(&j);
		double stepSize = (maximum - d - 1) / ((d + 1) * (maximum - 1));
		Eigen::VectorXd new_u = (1 - stepSize) * u;
		new_u[j] = new_u[j] + stepSize;
		count++;
		err = (new_u - u).norm();
		u = new_u;
	}

	// ----------------- Computing the Ellipse parameters -----------------
	//	% Finds the ellipse equation in the 'center form':
	// (x - c)' * A * (x-c) = 1
	// It computes a dxd matrix 'A' and a d dimensional vector 'c' as the center
	// of the ellipse.
	Eigen::MatrixXd U = u.asDiagonal();
	Eigen::MatrixXd A = (1.0 / d) * (P * U * P.transpose() - (P * u) * (P * u).transpose()).inverse();
	Eigen::VectorXd c = P * u;

	return { A, c };
}

Ellipse Geom::minEllipseCoverage(const vector<Point2f>& points)
{
	Eigen::MatrixXd P(2, points.size());
	for (int j = 0; j < points.size(); ++j)
	{
		P(0, j) = points[j].x;
		P(1, j) = points[j].y;
	}
	pair<Eigen::MatrixXd, Eigen::VectorXd> Ac = MinVolEllipse(P, 0.001);
	Eigen::MatrixXd A = Ac.first;
	Eigen::Vector2d c = Ac.second;
	//cout << A << endl;
	//cout << c << endl << endl;

	// 求特征值特征向量
	Eigen::EigenSolver<Eigen::MatrixXd> eigenSolver(A, true);
	Eigen::VectorXcd eigenValues = eigenSolver.eigenvalues();
	Eigen::MatrixXcd eigenVectors = eigenSolver.eigenvectors();
	//cout << eigenValues << endl;
	//cout << eigenVectors << endl;

	Ellipse e;
	e.center.x = c[0];
	e.center.y = c[1];
	e.a = 1 / sqrt(eigenValues[0].real());
	e.b = 1 / sqrt(eigenValues[1].real());
	//cout << "s:" << eigenValues.size() << endl;
	//cout << eigenVectors(0,0) << "----"<< eigenVectors(1,0)<< endl;
	e.angle = 180.0 * atan(eigenVectors(1, 0).real() / eigenVectors(0, 0).real()) / PI;
	return e;
}

// Generate random simple polygon
// https://observablehq.com/@tarte0/generate-random-simple-polygon
// input: 
// int: edge num
// Point2f: polygon center
// int: max Radius from center
Polygon Geom::generateRandomPolygon(int edgeNum, Point2f center, int maxRadius, unsigned int seed)
{
	if (seed == 0)
		srand((unsigned)time(NULL));
	else
		srand(seed);
	vector<Point2f> points;
	Point2f realCenter(0, 0);
	for (int i = 0; i < edgeNum; ++i)
	{
		int x = rand() % (2 * maxRadius) - maxRadius;
		int y = rand() % (2 * maxRadius) - maxRadius;
		Point2f p(x, y);
		realCenter = realCenter + p;
		points.push_back(p);
	}
	realCenter = realCenter / edgeNum;
	sort(points.begin(), points.end(), [realCenter](const Point2f& p1, const Point2f& p2) {
		double angle1, angle2;
		angle1 = angle(Vector2f(1, 0), p1 - realCenter);
		if (p1.y < realCenter.y)
			angle1 = 2 * PI - angle1;
		else if (p1.y == realCenter.y)
		{
			if (p1.x >= realCenter.x)
				angle1 = 0;
			else
				angle1 = PI;
		}
		angle2 = angle(Vector2f(1, 0), p2 - realCenter);
		if (p2.y < realCenter.y)
			angle2 = 2 * PI - angle2;
		else if (p2.y == realCenter.y)
		{
			if (p2.x >= realCenter.x)
				angle2 = 0;
			else
				angle2 = PI;
		}
		return angle1 < angle2;
	});

	for (int i = 0; i < points.size(); ++i)
		points[i] = points[i] + center;

	return Polygon(points);
}

// https://ieeexplore.ieee.org/document/5540225/citations
vector<Polygon> Geom::ConvexShapeDecomposition(const Polygon &polygon, int t, double eps)
{
	vector<Point2f> points;
	float pointDis = 1.0;
	for (int i = 0; i < polygon.size(); ++i)
	{
		Point2f p1 = polygon[i];
		Point2f p2 = polygon[(i + 1) % polygon.size()];
		Vector2f v = p2 - p1;
		int pointNum = v.length() / pointDis;
		v.normalize();
		for (float j = 0; j < pointNum; ++j)
		{
			Point2f p = p1 + v * j * pointDis;
			points.push_back(p);
		}
	}

	// 1. 
	vector<Vector2f> morseDirs;
	double angleIterval = 2.0 * PI / t;
	for (int i = 0; i < t; ++i)
	{
		double theta = PI / 2.0 + i * angleIterval;
		morseDirs.push_back(Vector2f(cos(theta), sin(theta)));
	}

	// 2
	set<pair<Point2f, Point2f>> mutexPairs;
	set< pair<Point2f, Point2f>> cuts;
	for (int i = 0; i < t; ++i)
	{
		
	}

	return vector<Polygon>();
}

vector<int> Geom::ShorestPathDijkstra(const Graph &graph, int beginIdx, int endIdx)
{
	int vertexNum = graph.getVertexNum();
	assert(beginIdx >= 0 && beginIdx < vertexNum && endIdx >= 0 && endIdx < vertexNum);

	vector<float> pathLengths(vertexNum, 0);
	vector<bool> visited(vertexNum, false);
	vector<vector<int>> paths(vertexNum);

	//首先初始化我们的dis数组
	for (int i = 0; i < vertexNum; i++)
	{
		pathLengths[i] = graph.getWeight(beginIdx, i);
		paths[i].push_back(beginIdx);
		paths[i].push_back(i);
	}
	//设置起点的到起点的路径为0
	pathLengths[beginIdx] = 0;
	visited[beginIdx] = true;

	int count = 1;
	//计算剩余的顶点的最短路径（剩余this->vexnum-1个顶点）
	while (count != vertexNum) {
		//temp用于保存当前dis数组中最小的那个下标
		//min记录的当前的最小值
		int temp = 0;
		float min = INFINITY;
		for (int i = 0; i < vertexNum; i++) {
			if (!visited[i] && pathLengths[i] < min) {
				min = pathLengths[i];
				temp = i;
			}
		}
		//cout << temp + 1 << "  "<<min << endl;
		//把temp对应的顶点加入到已经找到的最短路径的集合中
		visited[temp] = true;
		++count;
		for (int i = 0; i < vertexNum; i++) {
			//注意这里的条件arc[temp][i]!=INT_MAX必须加，不然会出现溢出，从而造成程序异常
			if (!visited[i] && graph.getWeight(temp, i) != INFINITY && 
				(pathLengths[temp] + graph.getWeight(temp, i)) < pathLengths[i]) {
				//如果新得到的边可以影响其他为访问的顶点，那就就更新它的最短路径和长度
				pathLengths[i] = pathLengths[temp] + graph.getWeight(temp, i);
				paths[i] = paths[temp];
				paths[i].push_back(i);
			}
		}
	}

	return paths[endIdx];
}

void Geom::tri_recursive(Polygon polygon, vector<Polygon>& triangles)
{
	if (polygon.is_empty())
		return;
	if (polygon.size() == 3)
	{
		triangles.push_back(polygon);
		return;
	}
	double minSquqreDis = 9999999;
	int idx;

	// select the shorest and valid edge
	for (int i = 0; i < polygon.size(); ++i)
	{
		int nextPoitnIdx = (i + 2) % polygon.size();
		Point2f p1 = polygon[i];
		Point2f p2 = polygon[nextPoitnIdx];
		// edge1
		double dis = pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
		if (dis < minSquqreDis &&
			edge_valid(polygon, i, nextPoitnIdx))
		{
			minSquqreDis = dis;
			idx = i;
		}
	}

	Polygon tri;
	tri.push_back(polygon[idx]);
	tri.push_back(polygon[(idx + 1) % polygon.size()]);
	tri.push_back(polygon[(idx + 2) % polygon.size()]);
	triangles.push_back(tri);

	polygon.points.erase(polygon.points.begin() + (idx + 1) % polygon.size());

	tri_recursive(polygon, triangles);
}

bool Geom::edge_valid(const Polygon & polygon, int idx1, int idx2)
{
	int preIdx = idx1 == 0 ? polygon.size() - 1 : idx1 - 1;
	Vector2f v1 = polygon[preIdx] - polygon[idx1];
	Vector2f v2 = polygon[(idx1 + 1) % polygon.size()] - polygon[idx1];
	Vector2f edge = polygon[idx2] - polygon[idx1];
	float angle = antiClockAngle(v1, v2);
	float angle2 = antiClockAngle(v1, edge);
	bool valid = true;;
	if (angle2 > angle)
		valid = false;
	if (!polygon.isClockwise())
		valid = !valid;
	return valid;
}

float Geom::antiClockAngle(Vector2f v1, Vector2f v2)
{
	v1.normalize();
	v2.normalize();
	float angle = acos(v1.dot(v2));
	if (v1.x * v2.y - v1.y * v2.x < 0)
		angle = 2 * PI - angle;
	return angle;
}

// find two polygons with the longest shared edge and merge them
bool Geom::mergeLongestSharedEdge(vector<Polygon>& polygons)
{
	// 1. find all shared edges and sort by length
	for(Polygon &polygon : polygons)
		if(polygon.isClockwise())
			reverse(polygon.points.begin(), polygon.points.end());
	vector<SharedEdge> sharedEdges;
	for (int i = 0; i < polygons.size(); ++i)
	{
		for (int j = i + 1; j < polygons.size(); ++j)
		{
			SharedEdge edge;
			bool found = shared_edge(polygons[i], polygons[j], edge);
			if (!found)
				continue;
			edge.polyIdx1 = i;
			edge.polyIdx2 = j;
			sharedEdges.push_back(edge);
			//double dis = pow(edge.first.x() - edge.second.x(), 2) + pow(edge.first.y() - edge.second.y(), 2);
			//if (dis > maxSquareDis)
			//{
			//	maxSquareDis = dis;
			//	longestEdge = edge;
			//}
		}
	}

	sort(sharedEdges.begin(), sharedEdges.end(), [](SharedEdge & edge1, SharedEdge & edge2) {
		double dis1 = pow(edge1.p1.x - edge1.p2.x, 2) + pow(edge1.p1.y - edge1.p2.y, 2);
		double dis2 = pow(edge2.p1.x - edge2.p2.x, 2) + pow(edge2.p1.y - edge2.p2.y, 2);
		return dis1 > dis2;
	});

	// 2. find the longest valid shared edge
	bool hasValidEdge = false;
	for (int i = 0; i < sharedEdges.size(); ++i)
	{
		Polygon polygon1 = polygons[sharedEdges[i].polyIdx1];
		Polygon polygon2 = polygons[sharedEdges[i].polyIdx2];
		if (mergeEdgeValid(polygon1, polygon2, sharedEdges[i]))
		{
			// found
			Polygon mergedP = mergePolygon(polygon1, polygon2, sharedEdges[i]);
			if (sharedEdges[i].polyIdx1 > sharedEdges[i].polyIdx2)
			{
				polygons.erase(polygons.begin() + sharedEdges[i].polyIdx1);
				polygons.erase(polygons.begin() + sharedEdges[i].polyIdx2);
			}
			else
			{
				polygons.erase(polygons.begin() + sharedEdges[i].polyIdx2);
				polygons.erase(polygons.begin() + sharedEdges[i].polyIdx1);
			}
			polygons.push_back(mergedP);
			hasValidEdge = true;
			break;
		}
	}

	return hasValidEdge;
}


Polygon Geom::mergePolygon(Polygon polygon1, Polygon polygon2, SharedEdge & edge)
{
	Polygon mergedPolygon;
	if ((edge.edge1Idx1 + 1) % polygon1.size() == edge.edge1Idx2)
	{
		for (int i = edge.edge1Idx1; i != edge.edge1Idx2; i = (i + polygon1.size() - 1) % polygon1.size())
			mergedPolygon.push_back(polygon1[i]);
	}
	else
		for (int i = edge.edge1Idx1; i != edge.edge1Idx2; i = (i + 1) % polygon1.size())
			mergedPolygon.push_back(polygon1[i]);

	if ((edge.edge2Idx2 + 1) % polygon2.size() == edge.edge2Idx1)
		for (int i = edge.edge2Idx2; i != edge.edge2Idx1; i = (i + polygon2.size() - 1) % polygon2.size())
			mergedPolygon.push_back(polygon2[i]);
	else
		for (int i = edge.edge2Idx2; i != edge.edge2Idx1; i = (i + 1) % polygon2.size())
			mergedPolygon.push_back(polygon2[i]);

	return mergedPolygon;
}

vector<Point2f> Geom::ShorestPath(Polygon polygon, int beginIdx, int endIdx)
{
	assert(beginIdx >= 0 && beginIdx < polygon.size() && endIdx >= 0 && endIdx < polygon.size());

	Graph graph(polygon.size());
	for (int i = 0; i < polygon.size(); ++i)
	{
		for (int j = 0; j < polygon.size(); ++j)
		{
			;
		}
	}

	return vector<Point2f>();
}

// if polygon1 and polygon has shared edge
bool Geom::shared_edge(Polygon polygon1, Polygon polygon2, SharedEdge &edge)
{
	for (int i = 0; i < polygon1.size(); ++i)
	{
		Point2f p1 = polygon1[i];
		for (int j = 0; j < polygon2.size(); ++j)
		{
			if (p1 == polygon2[j])
			{
				int preIdx = (j + 1) % polygon2.size();
				int lastIdx = j == 0 ? polygon2.size() - 1 : j - 1;
				if (polygon1[(i + 1) % polygon1.size()] == polygon2[preIdx])
				{
					edge.p1 = p1;
					edge.edge1Idx1 = i;
					edge.edge1Idx2 = (i + 1) % polygon1.size();
					edge.p2 = polygon2[preIdx];
					edge.edge2Idx1 = j;
					edge.edge2Idx2 = preIdx;
					return true;
				}
				if (polygon1[(i + 1) % polygon1.size()] == polygon2[lastIdx])
				{
					edge.p1 = p1;
					edge.edge1Idx1 = i;
					edge.edge1Idx2 = (i + 1) % polygon1.size();
					edge.p2 = polygon2[lastIdx];
					edge.edge2Idx1 = j;
					edge.edge2Idx2 = lastIdx;
					return true;
				}
			}
		}
	}
	return false;
}

// if two polygon can be merged
// make sure polygon1 and polygon2 are counterclockwise
bool Geom::mergeEdgeValid(Polygon polygon1, Polygon polygon2, SharedEdge & edge)
{
	if (polygon1.isClockwise() || polygon2.isClockwise())
	{
		cout << "Please input counterclockwise polygon" << endl;
		return false;
	}
	float fineAngle = 165; //180;
	bool valid1 = false, valid2 = false;
	int Aminus1, Aplus1;
	Point2f p0, p1;
	// 1
	if ((edge.edge1Idx1 + 1) % polygon1.size() == edge.edge1Idx2)
	{
		Aplus1 = edge.edge1Idx1 == 0 ? polygon1.size() - 1 : edge.edge1Idx1 - 1;
		Aminus1 = (edge.edge2Idx1 + 1) % polygon2.size();
		p0 = polygon2[Aminus1];
		p1 = polygon1[Aplus1];
	}
	else
	{
		Aminus1 = (edge.edge1Idx1 + 1) % polygon1.size();
		Aplus1 = edge.edge2Idx1 == 0 ? polygon2.size() - 1 : edge.edge2Idx1 - 1;
		p0 = polygon1[Aminus1];
		p1 = polygon2[Aplus1];
	}
	Point2f p = edge.p1;
	Vector2f referenceLine = p1 - p0;
	Vector2f v1 = p0 - p;
	Vector2f v2 = p1 - p;
	if (isLeft(referenceLine, Vector2f(p - p0)) < 0 && 
		angle(v1, v2, false) < fineAngle)
		return false;

	// 2
	if ((edge.edge1Idx2 + 1) % polygon1.size() == edge.edge1Idx1)
	{
		Aplus1 = edge.edge1Idx2 == 0 ? polygon1.size() - 1 : edge.edge1Idx2 - 1;
		Aminus1 = (edge.edge2Idx2 + 1) % polygon2.size();
		p0 = polygon2[Aminus1];
		p1 = polygon1[Aplus1];
	}
	else
	{
		Aminus1 = (edge.edge1Idx2 + 1) % polygon1.size();
		Aplus1 = edge.edge2Idx2 == 0 ? polygon2.size() - 1 : edge.edge2Idx2 - 1;
		p0 = polygon1[Aminus1];
		p1 = polygon2[Aplus1];
	}
	p = edge.p2;
	referenceLine = p1 - p0;
	v1 = p0 - p;
	v2 = p1 - p;
	if (isLeft(referenceLine, Vector2f(p - p0)) < 0 &&
		angle(v1, v2, false) < fineAngle)
		return false;
	return true;
}

bool Geom::IsInCircle(const Circle & circle, const Point2f & p)
{
	double dis = distance(circle.c, p);
	return dis < (circle.r + EPS) ? true : false;
}

bool Geom::IsInEllipse(const Ellipse & ellipse, const Point2f & p)
{
	double a = ellipse.a;
	double b = ellipse.b;
	double angle = ellipse.angle / 180.0 * PI;
	double m = ellipse.center.x;
	double n = ellipse.center.y;
	double t1 = pow((p.x - m) * cos(angle) + (p.y - n) * sin(angle), 2);
	double t2 = pow((m - p.x) * sin(angle) + (p.y - n) * cos(angle), 2);
	return t1 / (a * a) + t2 / (b * b) < 1;
}
