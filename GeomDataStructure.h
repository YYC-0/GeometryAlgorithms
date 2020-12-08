#pragma once
#include <cmath>
#include <vector>
#include <iostream>
using namespace std;
const double PI = 3.14159265358979323846;
class Point2f
{
public:
	Point2f(double x_ = 0, double y_ = 0);

	friend Point2f operator + (const Point2f& p1, const Point2f& p2) {
		return Point2f(p1.x + p2.x, p1.y + p2.y);
	}
	friend Point2f operator - (const Point2f& p1, const Point2f& p2) {
		return Point2f(p1.x - p2.x, p1.y - p2.y);
	}
	friend Point2f operator * (const Point2f& p1, double a) {
		return Point2f(p1.x * a, p1.y * a);
	}
	friend Point2f operator * (double a, const Point2f& p1) {
		return Point2f(p1.x * a, p1.y * a);
	}
	friend Point2f operator / (const Point2f& p1, double a) {
		return Point2f(p1.x / a, p1.y / a);
	}
	friend bool operator == (const Point2f& p1, const Point2f& p2) {
		return (p1.x == p2.x && p1.y == p2.y);
	}
	friend ostream & operator << (ostream& out, Point2f& p);

	double squareDis(const Point2f& p) const {
		return pow(x - p.x, 2) + pow(y - p.y, 2);
	}
	double distance(const Point2f& p) const {
		return sqrt(squareDis(p));
	}
	
	double x;
	double y;
};

class Vector2f
{
public:
	Vector2f(double x_ = 0, double y_ = 0);
	Vector2f(const Point2f& p);

	friend Vector2f operator + (const Vector2f& v1, const Vector2f& v2) {
		return Vector2f(v1.x + v2.x, v1.y + v2.y);
	}
	friend Vector2f operator - (const Vector2f& v1, const Vector2f& v2) {
		return Vector2f(v1.x - v2.x, v1.y - v2.y);
	}
	friend Point2f operator + (const Point2f& p, const Vector2f& v) {
		return Point2f(p.x + v.x, p.y + v.y);
	}
	friend Point2f operator - (const Point2f& p, const Vector2f& v) {
		return Point2f(p.x - v.x, p.y - v.y);
	}
	friend Vector2f operator * (const Vector2f& v, double a) {
		return Vector2f(v.x * a, v.y * a);
	}
	friend Vector2f operator * (double a, const Vector2f &v) {
		return Vector2f(v.x * a, v.y * a);
	}
	friend Vector2f operator / (const Vector2f& v1, double a) {
		return Vector2f(v1.x / a, v1.y / a);
	}
	friend bool operator == (const Vector2f& v1, const Vector2f& v2) {
		return (v1.x == v2.x && v1.y == v2.y);
	}
	friend ostream & operator << (ostream& out, Vector2f& v);
	double dot(const Vector2f& v) const {
		return x * v.x + y * v.y;
	}
	double cross(const Vector2f& p) const {
		return x * p.y - y * p.x;
	}
	double length() const {
		return sqrt(pow(x, 2) + pow(y, 2));
	}
	void normalize();

	double x;
	double y;
};

class Triangle2f
{
public:
	Triangle2f() = default;
	Triangle2f(Point2f p0, Point2f p1, Point2f p2) {
		p[0] = p0;
		p[1] = p1;
		p[2] = p2;
	}

	double area() {
		return fabs(cross(p[1] - p[0], p[2] - p[0]) / 2.0);
	}

	Point2f p[3];

private:
	double cross(const Point2f& p1, const Point2f& p2) {
		return p1.x * p2.y - p1.y * p2.x;
	}
};


class Circle
{
public:
	Circle(Point2f c = Point2f(0, 0), double r = 0);
	friend ostream & operator << (ostream& out, Circle& c);
	double getArea() const {
		return PI * r * r;
	}

	Point2f c; // center
	double r;   // radius

private:
	double area;
};

// Rectangle, defined by point downleft and top right
class Rectangle
{
public:
	Rectangle(Point2f downLeft_ = Point2f(0, 0), Point2f topRight_ = Point2f(0, 0));

	Point2f downLeft;
	Point2f topRight;
	double width;	// ³¤
	double height;	// ¿í
};

// simple polygon
class Polygon
{
public:
	Polygon() = default;
	Polygon(vector<Point2f> points);

	size_t size() const { return points.size(); }
	bool is_empty() const { return points.size() == 0; }
	void push_back(Point2f p);
	Point2f& operator[] (int i);
	const Point2f& operator[] (int i) const;
	Rectangle getBoundingBox() const { return boundingBox; }
	double area();

	bool isClockwise() const;
	void updataBoundingBox();

	vector<Point2f> points;

private:
	Rectangle boundingBox;
	int isLeft(const Vector2f &v1, const Vector2f &v2);
};

// Line, defined by a point and a direction
class Line
{
public:
	Line(Point2f p = Point2f(0,0), Vector2f v = Vector2f(0,0));

	Point2f p0;
	Vector2f dir;
};

// Segment
class Segment
{
public:
	Segment(Point2f p0 = Point2f(0, 0), Point2f p1 = Point2f(0, 0));

	Point2f p0;
	Point2f p1;
};

class Ellipse
{
public:
	Ellipse(Point2f center_ = Point2f(0, 0), double a_ = 0, double b_ = 0, double angle_ = 0);

	Point2f center;
	double a;
	double b;
	double angle; // [0,360) anticlock wise
};