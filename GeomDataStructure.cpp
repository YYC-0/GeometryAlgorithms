#include "GeomDataStructure.h"
#include <assert.h>

namespace Geometry
{
	Point2f::Point2f(double x_, double y_)
	{
		x = x_;
		y = y_;
	}

	Circle::Circle(Point2f c, double r)
	{
		this->c = c;
		this->r = r;
	}

	Polygon::Polygon(vector<Point2f> points)
	{
		this->points = points;
		updataBoundingBox();
	}

	void Polygon::push_back(Point2f p)
	{
		points.push_back(p);
		updataBoundingBox();
	}

	Point2f &Polygon::operator[](int i)
	{
		if (i < 0 || i>points.size())
			throw "Subscript out of range!";
		return points[i];
	}

	const Point2f &Polygon::operator[](int i) const
	{
		if (i < 0 || i>points.size())
			throw "Subscript out of range!";
		return points[i];
	}

	double Polygon::area()
	{
		double A = 0;
		for (int i = 0; i < points.size(); ++i)
		{
			Point2f p1 = points[i];
			Point2f p2 = points[(i + 1) % points.size()];
			//double a = p1.x * p2.y - p1.y * p2.x; // eqal to below
			double a = (p1.x + p2.x) * (p2.y - p1.y);
			A += a;
		}
		if (isClockwise())
			return -A / 2.0;
		else
			return A / 2.0;
	}

	bool Polygon::isClockwise() const
	{
		if (points.size() < 3)
			return true;
		// find the max y value point and its front and after
		double maxY = numeric_limits<double>::lowest();
		int idx = -1;
		for (int i = 0; i < points.size(); ++i)
			if (points[i].y > maxY)
			{
				maxY = points[i].y;
				idx = i;
			}
		int front = (idx == 0 ? points.size() - 1 : idx - 1);
		int after = (idx == points.size() - 1 ? 0 : idx + 1);
		Vector2f v1 = points[idx] - points[front];
		Vector2f v2 = points[after] - points[idx];
		double c = v1.cross(v2);
		return c < 0;
	}

	void Polygon::updataBoundingBox()
	{
		double up = numeric_limits<double>::lowest();
		double down = numeric_limits<double>::max();
		double left = numeric_limits<double>::max();
		double right = numeric_limits<double>::lowest();
		for (int i = 0; i < points.size(); ++i)
		{
			if (points[i].x > right)
				right = points[i].x;
			if (points[i].x < left)
				left = points[i].x;
			if (points[i].y > up)
				up = points[i].y;
			if (points[i].y < down)
				down = points[i].y;
		}
		boundingBox = Rectangle(Point2f(left, down), Point2f(right, up));
	}

	int Polygon::isLeft(const Vector2f &v1, const Vector2f &v2)
	{
		int l = v1.x * v2.y - v1.y * v2.x;
		if (l > 0)
			return 1;
		if (l < 0)
			return -1;
		return 0;
	}

	Vector2f::Vector2f(double x_, double y_)
	{
		x = x_;
		y = y_;
	}

	Vector2f::Vector2f(const Point2f &p)
	{
		x = p.x;
		y = p.y;
	}

	void Vector2f::normalize()
	{
		double len = length();
		x /= len;
		y /= len;
	}

	ostream &operator<<(ostream &out, Point2f &p)
	{
		out << "x: " << p.x << "  y: " << p.y;
		return out;
	}

	ostream &operator<<(ostream &out, Vector2f &v)
	{
		out << "x: " << v.x << "  y: " << v.y;
		return out;
	}

	ostream &operator<<(ostream &out, Circle &c)
	{
		out << "c: " << c.c << endl;
		out << "r: " << c.r;
		return out;
	}


	Line::Line(Point2f p_, Vector2f v_) :
		p0(p_),
		dir(v_)
	{
	}

	Segment::Segment(Point2f p0_, Point2f p1_) :
		p0(p0_),
		p1(p1_)
	{
	}

	Rectangle::Rectangle(Point2f downLeft_, Point2f topRight_) :
		downLeft(downLeft_),
		topRight(topRight_)
	{
		width = topRight.x - downLeft.x;
		height = topRight.y - downLeft.y;
	}

	Ellipse::Ellipse(Point2f center_, double a_, double b_, double angle_) :
		center(center_),
		a(a_),
		b(b_),
		angle(angle_)
	{
	}

	Graph::Graph(int vertexNumber) : vertexNum(vertexNumber)
	{
		weights.resize(vertexNum);
		for (int i = 0; i < weights.size(); ++i)
			weights[i].resize(vertexNum, INFINITY);
	}

	void Graph::addEdge(int vertex1, int vertex2, float weight)
	{
		assert(vertex1 >= 0 && vertex1 < vertexNum &&vertex2 >= 0 && vertex2 < vertexNum);
		if (weights[vertex1][vertex2] != INFINITY)
		{
			cout << "Edge " << vertex1 << " - " << vertex2 << " already existed!" << endl;
		}
		weights[vertex1][vertex2] = weight;
	}

	void Graph::print() const
	{
		cout << '\t';
		for (int i = 0; i < vertexNum; ++i)
			cout << i << '\t';
		cout << endl;
		for (int i = 0; i < vertexNum; ++i)
		{
			cout << i << '\t';
			for (int j = 0; j < vertexNum; ++j)
			{
				if (weights[i][j] == INFINITY)
					cout << "¡Þ";
				else
					cout << weights[i][j];
				cout << '\t';
			}
			cout << endl;
		}
	}

}