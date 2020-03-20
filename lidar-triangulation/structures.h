#pragma once

struct Point {
	double x, y, z;
};


class Triangle {
public:
	Point *p0, *p1, *p2;
	double area; // Signed area

	Triangle(Point *p0, Point *p1, Point *p2) : p0{ p0 }, p1 { p1 }, p2{ p2 } {
		this->area = 0.5f * (-p1->y * p2->x + p0->y * (-p1->x + p2->x) + p0->x * (p1->y - p2->y) + p1->x * p2->y);
	}

	int isInside(Point *p, double &w0, double &w1, double &w2) { // https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
		double sign = area >= 0 ? +1. : -1.;
		w1 = sign * (float)(p0->y * p2->x - p0->x * p2->y + (p2->y - p0->y) * p->x + (p0->x - p2->x) * p->y);
		w2 = sign * (float)(p0->x * p1->y - p0->y * p1->x + (p0->y - p1->y) * p->x + (p1->x - p0->x) * p->y);
		w0 = (2.f * area * sign) - w1 - w2;
		
		return w0 >= 0. && w1 >= 0. && w2 >= 0.;
	}
};
