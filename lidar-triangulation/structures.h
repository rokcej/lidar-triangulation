#pragma once

#include <iostream>

#define TOL (1e-8) // Precision tolerance

inline int M1(const int& x) { return (x + 1) % 3; } // Plus 1 mod 3
inline int M2(const int& x) { return (x + 2) % 3; } // Plus 2 mod 3

// Mesh face structure
struct Face {
	int v1, v2, v3;

	Face(int v1, int v2, int v3) : v1{ v1 }, v2{ v2 }, v3{ v3 } {}
};

// 3D point structure
struct Point {
	double x = 0., y = 0., z = 0.;
	int idx = -1;

	Point() {}
	Point(double x, double y, double z, int idx) : x{ x }, y{ y }, z{ z }, idx{ idx } {}
};

// Triangle structure
class Triangle {
public:
	Point *p[3];
	double area; // Signed area

	Triangle(Point* p0, Point* p1, Point* p2) : p{ p0, p1, p2 } {
		this->area = 0.5f * (-p1->y * p2->x + p0->y * (-p1->x + p2->x) + p0->x * (p1->y - p2->y) + p1->x * p2->y);
	}

	// Check if point q is inside of the triangle, store barycentric coordinates in w
	inline int isInside(Point *q, double *w) { // https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
		double sign = area >= 0 ? +1. : -1.;
		w[1] = sign * (p[0]->y * p[2]->x - p[0]->x * p[2]->y + (p[2]->y - p[0]->y) * q->x + (p[0]->x - p[2]->x) * q->y);
		w[2] = sign * (p[0]->x * p[1]->y - p[0]->y * p[1]->x + (p[0]->y - p[1]->y) * q->x + (p[1]->x - p[0]->x) * q->y);
		w[0] = (2. * area * sign) - w[1] - w[2];

		if (w[0] < TOL && w[0] > -TOL) w[0] = 0.;
		if (w[1] < TOL && w[1] > -TOL) w[1] = 0.;
		if (w[2] < TOL && w[2] > -TOL) w[2] = 0.;
		
		return w[0] >= 0. && w[1] >= 0. && w[2] >= 0.;
	}

	// Check if point q lies in circumscribed circle of the triangle
	inline bool inCircle(Point* q) { // https://stackoverflow.com/questions/39984709/how-can-i-check-wether-a-point-is-inside-the-circumcircle-of-3-points
		double d0x = p[0]->x - q->x;
		double d0y = p[0]->y - q->y;
		double d1x = p[1]->x - q->x;
		double d1y = p[1]->y - q->y;
		double d2x = p[2]->x - q->x;
		double d2y = p[2]->y - q->y;

		double det = (
			(d0x * d0x + d0y * d0y) * (d1x * d2y - d2x * d1y) -
			(d1x * d1x + d1y * d1y) * (d0x * d2y - d2x * d0y) +
			(d2x * d2x + d2y * d2y) * (d0x * d1y - d1x * d0y)
		);

		return det > -TOL;
	}
};

// Delaunay tree structure
class Node {
public:
	Triangle tri;
	int numChildren = 0;
	Node* child[3] = { nullptr };
	Node* neighbor[3] = { nullptr };
	
	bool processed = false; // Flag for mesh extraction

	Node(Point* p0, Point* p1, Point* p2) : tri{ p0, p1, p2 } {}

	// Split triangle into 2 triangles based on point p, which lies on one of the edges
	void split2(Point* p, int edge) {
		_split2(p, edge);

		if (neighbor[edge] != nullptr) {
			int neighEdge = neighbor[edge]->getEdge(this);
			neighbor[edge]->_split2(p, neighEdge);

			child[0]->neighbor[2] = neighbor[edge]->child[1];
			child[1]->neighbor[1] = neighbor[edge]->child[0];
			neighbor[edge]->child[0]->neighbor[2] = child[1];
			neighbor[edge]->child[1]->neighbor[1] = child[0];
		}
	}

	// Find which edge is connected to n
	int getEdge(Node* n) {
		for (int i = 0; i < 3; ++i) {
			if (neighbor[i] == n)
				return i;
		}
		std::cout << "ERROR: No neighbour found!" << std::endl;
		return -1;
	}

	// Make sure neighbours are connected back to this node instead of its parent
	void updateNeigbors(Node *parent) {
		for (int in = 0; in < 3; ++in) {
			if (neighbor[in] != nullptr) {
				for (int inn = 0; inn < 3; ++inn) {
					if (neighbor[in]->neighbor[inn] == parent) {
						neighbor[in]->neighbor[inn] = this;
						break;
					}
				}
			}
		}
	}

	// Validate edge, flip if it's illegal
	void validate(int edge) {
		if (neighbor[edge] == nullptr)
			return;

		Node* neigh = neighbor[edge];
		int neighEdge = neigh->getEdge(this);
		Point* q = neigh->tri.p[neighEdge];
		if (tri.inCircle(q)) {
			// Flip edge
			Node* n1 = new Node(tri.p[edge], tri.p[M1(edge)], q);
			Node* n2 = new Node(tri.p[edge], q, tri.p[M2(edge)]);

			n1->neighbor[0] = neigh->neighbor[M1(neighEdge)];
			n1->neighbor[1] = n2;
			n1->neighbor[2] = neighbor[M2(edge)];

			n2->neighbor[0] = neigh->neighbor[M2(neighEdge)];
			n2->neighbor[1] = neighbor[M1(edge)];
			n2->neighbor[2] = n1;

			this->numChildren = 2;
			this->child[0] = n1;
			this->child[1] = n2;

			neigh->numChildren = 2;
			neigh->child[0] = n2;
			neigh->child[1] = n1;

			n1->updateNeigbors(this);
			n2->updateNeigbors(this);
			n1->updateNeigbors(neigh);
			n2->updateNeigbors(neigh);

			n1->validate(0);
			n2->validate(0);
		}
	}

private:
	void _split2(Point* p, int edge) {
		this->numChildren = 2;

		int i0 = edge, i1 = (edge + 1) % 3, i2 = (edge + 2) % 3;
		child[0] = new Node(p, tri.p[i2], tri.p[i0]);
		child[1] = new Node(p, tri.p[i0], tri.p[i1]);

		child[0]->neighbor[0] = neighbor[i1];
		child[0]->neighbor[1] = child[1];
		child[1]->neighbor[0] = neighbor[i2];
		child[1]->neighbor[2] = child[0];

		child[0]->updateNeigbors(this);
		child[1]->updateNeigbors(this);
	}
};
