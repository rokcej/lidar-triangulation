#pragma once

struct Point {
	double x, y, z;
};


class Triangle {
public:
	Point* p[3];
	Point *p0, *p1, *p2;
	double area; // Signed area

	Triangle(Point* p0, Point* p1, Point* p2) : p{ p0, p1, p2 }, p0 { p0 }, p1{ p1 }, p2{ p2 } {
		this->area = 0.5f * (-p1->y * p2->x + p0->y * (-p1->x + p2->x) + p0->x * (p1->y - p2->y) + p1->x * p2->y);
	}
	Triangle(Triangle* t) : Triangle(t->p0, t->p1, t->p2) {}

	int isInside(Point *p, double *w) { // https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
		double sign = area >= 0 ? +1. : -1.;
		w[1] = sign * (float)(p0->y * p2->x - p0->x * p2->y + (p2->y - p0->y) * p->x + (p0->x - p2->x) * p->y);
		w[2] = sign * (float)(p0->x * p1->y - p0->y * p1->x + (p0->y - p1->y) * p->x + (p1->x - p0->x) * p->y);
		w[0] = (2.f * area * sign) - w[1] - w[2];
		
		return w[0] >= 0. && w[1] >= 0. && w[2] >= 0.;
	}
};

class Node {
public:
	Triangle tri;
	bool hasChildren = false;
	Node* child[3] = { nullptr };
	Node* neighbor[3] = { nullptr };

	Node(Point* p0, Point* p1, Point* p2) : tri{ p0, p1, p2 } {}

	void split2(Point* p, int edge) {
		this->_split2(p, edge);

		if (neighbor[edge] != nullptr) {
			int edgeNeigh = 0;
			if (neighbor[edge]->neighbor[1] == this)
				edgeNeigh = 1;
			else if (neighbor[edge]->neighbor[2] == this)
				edgeNeigh = 2;

			neighbor[edge]->_split2(p, edgeNeigh);

			child[0]->neighbor[2] = neighbor[edge]->child[1];
			child[1]->neighbor[1] = neighbor[edge]->child[0];
			neighbor[edge]->child[0]->neighbor[2] = child[1];
			neighbor[edge]->child[1]->neighbor[1] = child[0];
		}
	}

	void updateNeigbors(Node *parent) { // Make sure neighbours are connected back to this node instead of its parent
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

private:
	void _split2(Point* p, int edge) {
		int i0 = edge, i1 = (edge + 1) % 3, i2 = (edge + 2) % 3;
		child[0] = new Node(p, tri.p[i2], tri.p[i0]);
		child[1] = new Node(p, tri.p[i0], tri.p[i1]);

		child[0]->neighbor[0] = neighbor[i1];
		child[0]->neighbor[1] = child[1];
		child[1]->neighbor[0] = neighbor[i2];
		child[1]->neighbor[2] = child[0];

		child[0]->updateNeigbors(this);
		child[1]->updateNeigbors(this);

		this->hasChildren = true;
	}
};
