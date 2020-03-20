#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include <iostream>
#include <math.h>
#include "structures.h"
#include "lasreader.hpp"
#include "laswriter.hpp"

#define DEBUG true

inline int M1(const int& x) { return (x + 1) % 3; }
inline int M2(const int& x) { return (x + 2) % 3; }

int main() {
	// Read LiDAR data
	LASreadOpener lasreadopener;
	lasreadopener.set_file_name("GK_462_100.laz");
	LASreader* lasreader = lasreadopener.open();

	int numPoints = 10000; // (int)lasreader->npoints;
	Point* points = new Point[numPoints];

	for (int i = 0; i < numPoints && lasreader->read_point(); ++i) {
		// Convert from centimeters to meters
		points[i].x = (double)lasreader->point.X / 100.;
		points[i].y = (double)lasreader->point.Y / 100.;
		points[i].z = (double)lasreader->point.Z / 100.;
	}

	lasreader->close();
	delete lasreader;

	for (int i = 0; i < numPoints; ++i) {

	}

	// Get min, max and span
	double xMin = points[0].x, xMax = points[0].x;
	double yMin = points[0].y, yMax = points[0].y;
	double zMin = points[0].z, zMax = points[0].z;
	for (int i = 1; i < numPoints; ++i) {
		if (points[i].x < xMin) xMin = points[i].x;
		else if (points[i].x > xMax) xMax = points[i].x;
		if (points[i].y < yMin) yMin = points[i].y;
		else if (points[i].y > yMax) yMax = points[i].y;
		if (points[i].z < zMin) zMin = points[i].z;
		else if (points[i].z > zMax) zMax = points[i].z;
	}
	double xSpan = xMax - xMin;
	double ySpan = yMax - yMin;

	// Shift points
	for (int i = 0; i < numPoints; ++i) {
		points[i].x -= xMin;
		points[i].y -= yMin;
		points[i].z -= zMin;
	}
	xMax -= xMin; yMax -= yMin; zMax -= zMin;
	xMin = 0.; yMin = 0.; zMin = 0.;

	// Create bounding triangle
	double hBound = (tan(M_PI / 3.) * xSpan / 2.) + ySpan;
	double wBound = 2. * hBound / tan(M_PI / 3.);
	Point p0Bound = { xMin - (wBound - xSpan) / 2. - 1., yMin - 1., 0. };
	Point p1Bound = { xMax + (wBound - xSpan) / 2. + 1., yMin - 1., 0. };
	Point p2Bound = { (xMin + xMax) / 2., yMax + hBound - ySpan + 1., 0. };
	Triangle triBound(&p0Bound, &p1Bound, &p2Bound);

	// Test bounding triangle
	if (DEBUG) {
		for (int i = 0; i < numPoints; ++i) {
			double w[3];
			if (!triBound.isInside(&points[i], w) || w[0] == 0.0 || w[1] == 0.0 || w[2] == 0.0) {
				if (w[0] == 0.0 || w[1] == 0.0 || w[2] == 0.0)
					std::cout << "Point on the edge of bounding triangle!" << std::endl;
				else
					std::cout << "Point outside bounding triangle!" << std::endl;

				std::cout << "Point: " << points[i].x << ", " << points[i].y << std::endl;
				std::cout << "Baricentric weights: " << w[0] << ", " << w[1] << ", " << w[2] << std::endl;
				std::cout << "Triangle point 1: " << triBound.p0->x << ", " << triBound.p0->y << std::endl;
				std::cout << "Triangle point 2: " << triBound.p1->x << ", " << triBound.p1->y << std::endl;
				std::cout << "Triangle point 3: " << triBound.p2->x << ", " << triBound.p2->y << std::endl;

				return 1;
			}
		}
	}

	// Delaunay tree (DT)
	Node* root = new Node(&p0Bound, &p1Bound, &p2Bound);

	// Incrementally add points to DT
	for (int iPoint = 0; iPoint < numPoints; ++iPoint) {
		// Find triangle in which the point lies
		Point* p = &points[iPoint];
		Node* n = root;
		double w[3];
		while (n->hasChildren) {
			if (n->child[0] != nullptr && n->child[0]->tri.isInside(p, w)) n = n->child[0];
			else if (n->child[1] != nullptr && n->child[1]->tri.isInside(p, w)) n = n->child[1];
			else if (n->child[2] != nullptr && n->child[2]->tri.isInside(p, w)) n = n->child[2];
			else {
				std::cout << "ERROR: Point not inside any triangle!" << std::endl;
				std::cout << iPoint << std::endl;
				std::cout << w[0] << ", " << w[1] << ", " << w[2] << std::endl;
				return 1;
			}
		}

		// Add point to triangle
		if (n == root || w[0] != 0. && w[1] != 0. && w[2] != 0.)  { // Point inside of triangle
			for (int i = 0; i < 3; ++i) {
				n->child[i] = new Node(p, n->tri.p[M1(i)], n->tri.p[M2(i)]);
			}

			/*n->child[0] = new Node(p, n->tri.p1, n->tri.p2);
			n->child[1] = new Node(p, n->tri.p2, n->tri.p0);
			n->child[2] = new Node(p, n->tri.p0, n->tri.p1);*/

			for (int i = 0; i < 3; ++i) {
				n->child[i]->neighbor[0] = n->neighbor[i];
				n->child[i]->neighbor[1] = n->neighbor[M1(i)];
				n->child[i]->neighbor[2] = n->neighbor[M2(i)];
			}

			/*n->child[0]->neighbor[0] = n->neighbor[0];
			n->child[0]->neighbor[1] = n->child[1];
			n->child[0]->neighbor[2] = n->child[2];

			n->child[1]->neighbor[0] = n->neighbor[1];
			n->child[1]->neighbor[1] = n->child[2];
			n->child[1]->neighbor[2] = n->child[0];

			n->child[2]->neighbor[0] = n->neighbor[2];
			n->child[2]->neighbor[1] = n->child[0];
			n->child[2]->neighbor[2] = n->child[1];*/

			for (int i = 0; i < 3; ++i) {
				n->child[i]->updateNeigbors(n);
			}

		} else { // Point on edge of triangle
			int edge = 0;
			if (w[1] == 0.) edge = 1;
			else if (w[2] == 0.) edge = 2;

			n->split2(p, edge);
		}
		n->hasChildren = true;
	}

	return 0;
}
