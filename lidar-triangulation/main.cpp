#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include <iostream>
#include <iomanip>
#include <math.h>
#include <random>
#include <vector>
#include <string>
#include <fstream>
#include <iomanip>
#include "structures.h"
// LAStools
#include "lasreader.hpp"
#include "laswriter.hpp"

#define DEBUG false
#define MAX_POINTS 10000000 // Set to 0 to disable limit

// Check if point x lies between points a and b
bool between(Point* a, Point* b, Point* x) {
	double ax = (a->x - x->x) * (a->x - x->x) + (a->y - x->y) * (a->y - x->y);
	double bx = (b->x - x->x) * (b->x - x->x) + (b->y - x->y) * (b->y - x->y);
	double ab = (a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y);

	return sqrt(ax) + sqrt(bx) == sqrt(ab);
}

// Get triangle faces from Delaunay tree
void getFaces(Node *node, std::vector<Face>& faces, Point *points) {
	if (node->processed)
		return;
	node->processed = true;

	if (node->numChildren > 0) {
		getFaces(node->child[0], faces, points);
		getFaces(node->child[1], faces, points);
		if (node->numChildren > 2)
			getFaces(node->child[2], faces, points);
	} else {
		int v1 = node->tri.p[0]->idx;
		int v2 = node->tri.p[1]->idx;
		int v3 = node->tri.p[2]->idx;
		if (v1 >= 0 && v2 >= 0 && v3 >= 0)
			faces.emplace_back(v1, v2, v3);
	}
}

// Save mesh as .obj
void saveObj(std::string filename, Point *points, int numPoints, std::vector<Face>& faces) {
	std::ofstream file;
	file.open(filename);

	// Vertices
	for (int i = 0; i < numPoints; ++i) {
		file << "v " << points[i].x << " " << points[i].y << " " << points[i].z << std::endl;
	}
	file << std::endl;
	// Faces
	for (int i = 0; i < faces.size(); ++i) {
		file << "f " << faces[i].v1 + 1 << " " << faces[i].v2 + 1 << " " << faces[i].v3 + 1 << std::endl;
	}
	file << std::endl;

	file.close();
}

int main() {
	// Read LiDAR data
	std::cout << "Reading data ..." << std::endl;
	LASreadOpener lasreadopener;
	lasreadopener.set_file_name("GK_462_100.laz");
	LASreader* lasreader = lasreadopener.open();

	int numPoints = (int)lasreader->npoints;
	if (MAX_POINTS > 0) numPoints = MAX_POINTS;
	Point* points = new Point[numPoints];

	for (int i = 0; i < numPoints && lasreader->read_point(); ++i) {
		// Print progress
		if (i % 10000 == 0) {
			printf("%.2f%%\r", 100.f * (float)i / (float)numPoints);
			fflush(stdout);
		}

		// Convert from centimeters to meters
		points[i] = Point(
			(double)lasreader->point.X / 100.,
			(double)lasreader->point.Y / 100.,
			(double)lasreader->point.Z / 100.,
			i
		);
	}
	
	lasreader->close();
	delete lasreader;


	// Shuffle points (if points are random, average O(1) edge flips)
	std::cout << "Shuffling points ..." << std::endl;
	std::random_device dev;
	std::mt19937 rng(dev());
	for (int i = 0; i < numPoints - 1; ++i) {
		std::uniform_int_distribution<std::mt19937::result_type> dist(i, numPoints - 1);
		int j = dist(rng);
		if (i == j) {
			continue;
		} else {
			Point p = points[i];
			points[i] = points[j];
			points[j] = p;

			points[i].idx = i;
			points[j].idx = j;
		}
	}


	// Get min, max and span
	std::cout << "Adjusting points ..." << std::endl;
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


	// Triangulation
	std::cout << "Triangulating points ..." << std::endl;

	// Create bounding triangle
	double hBound = (tan(M_PI / 3.) * xSpan / 2.) + ySpan;
	double wBound = 2. * hBound / tan(M_PI / 3.);         
	Point p0Bound(xMin - (wBound - xSpan) / 2. - (wBound), yMin - (hBound), 0., -1);
	Point p1Bound(xMax + (wBound - xSpan) / 2. + (wBound), yMin - (hBound), 0., -1);
	Point p2Bound((xMin + xMax) / 2., yMax + hBound - ySpan + (hBound), 0., -1);

	// Test bounding triangle
	if (DEBUG) {
		std::cout << "Testing data ..." << std::endl;
		Triangle triBound(&p0Bound, &p1Bound, &p2Bound);
		for (int i = 0; i < numPoints; ++i) {
			double w[3];
			if (!triBound.isInside(&points[i], w) || w[0] == 0.0 || w[1] == 0.0 || w[2] == 0.0) {
				if (w[0] == 0.0 || w[1] == 0.0 || w[2] == 0.0)
					std::cout << "Point on the edge of bounding triangle!" << std::endl;
				else
					std::cout << "Point outside bounding triangle!" << std::endl;
				return 1;
			}
		}
	}

	// Delaunay tree (DT)
	Node* root = new Node(&p0Bound, &p1Bound, &p2Bound);

	// Incrementally add points to DT
	for (int iPoint = 0; iPoint < numPoints; ++iPoint) {
		// Print progress
		if (iPoint % 10000 == 0) {
			printf("%.2f%%\r", 100.f * (float)iPoint / (float)numPoints);
			fflush(stdout);
		}

		// Find triangle in which the point lies
		Point* p = &(points[iPoint]);
		Node* n = root;
		double w[3];
		while (n->numChildren > 0) {
			if (n->child[0]->tri.isInside(p, w)) n = n->child[0];
			else if (n->child[1]->tri.isInside(p, w)) n = n->child[1];
			else if (n->numChildren > 2 && n->child[2]->tri.isInside(p, w)) n = n->child[2];
			else {
				std::cout << "ERROR: Point " << iPoint << " not inside any triangle!" << std::endl;
				return 1;
			}
		}

		// Check if point already exists
		if (n != root && w[0] == 0. && w[1] == 0. || w[0] == 0. && w[2] == 0. || w[1] == 0. && w[2] == 0.) {
			continue;
		}

		// Add point to triangle
		if (n == root || w[0] != 0. && w[1] != 0. && w[2] != 0.)  { // Point inside of triangle
			n->numChildren = 3;

			// Create new triangles
			for (int i = 0; i < 3; ++i)
				n->child[i] = new Node(p, n->tri.p[M1(i)], n->tri.p[M2(i)]);
			// Set neighbours
			for (int i = 0; i < 3; ++i) {
				n->child[i]->neighbor[0] = n->neighbor[i];
				n->child[i]->neighbor[1] = n->child[M1(i)];
				n->child[i]->neighbor[2] = n->child[M2(i)];
			}
			// Update neighbours
			for (int i = 0; i < 3; ++i)
				n->child[i]->updateNeigbors(n);
			// Validate edges
			for (int i = 0; i < 3; ++i)
				n->child[i]->validate(0);
		} else { // Point on edge of triangle
			int edge = 0;
			if (w[1] == 0.) edge = 1;
			else if (w[2] == 0.) edge = 2;

			n->split2(p, edge);

			// Validate edges
			for (int i = 0; i < 2; ++i) {
				n->child[i]->validate(0);
				if (n->neighbor[edge] != nullptr)
					n->neighbor[edge]->child[i]->validate(0);
			}
		}
	}
	
	// Extract triangles
	std::cout << "Extracting mesh ..." << std::endl;
	std::vector<Face> faces;
	getFaces(root, faces, points);

	// Export mesh
	std::cout << "Saving .obj file ..." << std::endl;
	saveObj("mesh.obj", points, numPoints, faces);

	std::cout << "Done!" << std::endl;

	return 0;
}
