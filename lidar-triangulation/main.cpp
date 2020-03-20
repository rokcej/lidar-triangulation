#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <fstream>
#include "structures.h"
#include "lasreader.hpp"
#include "laswriter.hpp"

#define DEBUG true

inline int M1(const int& x) { return (x + 1) % 3; }
inline int M2(const int& x) { return (x + 2) % 3; }

void getFaces(Node* node, std::vector<Face> faces, Point *points) {
	if (node->processed)
		return;
	node->processed = true;

	if (node->hasChildren) {
		getFaces(node->child[0], faces, points);
		getFaces(node->child[1], faces, points);
		if (node->child[2] != nullptr)
			getFaces(node->child[2], faces, points);
	} else {
		faces.emplace_back(
			(int)(node->tri.p[0] - points),
			(int)(node->tri.p[1] - points),
			(int)(node->tri.p[2] - points)
		);
	}
}

void saveObj(std::string filename, Point *points, int numPoints, std::vector<Face> faces) {
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

	int maxPoints = 10000; // (int)lasreader->npoints;
	int numPoints = 0;
	Point* points = new Point[maxPoints];

	for (int i = 0; i < maxPoints && lasreader->read_point(); ++i) {
		// Convert from centimeters to meters
		double x = (double)lasreader->point.X / 100.;
		double y = (double)lasreader->point.Y / 100.;
		double z = (double)lasreader->point.Z / 100.;
		
		// Remove duplicates, keep the highest z value
		bool duplicate = false;
		for (int j = 0; j < numPoints; ++j) {
			if (points[j].x == x && points[j].y == y) {
				if (z > points[j].z)
					points[j].z = z;
				duplicate = true;
				break;
			}
		}

		if (!duplicate) {
			points[numPoints].x = x;
			points[numPoints].y = y;
			points[numPoints].z = z;
			++numPoints;
		}
	}

	lasreader->close();
	delete lasreader;

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
	Point p0Bound = { xMin - (wBound - xSpan) / 2. - (wBound), yMin - (hBound), 0. };
	Point p1Bound = { xMax + (wBound - xSpan) / 2. + (wBound), yMin - (hBound), 0. };
	Point p2Bound = { (xMin + xMax) / 2., yMax + hBound - ySpan + (hBound), 0. };
	Triangle triBound(&p0Bound, &p1Bound, &p2Bound);

	// Test bounding triangle
	if (DEBUG) {
		std::cout << "Testing data ..." << std::endl;
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
	std::cout << "Triangulating points ..." << std::endl;
	Node* root = new Node(&p0Bound, &p1Bound, &p2Bound);

	// Incrementally add points to DT
	for (int iPoint = 0; iPoint < numPoints; ++iPoint) {
		// Find triangle in which the point lies
		Point* p = &points[iPoint];
		Node* n = root;
		double w[3];
		while (n->hasChildren) {
			if (n->child[0]->tri.isInside(p, w)) n = n->child[0];
			else if (n->child[1]->tri.isInside(p, w)) n = n->child[1];
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
