# LiDAR Triangulation

The program reads [LiDAR](https://en.wikipedia.org/wiki/Lidar) terrain data and uses [Delaunay triangulation](https://en.wikipedia.org/wiki/Delaunay_triangulation) to convert the point cloud into a triangle mesh. Written in C++.

## Algorithm

The algorithm uses an incremental approach to achieve 2D Delaunay triangulation. The `z` coordinate of points is disregarded during the triangulation process.

It starts by creating a "supertriangle" that encompasses all points. Points are then inserted into the current triangulation one-by-one. When a point is inserted, it splits the triangle that contains it into 3 smaller triangles. (A degenerate case exists when the point lies on an edge. Then, the 2 triangles that share the edge are each split into 2 triangles). When new triangles are created, the algorithm recursively checks if they are valid and flips any illegal edges.

The Delaunay tree data structure is used to keep track of all splits and flips performed. Given *n* points, this allows us to find the triangle that contains any point in O(log *n*) time. The order of points is randomized to ensure that each insertion flips, on average, O(1) edges. This results in the total computational complexity of O(*n* log *n*).

## Parameters

You can change the following `#define`s to configure how the program works.

* **`INPUT_FILE`** - Input `.laz`/`.las` file containing LiDAR terrain data
* **`OUTPUT_FILE`** - Output `.obj` file containing the triangle mesh
* **`MAX_POINTS`** - Maximum number of points to use (set to `0` to disable the limit)
* **`DEBUG`** - Run in debug mode

# Dependencies

* **LAStools** (LASlib and LASzip) - Library for parsing `.laz`/`.las` LiDAR files

