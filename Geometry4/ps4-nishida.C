#include <vector>
#include <iostream>
#include "acp.h"
#include "kdtree.h"

using namespace std;

/**
 * Final Project: Kd-tree data structure for line segments.
 * This program reads N + 1 line segments, and test if the last line segment intersects with the first N line segments.
 *
 * The input format is as follows:
 * N L1x_1 L1y_1 L1x_2 L1y_2 L2x_1 L2y_1 L2x_2 L2y_2 ... LNx_1 LNy_1 LNx_2 LNy_2 Lx_1 Ly_1 Lx_2 Ly_2
 * The output is "intersected" or "not intersected".
 */
int main(int argc, char *argv[]) {
	Parameter::enable();

	Arrangement arr;

	// read input data to build a kd tree
	int numLineSegments;
	cin >> numLineSegments;

	for (int i = 0; i < numLineSegments; ++i) {
		double x1, y1, x2, y2;
		cin >> x1 >> y1 >> x2 >> y2;

		LineSegment *l = new LineSegment(new InputPoint(x1, y1), new InputPoint(x2, y2));
		arr.addLineSegment(l);
	}

	// test if the given line segment intersects with the other line segments
	{
		double x1, y1, x2, y2;
		cin >> x1 >> y1 >> x2 >> y2;

		LineSegment *l = new LineSegment(new InputPoint(x1, y1), new InputPoint(x2, y2));
		if (arr.intersects(l)) {
			cout << "intersected" << endl;
		} else {
			cout << "not intersected" << endl;
		}
	}

	return 0;
}
