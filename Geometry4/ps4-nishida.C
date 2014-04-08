#include <vector>
#include <iostream>
#include "acp.h"
#include "kdtree.h"

using namespace std;

/**
 * Final Project: Kd-tree data structure for line segments.
 * This program randomly generates N + 1 line segments, and test if the last line segment intersects with the first N line segments.
 *
 * This outputs all the N + 1 line segments, ane the result, "intersected" or "not intersected".
 */
int main(int argc, char *argv[]) {
	Parameter::enable();

	Arrangement arr;

	// read input data to build a kd tree
	int numLineSegments;
	cin >> numLineSegments;

	for (int i = 0; i < numLineSegments; ++i) {
		double x1 = rand() % 100;
		double y1 = rand() % 100;
		double x2 = rand() % 100;
		double y2 = rand() % 100;

		cout << x1 << ", " << y1 << ", " << x2 << ", " << y2 << endl;

		LineSegment *l = new LineSegment(new InputPoint(x1, y1), new InputPoint(x2, y2));
		arr.addLineSegment(l);
	}

	// test if the given line segment intersects with the other line segments
	{
		double x1 = rand() % 100;
		double y1 = rand() % 100;
		double x2 = rand() % 100;
		double y2 = rand() % 100;

		cout << "Test Line: " << x1 << ", " << y1 << ", " << x2 << ", " << y2 << endl;

		LineSegment *l = new LineSegment(new InputPoint(x1, y1), new InputPoint(x2, y2));
		if (arr.intersects(l)) {
			cout << "intersected" << endl;
		} else {
			cout << "not intersected" << endl;
		}
	}

	return 0;
}
