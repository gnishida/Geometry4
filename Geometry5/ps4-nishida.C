#include <vector>
#include <iostream>
#include "acp.h"
#include "kdtree.h"
#include <time.h>

using namespace std;

/**
 * Final Project: Kd-tree data structure for line segments.
 * This program randomly generates N + 1 line segments, and test if the last line segment intersects with the first N line segments.
 *
 * This outputs all the N + 1 line segments, ane the result, "intersected" or "not intersected".
 */
int main(int argc, char *argv[]) {
	Parameter::enable();

	for (int n = 100; n < 1000; n+=100) {
		// read input data to build a kd tree
		LineSegments lineSegments;

		for (int i = 0; i < n; ) {
			double x1 = rand() % 1000;
			double y1 = rand() % 1000;

			double x2 = x1 + rand() % 10;
			double y2 = y1 + rand() % 10;

			LineSegment *l = new LineSegment(new InputPoint(x1, y1), new InputPoint(x2, y2));

			if (!naiveIntersects(lineSegments, *l)) {
				//cout << x1 << " " << y1 << " " << x2 << " " << y2 << endl;
				lineSegments.push_back(l);
				++i;
			}
		}

		KdTree kdTree;
		kdTree.build(lineSegments);

		// generate 1000 test data
		LineSegments tests;
		for (int i = 0; i < 1000; ++i) {
			double x1 = rand() % 1000;
			double y1 = rand() % 1000;

			double x2 = x1 + rand() % 10;
			double y2 = y1 + rand() % 10;

			LineSegment *l = new LineSegment(new InputPoint(x1, y1), new InputPoint(x2, y2));

			tests.push_back(l);
		}

		// test if the given line segment intersects with the other line segments
		{
			time_t start = clock();
			for (int i = 0; i < tests.size(); ++i) {
				kdTree.intersects(tests[i]);
			}
			time_t end = clock();
			cout << "Kd-Tree Elapsed time [ms] (n = " << n << ") : " << (double)(end-start)/CLOCKS_PER_SEC << endl;
		}

		{
			time_t start = clock();
			for (int i = 0; i < tests.size(); ++i) {
				naiveIntersects(lineSegments, *tests[i]);
			}
			time_t end = clock();
			cout << "Naive Elapsed time [ms]: (n = " << n << ") :  " << (double)(end-start)/CLOCKS_PER_SEC << endl;
		}

		// check the correctness of the result of the Kd-tree version against the naive approach
		{
			for (int i = 0; i < tests.size(); ++i) {
				if (kdTree.intersects(tests[i]) != naiveIntersects(lineSegments, *tests[i])) {
					cout << "Incorrect result was found!!! i = " << i << " n = " << n << endl;
					pl(tests[i]);
					cout << "Kd Tree: " << kdTree.intersects(tests[i]) << endl;
					cout << "Naive  : " << naiveIntersects(lineSegments, *tests[i]) << endl;
				}
			}
		}
	}

	return 0;
}
