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
		
	for (int n = 1000; n <= 10000; n+=1000) {
		// read input data to build a kd tree
		LineSegments lineSegments;

		for (int i = 0; i < n; ) {
			double x1 = rand() % 1000;
			double y1 = rand() % 1000;

			double x2 = x1 + rand() % 10;
			double y2 = y1 + rand() % 10;

			//cout << x1 << "," << y1 << "," << x2 << "," << y2 << endl;
			LineSegment *l = new LineSegment(new InputPoint(x1, y1), new InputPoint(x2, y2));

			if (!naiveIntersects(lineSegments, *l)) {
				lineSegments.push_back(l);
				++i;
			}
		}

		KdTree kdTree1;
		KdTree kdTree2;
		KdTree kdTree3;
		kdTree1.build(lineSegments);
		kdTree2.medianBuild(lineSegments);
		kdTree3.naiveBuild(lineSegments);

		// generate 10000 test data
		LineSegments tests;
		for (int i = 0; i < 10000; ++i) {
			double x1 = rand() % 1000;
			double y1 = rand() % 1000;

			double x2 = x1 + rand() % 10;
			double y2 = y1 + rand() % 10;

			LineSegment *l = new LineSegment(new InputPoint(x1, y1), new InputPoint(x2, y2));

			tests.push_back(l);
		}

		// test by kdtree
		{
			time_t start = clock();
			for (int i = 0; i < tests.size(); ++i) {
				kdTree1.intersects(tests[i]);
			}
			time_t end = clock();
			cout << "Kd-Tree (cost  ) Elapsed time [ms] (n = " << n << ") : " << (double)(end-start)/CLOCKS_PER_SEC*0.1 << endl;
		}

		// test by median kdtree
		{
			time_t start = clock();
			for (int i = 0; i < tests.size(); ++i) {
				kdTree2.intersects(tests[i]);
			}
			time_t end = clock();
			cout << "Kd-Tree (median) Elapsed time [ms] (n = " << n << ") : " << (double)(end-start)/CLOCKS_PER_SEC*0.1 << endl;
		}

		// test by naive kdtree
		{
			time_t start = clock();
			for (int i = 0; i < tests.size(); ++i) {
				kdTree3.intersects(tests[i]);
			}
			time_t end = clock();
			cout << "Kd-Tree (random) Elapsed time [ms] (n = " << n << ") : " << (double)(end-start)/CLOCKS_PER_SEC*0.1 << endl;
		}

		// test by N^2 approach
		{
			time_t start = clock();
			for (int i = 0; i < tests.size(); ++i) {
				naiveIntersects(lineSegments, *tests[i]);
			}
			time_t end = clock();
			cout << "N^2 approach     Elapsed time [ms] (n = " << n << ") : " << (double)(end-start)/CLOCKS_PER_SEC*0.1 << endl;
		}

		//break;
	}

	return 0;
}
