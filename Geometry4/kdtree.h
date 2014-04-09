#ifndef KDTREE
#define KDTREE

#include <vector>
#include <set>
#include <map>
#include <iomanip>
#include "point.h"
#include "object.h"

using namespace std;
using namespace acp;

///////////////////////////////////////////////////////////////////////////////////
// Arrangement

class LineSegment {
 public:
  LineSegment () : p0(0), p1(0) {}
  LineSegment (Point *p0, Point *p1) : p0(p0), p1(p1) {}
  bool intersects (LineSegment *l);

  Point *p0;
  Point *p1;
};

typedef vector<LineSegment *> LineSegments;

class KdTreeNode {
 public:
  KdTreeNode (int splitType) : lineSegment(0), splitType(splitType), splitAt(0), left(0), right(0) {}
  KdTreeNode (LineSegment *l, int splitType) : lineSegment(l), splitType(splitType), splitAt(l->p0), left(0), right(0) {}
  void insert (LineSegment *l);
  void insertRight (LineSegment *l);
  void insertLeft (LineSegment *l);
  bool intersects (LineSegment *l);
  void debug (int level);
  int depth ();

  int splitType;	// split by a plane that is perpendicular to X axis (0) or Y axis (1).
  Point *splitAt;
  LineSegment *lineSegment;
  KdTreeNode *left;
  KdTreeNode *right;
};

class KdTree {
 public:
  KdTree () : root(0) {}
  void insert (LineSegment *l);
  bool intersects (LineSegment *l);
  void debug ();
  void build (LineSegments &lineSegments);
  void medianBuild (LineSegments &lineSegments);
  void naiveBuild (LineSegments &lineSegments);
  void orderLineSegmentsByCost (LineSegments &lineSegments, LineSegments::iterator begin, LineSegments::iterator end, int orderType, int depth, map<int, LineSegments> &orderedLineSegments);
  void orderLineSegmentsByMedian (LineSegments &lineSegments, LineSegments::iterator begin, LineSegments::iterator end, int orderType, int depth, map<int, LineSegments> &orderedLineSegments);
  double computeCost (LineSegments &lineSegments, LineSegments::iterator begin, LineSegments::iterator end, Point* p, int splitType);
  int depth ();

  KdTreeNode *root;
};

class LineXOrder {
 public:
  bool operator() (LineSegment *l, LineSegment *m) const {
    return l != m && XOrder(l->p0, m->p0) == 1;
  }
};

class LineYOrder {
 public:
  bool operator() (LineSegment *l, LineSegment *m) const {
    return l != m && YOrder(l->p0, m->p0) == 1;
  }
};

bool naiveIntersects (LineSegments &lineSegments, LineSegment &l);

void splitLineSegment (LineSegment *l, Point *splitAt, int splitType, LineSegment **l0, LineSegment **l1);

void pl(LineSegment *l);

#endif
