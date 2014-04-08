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
  LineSegment (Point *p0, Point *p1) : p0(p0), p1(p1) {}
  bool intersects (LineSegment *l);

  Point *p0;
  Point *p1;
};

typedef vector<LineSegment *> LineSegments;

class KdTreeNode {
 public:
  KdTreeNode (LineSegment *l, int splitType) : lineSegment(l), splitType(splitType), splitAt(l->p0), left(0), right(0) {}
  void insert (LineSegment *l);
  void insertRight (LineSegment *l);
  void insertLeft (LineSegment *l);
  bool intersects (LineSegment *l);
  void debug (int level);

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

  KdTreeNode *root;
};

class Arrangement {
 public:
  Arrangement () {}
  ~Arrangement ();
  void addLineSegment (LineSegment *l);
  bool intersects (LineSegment *l);

  LineSegments lineSegments;
};

void splitLineSegment (LineSegment *l, Point *splitAt, int splitType, LineSegment **l0, LineSegment **l1);

void pl(LineSegment *l);

#endif
