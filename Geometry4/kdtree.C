#include "kdtree.h"
#include "permute.h"
#include <fstream>

//////////////////////////////////////////////////////////////////////////////////
// arrangement

bool LineSegment::intersects (LineSegment *l)
{
  return 
	LeftTurn(p0, l->p0, l->p1) != LeftTurn(p1, l->p0, l->p1) &&
    LeftTurn(l->p0, p0, p1) != LeftTurn(l->p1, p0, p1);
}

void KdTreeNode::insert (LineSegment *l)
{
  switch (splitType) {
  case 0:
    if (l->p0 != splitAt && l->p1 != splitAt && XOrder(l->p0, splitAt) == 1 && XOrder(l->p1, splitAt) == 1) {
	  insertLeft(l);
	} else if (l->p0 != splitAt && l->p1 != splitAt && XOrder(splitAt, l->p0) == 1 && XOrder(splitAt, l->p1) == 1) {
	  insertRight(l);
	} else {
	  LineSegment *l0, *l1;
	  splitLineSegment(l, splitAt, splitType, &l0, &l1);

      insertLeft(l0);
	  insertRight(l1);
	}
	break;
  case 1:
    if (l->p0 != splitAt && l->p1 != splitAt && YOrder(l->p0, splitAt) == 1 && YOrder(l->p1, splitAt) == 1) {
	  insertLeft(l);
	} else if (l->p0 != splitAt && l->p1 != splitAt && YOrder(splitAt, l->p0) == 1 && YOrder(splitAt, l->p1) == 1) {
	  insertRight(l);
	} else {
	  LineSegment *l0, *l1;
	  splitLineSegment(l, splitAt, splitType, &l0, &l1);
      insertLeft(l0);
	  insertRight(l1);
	}
    break;
  }
}

void KdTreeNode::insertRight (LineSegment *l)
{
  if (right)
    right->insert(l);
  else 
    right = new KdTreeNode(l, (splitType + 1) % 2);
}

void KdTreeNode::insertLeft (LineSegment *l)
{
  if (left)
    left->insert(l);
  else
    left = new KdTreeNode(l, (splitType + 1) % 2);
}

bool KdTreeNode::intersects (LineSegment *l)
{
  if (lineSegment != 0 && lineSegment->intersects(l)) return true;

  switch (splitType) {
  case 0:
    if (l->p0 != splitAt && l->p1 != splitAt && XOrder(l->p0, splitAt) == 1 && XOrder(l->p1, splitAt) == 1) {
	  if (left == 0)
		return false;
	  else
        return left->intersects(l);
	} else if (l->p0 != splitAt && l->p1 != splitAt && XOrder(splitAt, l->p0) == 1 && XOrder(splitAt, l->p1) == 1) {
	  if (right == 0)
	    return false;
	  else
        return right->intersects(l);
	} else {
	  LineSegment *l0, *l1;
	  splitLineSegment(l, splitAt, splitType, &l0, &l1);

	  if (left != 0)
        if (left->intersects(l0)) return true;
	  if (right != 0) 
        if (right->intersects(l1)) return true;
	  return false;
	}
	break;
  case 1:
    if (l->p0 != splitAt && l->p1 != splitAt && YOrder(l->p0, splitAt) == 1 && YOrder(l->p1, splitAt) == 1) {
	  if (left == 0)
		return false;
	  else
        return left->intersects(l);
	} else if (l->p0 != splitAt && l->p1 != splitAt && YOrder(splitAt, l->p0) == 1 && YOrder(splitAt, l->p1) == 1) {
	  if (right == 0)
	    return false;
	  else
        return right->intersects(l);
	} else {
	  LineSegment *l0, *l1;
	  splitLineSegment(l, splitAt, splitType, &l0, &l1);

	  if (left != 0)
        if (left->intersects(l0)) return true;
	  if (right != 0) 
        if (right->intersects(l1)) return true;
	  return false;
	}
    break;
  }
}

void KdTreeNode::debug (int level)
{
  for (int i = 0; i < level; ++i)
    cout << " ";
  cout << "(" << splitAt->getP().getX().mid() << ", " << splitAt->getP().getY().mid() << ") type: " << splitType << endl;

  if (left != 0) {
    for (int i = 0; i < level; ++i)
      cout << " ";
    cout << "Left:" << endl;
	left->debug(level + 3);
  }
  if (right != 0) {
    for (int i = 0; i < level; ++i)
      cout << " ";
    cout << "Right:" << endl;
	right->debug(level + 3);
  }
}

void KdTreeNode::build (LineSegments &lineSegments, LineSegments::iterator begin, LineSegments::iterator end)
{
  // retrieve the median
  if (splitType == 0) {
    sort(begin, end, LineXOrder());
  } else {
    sort(begin, end, LineYOrder());
  }

  int mid = (end - begin) / 2;
  lineSegment = lineSegments[mid];
  splitAt = lineSegment->p0;

  if (mid > 0) {
    left = new KdTreeNode((splitType + 1) % 2);
	left->build(lineSegments, begin, begin + mid);
  }
  if (begin + mid + 1 != end) {
    right = new KdTreeNode((splitType + 1) % 2);
	right->build(lineSegments, begin + mid + 1, end);
  }
}

int KdTreeNode::depth ()
{
  int leftDepth = 0;
  if (left != 0) leftDepth = left->depth();

  int rightDepth = 0;
  if (right != 0) rightDepth = right->depth();

  return (leftDepth > rightDepth) ? leftDepth + 1 : rightDepth + 1;
}

void KdTree::insert (LineSegment *l)
{
  if (root == 0)
    root = new KdTreeNode(l, 0);
  else
    root->insert(l);
}

bool KdTree::intersects (LineSegment *l)
{
  if (root == 0) return false;
  else return root->intersects(l); 
}

void KdTree::debug ()
{
  if (root != 0)
    root->debug(0);
}

void KdTree::build (LineSegments &lineSegments)
{
  map<int, LineSegments> orderedLineSegments;
  orderLineSegments(lineSegments, lineSegments.begin(), lineSegments.end(), 0, 0, orderedLineSegments);

  for (map<int, LineSegments>::iterator it = orderedLineSegments.begin(); it != orderedLineSegments.end(); ++it) {
	for (LineSegments::iterator l = it->second.begin(); l != it->second.end(); ++l) {
	  insert(*l);
	}
  }
}

void KdTree::naiveBuild (LineSegments &lineSegments)
{
  // Compute a random permutation p5, p6, . . . , pn of the remaining points.
  int n = lineSegments.size();
  int *p = new int [n];
  randomPermutation (n, p);

  for (int i = 0; i < lineSegments.size(); ++i) {
    insert(lineSegments[p[i]]);
  }
}

void KdTree::orderLineSegments (LineSegments &lineSegments, LineSegments::iterator begin, LineSegments::iterator end, int orderType, int depth, map<int, LineSegments> &orderedLineSegments)
{
  if (orderType == 0) {
    sort(begin, end, LineXOrder());
  } else {
    sort(begin, end, LineYOrder());
  }

  int mid = (end - begin) / 2;

  orderedLineSegments[depth].push_back(*(begin + mid));

  if (mid > 0) {
    orderLineSegments(lineSegments, begin, begin + mid, 1 - orderType, depth + 1, orderedLineSegments);
  }
  if (begin + mid + 1 != end) {
    orderLineSegments(lineSegments, begin + mid + 1, end, 1 - orderType, depth + 1, orderedLineSegments);
  }
}

int KdTree::depth ()
{
  if (root == 0) return 0;
  else return root->depth();
}

bool naiveIntersects (LineSegments &lineSegments, LineSegment &l)
{
  for (LineSegments::iterator it = lineSegments.begin(); it != lineSegments.end(); ++it) {
    if ((*it)->intersects(&l)) return true;
  }

  return false;
}

void splitLineSegment (LineSegment *l, Point *splitAt, int splitType, LineSegment **l0, LineSegment **l1)
{
  if (splitType == 0) {
	Point *p = new LineIntersectionWithYAxis(l->p0, l->p1, splitAt);

	if (l->p0 != splitAt && XOrder(l->p0, splitAt) == 1) {
	  *l0 = new LineSegment(l->p0, p);
	  *l1 = new LineSegment(l->p1, p);
	} else {
	  *l0 = new LineSegment(l->p1, p);
	  *l1 = new LineSegment(l->p0, p);
	}
  } else {
    Point *p = new LineIntersectionWithXAxis(l->p0, l->p1, splitAt);

    if (l->p0 != splitAt && YOrder(l->p0, splitAt) == 1) {
	  *l0 = new LineSegment(l->p0, p);
	  *l1 = new LineSegment(l->p1, p);
	} else {
	  *l0 = new LineSegment(l->p1, p);
	  *l1 = new LineSegment(l->p0, p);
	}
  }
}

void pl(LineSegment *l)
{
  cout << "(" << l->p0->getP().getX().mid() << "," << l->p0->getP().getY().mid() << ") - (" << l->p1->getP().getX().mid() << "," << l->p1->getP().getY().mid() << ")" << endl;
}
