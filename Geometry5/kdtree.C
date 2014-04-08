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
    if (XOrder(l->p0, splitAt) == 1 && XOrder(l->p1, splitAt) == 1) {
	  insertLeft(l);
	} else if (XOrder(splitAt, l->p0) == 1 && XOrder(splitAt, l->p1) == 1) {
	  insertRight(l);
	} else {
	  LineSegment *l0, *l1;
	  splitLineSegment(l, splitAt, splitType, &l0, &l1);

      insertLeft(l0);
	  insertRight(l1);
	}
	break;
  case 1:
    if (YOrder(l->p0, splitAt) == 1 && YOrder(l->p1, splitAt) == 1) {
	  insertLeft(l);
	} else if (YOrder(splitAt, l->p0) == 1 && YOrder(splitAt, l->p1) == 1) {
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
    if (XOrder(l->p0, splitAt) == 1 && XOrder(l->p1, splitAt) == 1) {
	  if (left == 0)
		return false;
	  else
        return left->intersects(l);
	} else if (XOrder(splitAt, l->p0) == 1 && XOrder(splitAt, l->p1) == 1) {
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
    if (YOrder(l->p0, splitAt) == 1 && YOrder(l->p1, splitAt) == 1) {
	  if (left == 0)
		return false;
	  else
        return left->intersects(l);
	} else if (YOrder(splitAt, l->p0) == 1 && YOrder(splitAt, l->p1) == 1) {
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
  // Compute a random permutation p5, p6, . . . , pn of the remaining points.
  int n = lineSegments.size();
  int *p = new int [n];
  randomPermutation (n, p);

  for (int i = 0; i < lineSegments.size(); ++i) {
    insert(lineSegments[p[i]]);
  }
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
    PV2 p = splitAt->getP() + PV2(0, 10);
    PV2 intP = lineIntersection(l->p0->getP(), l->p1->getP(), splitAt->getP(), p);

	if (XOrder(l->p0, splitAt) == 1) {
	  *l0 = new LineSegment(l->p0, new InputPoint(intP));
	  *l1 = new LineSegment(l->p1, new InputPoint(intP));
	} else {
	  *l0 = new LineSegment(l->p1, new InputPoint(intP));
	  *l1 = new LineSegment(l->p0, new InputPoint(intP));
	}
  } else {
    PV2 p = splitAt->getP() + PV2(10, 0);
    PV2 intP = lineIntersection(l->p0->getP(), l->p1->getP(), splitAt->getP(), p);

    if (YOrder(l->p0, splitAt) == 1) {
	  *l0 = new LineSegment(l->p0, new InputPoint(intP));
	  *l1 = new LineSegment(l->p1, new InputPoint(intP));
	} else {
	  *l0 = new LineSegment(l->p1, new InputPoint(intP));
	  *l1 = new LineSegment(l->p0, new InputPoint(intP));
	}
  }
}

void pl(LineSegment *l)
{
  cout << "(" << l->p0->getP().getX().mid() << "," << l->p0->getP().getY().mid() << ") - (" << l->p1->getP().getX().mid() << "," << l->p1->getP().getY().mid() << ")" << endl;
}
