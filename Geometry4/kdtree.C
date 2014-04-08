#include "kdtree.h"
#include <fstream>

//////////////////////////////////////////////////////////////////////////////////
// arrangement

void KdTreeNode::insert (LineSegment *l)
{
  switch (splitType) {
  case 0:
    if (XOrder(l->p0, splitAt) && XOrder(l->p1, splitAt)) {
	  insertRight(l);
	} else if (XOrder(splitAt, l->p0) && XOrder(splitAt, l->p1)) {
	  insertLeft(l);
	} else {
	  LineSegment *l0, *l1;
	  splitLineSegment(l, splitAt, splitType, l0, l1);
      insertRight(l0);
	  insertLeft(l1);
	}
	break;
  case 1:
    if (YOrder(l->p0, splitAt) && YOrder(l->p1, splitAt)) {
	  insertRight(l);
	} else if (YOrder(splitAt, l->p0) && YOrder(splitAt, l->p1)) {
	  insertLeft(l);
	} else {
	  LineSegment *l0, *l1;
	  splitLineSegment(l, splitAt, splitType, l0, l1);
      insertRight(l0);
	  insertLeft(l1);
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
  switch (splitType) {
  case 0:
    if (XOrder(l->p0, splitAt) && XOrder(l->p1, splitAt)) {
	  if (right == 0)
		return false;
	  else
        return right->intersects(l);
	} else if (XOrder(splitAt, l->p0) && XOrder(splitAt, l->p1)) {
	  if (left == 0)
	    return false;
	  else
        return left->intersects(l);
	} else {
	  LineSegment *l0, *l1;
	  splitLineSegment(l, splitAt, splitType, l0, l1);

	  if (left != 0)
        if (left->intersects(l0)) return true;
	  if (right != 0) 
        if (right->intersects(l1)) return true;
	  return false;
	}
	break;
  case 1:
    if (YOrder(l->p0, splitAt) && YOrder(l->p1, splitAt)) {
	  if (right == 0)
		return false;
	  else
        return right->intersects(l);
	} else if (YOrder(splitAt, l->p0) && YOrder(splitAt, l->p1)) {
	  if (left == 0)
	    return false;
	  else
        return left->intersects(l);
	} else {
	  LineSegment *l0, *l1;
	  splitLineSegment(l, splitAt, splitType, l0, l1);

	  if (left != 0)
        if (left->intersects(l0)) return true;
	  if (right != 0) 
        if (right->intersects(l1)) return true;
	  return false;
	}
    break;
  }
}

void KdTree::insert (LineSegment *l)
{
  if (root == 0)
    root = new KdTreeNode(l);
  else
    root->insert(l);
}

bool KdTree::intersects (LineSegment *l)
{
  return root->intersects(l); 
}

Arrangement::~Arrangement ()
{
  for (LineSegments::iterator l = lineSegments.begin(); l != lineSegments.end(); ++l)
    delete *l;
}

void Arrangement::addLineSegment (LineSegment *l)
{
	lineSegments.push_back(l);
}

bool Arrangement::intersects (LineSegment *l)
{
	KdTree kdTree;

	for (int i = 0; i < lineSegments.size(); ++i) {
		kdTree.insert(lineSegments[i]);
	}

	return kdTree.intersects(l);
}

void splitLineSegment (LineSegment *l, Point *splitAt, int splitType, LineSegment *l0, LineSegment *l1)
{
  if (splitType == 0) {
    PV2 p = splitAt->getP() + PV2(0, 10);
    PV2 intP = lineIntersection(l->p0->getP(), l->p1->getP(), splitAt->getP(), p);

    if (XOrder(l->p0, splitAt)) {
	  l0 = new LineSegment(l->p0, new InputPoint(intP));
	  l1 = new LineSegment(l->p1, new InputPoint(intP));
	} else {
	  l0 = new LineSegment(l->p1, new InputPoint(intP));
	  l1 = new LineSegment(l->p0, new InputPoint(intP));
	}
  } else {
    PV2 p = splitAt->getP() + PV2(10, 0);
    PV2 intP = lineIntersection(l->p0->getP(), l->p1->getP(), splitAt->getP(), p);

    if (YOrder(l->p0, splitAt)) {
	  l0 = new LineSegment(l->p0, new InputPoint(intP));
	  l1 = new LineSegment(l->p1, new InputPoint(intP));
	} else {
	  l0 = new LineSegment(l->p1, new InputPoint(intP));
	  l1 = new LineSegment(l->p0, new InputPoint(intP));
	}
  }
}
