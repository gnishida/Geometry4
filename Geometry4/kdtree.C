#include "kdtree.h"
#include <fstream>

//////////////////////////////////////////////////////////////////////////////////
// arrangement

void KdTreeNode::insert (LineSegment *l)
{
  switch (splitType) {
  case 0:
    if (XOrder(l->p0, splitAt) && XOrder(l->p1, splitAt)) {
	  if (right)
	    right->insert(l);
	  else 
	    right = new KdTreeNode(l);		
	} else if (XOrder(splitAt, l->p0) && XOrder(splitAt, l->p1)) {
	  if (left)
	    left->insert(l);
	  else
	    left = new KdTreeNode(l);
	} else {
      PV2 p = splitAt->getP() + PV2(0, 10);
      PV2 intP = lineIntersection(l->p0->getP(), l->p1->getP(), splitAt->getP(), p);

	  if (XOrder(l->p0, splitAt)) {
		LineSegment *l1 = new LineSegment(l->p0, intP);
		if (right)
		  right->insert(l1);
		else
		  right = new KdTreeNode(l1);
		LineSegment *l2 = new LineSegment(l->p1, intP);
		if (left)
		  left->insert(l2);
		else
		  left = new KdTreeNode(l2);
	  } else {
		LineSegment *l1 = new LineSegment(l->p0, intP);
		if (left)
		  left->insert(l1);
		else
		  left = new KdTreeNode(l1);
		LineSegment *l2 = new LineSegment(l->p1, intP);
		if (right)
		  right->insert(l2);
		else
		  right = new KdTreeNode(l2);
	  }
	}
	break;
  case 1:
    break;
  }
}

void KdTree::insert (LineSegment *l)
{
  if (root == 0) {
    root = new KdTreeNode(l);
  }
  else
    root->insert(l);
}

bool KdTree::intersects (LineSegment *l)
{
  return false;
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