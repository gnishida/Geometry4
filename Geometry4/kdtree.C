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
  orderLineSegmentsByCost(lineSegments, lineSegments.begin(), lineSegments.end(), 0, 0, orderedLineSegments);

  for (LineSegments::iterator it = lineSegments.begin(); it != lineSegments.end(); ++it) {
    //pl(*it);
  }

  for (map<int, LineSegments>::iterator it = orderedLineSegments.begin(); it != orderedLineSegments.end(); ++it) {
	for (LineSegments::iterator l = it->second.begin(); l != it->second.end(); ++l) {
	  insert(*l);
	}
  }
}

void KdTree::medianBuild (LineSegments &lineSegments)
{
  map<int, LineSegments> orderedLineSegments;
  orderLineSegmentsByMedian(lineSegments, lineSegments.begin(), lineSegments.end(), 0, 0, orderedLineSegments);

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

void KdTree::orderLineSegmentsByCost (LineSegments &lineSegments, LineSegments::iterator begin, LineSegments::iterator end, int orderType, int depth, map<int, LineSegments> &orderedLineSegments)
{
  double min_c = std::numeric_limits<double>::max();
  int mid;
  int i = 0;
  for (LineSegments::iterator it = begin; it != end; ++it, ++i) {
	double c = computeCost(lineSegments, begin, end, (*it)->p0, orderType);
	if (c < min_c) {
	  min_c = c;
      mid = i;
	}
	/*
	c = computeCost(lineSegments, begin, end, (*it)->p1, orderType);
	if (c < min_c) {
      mid = i;
	}
	*/
  }

  // separate the line segments such that the first half is for the left sub-tree and the second half is for the right sub-tree
  LineSegment *mid_l = *(begin + mid);
  {
    swap(*(end - 1), *(begin + mid));
	LineSegments::iterator it = begin;
	LineSegments::iterator r_index = end - 2;
	for (; it != end - 1 && it <= r_index;) {
      if (orderType == 0) {
        if (XOrder((*it)->p0, mid_l->p0) == 1) {
		  it++;
		} else {
          swap(*it, *r_index);
		  r_index--;
		}
	  } else {
        if (YOrder((*it)->p0, mid_l->p0) == 1) {
		  it++;
		} else {
          swap(*it, *r_index);
		  r_index--;
		}
	  }
	}

    swap(*it, *(end - 1));

	mid = it - begin;
  }

  orderedLineSegments[depth].push_back(mid_l);

  if (mid > 0) {
    orderLineSegmentsByCost(lineSegments, begin, begin + mid, 1 - orderType, depth + 1, orderedLineSegments);
  }
  if (begin + mid + 1 != end) {
    orderLineSegmentsByCost(lineSegments, begin + mid + 1, end, 1 - orderType, depth + 1, orderedLineSegments);
  }
}

void KdTree::orderLineSegmentsByMedian (LineSegments &lineSegments, LineSegments::iterator begin, LineSegments::iterator end, int orderType, int depth, map<int, LineSegments> &orderedLineSegments)
{
  if (orderType == 0) {
    sort(begin, end, LineXOrder());
  } else {
    sort(begin, end, LineYOrder());
  }

  int mid = (end - begin) / 2;

  orderedLineSegments[depth].push_back(*(begin + mid));

  if (mid > 0) {
    orderLineSegmentsByMedian(lineSegments, begin, begin + mid, 1 - orderType, depth + 1, orderedLineSegments);
  }
  if (begin + mid + 1 != end) {
    orderLineSegmentsByMedian(lineSegments, begin + mid + 1, end, 1 - orderType, depth + 1, orderedLineSegments);
  }
}

double KdTree::computeCost (LineSegments &lineSegments, LineSegments::iterator begin, LineSegments::iterator end, Point* p, int splitType)
{
  int TL = 0;
  int TR = 0;
  Parameter PL, PR;

  Parameter min_p((double)999999);
  Parameter max_p((double)-999999);

  if (splitType == 0) {
    for (LineSegments::iterator it = begin; it != end; ++it) {
	  //pl(*it);
      if ((*it)->p0 != p && (*it)->p1 != p && XOrder((*it)->p0, p) == 1 && XOrder((*it)->p1, p) == 1) {
        TL++;
		if ((*it)->p0->getP().getX() < min_p) {
			min_p = (*it)->p0->getP().getX();
		}
		if ((*it)->p1->getP().getX() < min_p) {
			min_p = (*it)->p1->getP().getX();
		}
	  } else if ((*it)->p0 != p && (*it)->p1 != p && XOrder(p, (*it)->p0) == 1 && XOrder(p, (*it)->p1) == 1) {
	    TR++;
		if ((*it)->p0->getP().getX() > max_p) {
			max_p = (*it)->p0->getP().getX();
		}
		if ((*it)->p1->getP().getX() > max_p) {
			max_p = (*it)->p1->getP().getX();
		}
	  } else {
	    TL++;
		TR++;
		if ((*it)->p0->getP().getX() < min_p) {
			min_p = (*it)->p0->getP().getX();
		}
		if ((*it)->p1->getP().getX() < min_p) {
			min_p = (*it)->p1->getP().getX();
		}
		if ((*it)->p0->getP().getX() > max_p) {
			max_p = (*it)->p0->getP().getX();
		}
		if ((*it)->p1->getP().getX() > max_p) {
			max_p = (*it)->p1->getP().getX();
		}
  	  }
    }

	PL = (p->getP().getX() - min_p) / (max_p - min_p);
	PR = (max_p - p->getP().getX()) / (max_p - min_p);
  } else {
    for (LineSegments::iterator it = begin; it != end; ++it) {
      if ((*it)->p0 != p && (*it)->p1 != p && YOrder((*it)->p0, p) == 1 && YOrder((*it)->p1, p) == 1) {
        TL++;
		if ((*it)->p0->getP().getY() < min_p) {
			min_p = (*it)->p0->getP().getY();
		}
		if ((*it)->p1->getP().getY() < min_p) {
			min_p = (*it)->p1->getP().getY();
		}
	  } else if ((*it)->p0 != p && (*it)->p1 != p && YOrder(p, (*it)->p0) == 1 && YOrder(p, (*it)->p1) == 1) {
	    TR++;
		if ((*it)->p0->getP().getY() > max_p) {
			max_p = (*it)->p0->getP().getY();
		}
		if ((*it)->p1->getP().getY() > max_p) {
			max_p = (*it)->p1->getP().getY();
		}
	  } else {
	    TL++;
		TR++;
		if ((*it)->p0->getP().getY() < min_p) {
			min_p = (*it)->p0->getP().getY();
		}
		if ((*it)->p1->getP().getY() < min_p) {
			min_p = (*it)->p1->getP().getY();
		}
		if ((*it)->p0->getP().getY() > max_p) {
			max_p = (*it)->p0->getP().getY();
		}
		if ((*it)->p1->getP().getY() > max_p) {
			max_p = (*it)->p1->getP().getY();
		}
  	  }
    }

	PL = (p->getP().getY() - min_p) / (max_p - min_p);
	PR = (max_p - p->getP().getY()) / (max_p - min_p);
  }

  return log((double)TL) * PL.mid() + log((double)TR) * PR.mid();
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
