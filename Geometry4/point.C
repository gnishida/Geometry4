#include "point.h"

int XOrder::sign ()
{
  return (b->getP().x - a->getP().x).sign();
}

int LineSegmentXOrder::sign ()
{
  return XOrder(a->p0, b) && XOrder(a->p1, b);
}

int YOrder::sign ()
{
  return (b->getP().y - a->getP().y).sign();
}

int LineSegmentYOrder::sign ()
{
  return YOrder(a->p0, b) && YOrder(a->p1, b);
}

int CCW::sign ()
{
  return a->getP().cross(b->getP()).sign();
}

int LeftTurn::sign ()
{
  return (c->getP() - b->getP()).cross(a->getP() - b->getP()).sign();
}

PV2 lineIntersection (const PV2 &a, const PV2 &b, const PV2 &c, const PV2 &d)
{
  PV2 u = b - a, v = d - c;
  Parameter k = (c - a).cross(v)/u.cross(v);
  return a + k*u;
}

void pp (Point *p)
{
  PV2 pp = p->getP();
  cerr << setprecision(16) << "(" << pp.x.mid() << " " 
       << pp.y.mid() << ")" << endl;
}
