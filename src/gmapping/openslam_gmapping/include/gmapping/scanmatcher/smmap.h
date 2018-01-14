#ifndef SCAN_MATCH_MAP_H
#define SCAN_MATCH_MAP_H
#include <gmapping/grid/map.h>
#include <gmapping/grid/harray2d.h>
#include <gmapping/utils/point.h>
#define SIGHT_INC 1

namespace GMapping {

//! map cell using float for memory saving
struct PointAccumulator{

  PointAccumulator(): acc(0,0), n(0), visits(0){}
  PointAccumulator(int i): acc(0,0), n(0), visits(0){assert(i==-1);}

  inline void update(bool value, const Point& p=Point(0,0));
  inline Point mean() const {return 1./n*Point(acc.x, acc.y);}
  inline operator double() const { return visits?(double)n*SIGHT_INC/(double)visits:-1; }
  inline void add(const PointAccumulator& p) {acc=acc+p.acc; n+=p.n; visits+=p.visits; }
  inline double entropy() const;

  //! we accumulate the obstacle pose under reference frame of world every time we observate it , and keep it in related cell.
  FloatPoint acc;
  //! we can only keep up to  65535 times of observation ,then overflow.
  unsigned short n, visits;

};

void PointAccumulator::update(bool value, const Point& p){
  if (value) {
    acc.x+= static_cast<float>(p.x);
    acc.y+= static_cast<float>(p.y);
    n++;
    visits+= SIGHT_INC;
  } else
    visits++;
}

double PointAccumulator::entropy() const{
  if (!visits)
    return -log(.5);
  if (n==visits || n==0)
    return 0;
  double x=(double)n*SIGHT_INC/(double)visits;
  return -( x*log(x)+ (1-x)*log(1-x) );
}


typedef Map<PointAccumulator,HierarchicalArray2D<PointAccumulator> > ScanMatcherMap;

};

#endif 
