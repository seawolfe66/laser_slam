#include <limits>
#include <iostream>
#include <assert.h>
#include <sys/types.h>
#include <gmapping/utils/gvalues.h>
#include <gmapping/sensor/sensor_range/rangereading.h>

namespace GMapping{

using namespace std;

RangeReading::RangeReading(const RangeSensor* rs, double time):
  SensorReading(rs,time){}

RangeReading::RangeReading(unsigned int n_beams, const double* d, const RangeSensor* rs, double time):
  SensorReading(rs,time){
  assert(n_beams==rs->beams().size());
  resize(n_beams);
  for (unsigned int i=0; i<size(); i++)
    (*this)[i]=d[i];
}

RangeReading::~RangeReading(){
}

unsigned int RangeReading::rawView(double* v, double density) const{
  if (density==0){
    for (unsigned int i=0; i<size(); i++)
      v[i]=(*this)[i];
  } else {
    Point lastPoint(0,0);
    uint suppressed=0;
    for (unsigned int i=0; i<size(); i++){
      const RangeSensor* rs=dynamic_cast<const RangeSensor*>(getSensor());
      assert(rs);
      Point lp(
        cos(rs->beams()[i].pose.theta)*(*this)[i],
        sin(rs->beams()[i].pose.theta)*(*this)[i]);
      Point dp=lastPoint-lp;
      double distance=sqrt(dp*dp);
      if (distance<density){
        //        v[i]=MAXDOUBLE;
        v[i]=std::numeric_limits<double>::max();
        suppressed++;
      }
      else{
        lastPoint=lp;
        v[i]=(*this)[i];
      }
      //std::cerr<< __PRETTY_FUNCTION__ << std::endl;
      //std::cerr<< "suppressed " << suppressed <<"/"<<size() << std::endl;
    }
  }
  //  return size();
  return static_cast<unsigned int>(size());

}
};

