#include <gmapping/sensor/sensor_range/rangesensor.h>

namespace GMapping{

RangeSensor::RangeSensor(std::string name): Sensor(name){}

RangeSensor::RangeSensor(std::string name, unsigned int beams_num, double start_angle, double res, const OrientedPoint& lase_base_position):Sensor(name),
  laser_base_pose_(lase_base_position), m_ranges(beams_num){
  double angle = start_angle;
  for (unsigned int i=0; i<beams_num; i++, angle+=res){
    RangeSensor::Beam& beam(m_ranges[i]);
    beam.pose.x=0;
    beam.pose.y=0;
    beam.pose.theta=angle;
  }
}
};
