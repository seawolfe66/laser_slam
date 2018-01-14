#ifndef RANGESENSOR_H
#define RANGESENSOR_H

#include <vector>
#include <gmapping/sensor/sensor_base/sensor.h>
#include <gmapping/utils/point.h>

namespace GMapping{

/*!RangSensor class for sensors like laser
* usually ,we mount laser on base of robot , every laser has it's own m_pose, meaning laser pose relative base,
* in scanmatcher ,when we compute one hit beam ,
* we only need laser_map_pose ,range from reading, angle table this three parameters to compute the hit point in map.
* laser_map_pose is transformed using lase_base_pose and base_map_pose, the base_map_pose is estimated using particles.
*/
class RangeSensor: public Sensor{

    public:
        struct Beam{
            OrientedPoint pose; //pose relative to the center of the sensor
        };
        RangeSensor(std::string name);
        RangeSensor(std::string name, unsigned int beams, double start_angle, double res, const OrientedPoint& lase_base_position=OrientedPoint(0,0,0));
        inline const std::vector<Beam>& beams() const {return m_ranges;}
        inline std::vector<Beam>& beams() {return m_ranges;}
        inline OrientedPoint getPose() const {return laser_base_pose_;}

    protected:
        OrientedPoint laser_base_pose_;  //<! the laser pose in base_frame, or laser pose relative base.
        std::vector<Beam> m_ranges; //<! range angle table.

        //! todo , remove beam ,using below.
        //double start_angle;
        //double resolution;
};

};

#endif
