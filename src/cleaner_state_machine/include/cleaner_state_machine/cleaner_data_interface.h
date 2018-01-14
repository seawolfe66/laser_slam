#ifndef CLEANER_STATE_MACHINE_DATA_INTERFACE_H_
#define CLEANER_STATE_MACHINE_DATA_INTERFACE_H_
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>

namespace cleaner_state_machine{
    enum RESULT {SUCCESS,
             FAIL,
             NEXT,
             ALTERNATIVE,
             WAIT};
    /**
    * @interface ICleanerDataInterface
    * @brief Provides an interface for the use of data interaction functions between states.
    */
    class ICleanerDataInterface{
    public:
        virtual bool getCurrentArea(geometry_msgs::PolygonStamped& validPolygon) = 0;
        virtual bool setCurrentArea(const geometry_msgs::PolygonStamped& validPolygon) = 0;
    };

}   
#endif 


