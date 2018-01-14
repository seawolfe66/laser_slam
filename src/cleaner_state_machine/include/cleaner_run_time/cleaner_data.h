#ifndef CLEANER_STATE_MACHINE_DATA_H_
#define CLEANER_STATE_MACHINE_DATA_H_
#include <cleaner_state_machine/cleaner_data_interface.h>

namespace cleaner_state_machine{
    /**
    * @class CleanerData
    * @brief Provides a class to realize the ICleanerDataInterface interface  for the use of data interaction functions between states.
    */
    class CleanerData: public ICleanerDataInterface{
    public:
        CleanerData();
        ~CleanerData();
        bool getCurrentArea(geometry_msgs::PolygonStamped& validPolygon);
        bool setCurrentArea(const geometry_msgs::PolygonStamped& validPolygon);
    private:
        geometry_msgs::PolygonStamped currentArea_;
    };

}   
#endif 
