#include <cleaner_run_time/cleaner_data.h>

namespace cleaner_state_machine{

    CleanerData::CleanerData()
    {

    }
    CleanerData::~CleanerData()
    {
            
    }
    bool CleanerData::getCurrentArea(geometry_msgs::PolygonStamped& validPolygon)
    {
        validPolygon = currentArea_;
        return true;
    }
    bool CleanerData::setCurrentArea(const geometry_msgs::PolygonStamped& validPolygon)
    {
        currentArea_ = validPolygon;
        return true;
    }
}
