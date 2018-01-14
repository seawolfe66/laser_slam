#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PolygonStamped.h>

#include <frontier_exploration/ExploreTaskAction.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>
#include <frontier_exploration/CostmapPolygon.h>

#include <tf/transform_listener.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <frontier_exploration/geometry_tools.h>
#include <glog/logging.h>
#include <glog/vlog_is_on.h>

namespace frontier_exploration{

/**
 * @brief Server for frontier exploration action, runs the state machine associated with a
 * structured frontier exploration task and manages robot movement through move_base.
 */
class FrontierExplorationServer
{

public:

    /**
     * @brief Constructor for the server, sets up this node's ActionServer for exploration and ActionClient to move_base for robot movement.
     * @param name Name for SimpleActionServer
     */
    FrontierExplorationServer(std::string name) :
        tf_listener_(ros::Duration(10.0)),
        private_nh_("~"),
        as_(nh_, name, boost::bind(&FrontierExplorationServer::executeCb, this, _1), false),
       // move_client_("move_base",true),
        retry_(500)
    {

        private_nh_.param<double>("frequency", frequency_, 0.0);
        private_nh_.param<double>("goal_aliasing", goal_aliasing_, 0.1);
//publish RectanglePolygon
//	rectanglePolygon_pub_ = private_nh_.advertise<geometry_msgs::PolygonStamped>("RectanglePolygon",50);

        explore_costmap_ros_ = boost::shared_ptr<costmap_2d::Costmap2DROS>(new costmap_2d::Costmap2DROS("explore_costmap", tf_listener_));

        as_.registerPreemptCallback(boost::bind(&FrontierExplorationServer::preemptCb, this));
        as_.start();
        google::InitGoogleLogging("explore_server");

        FLAGS_log_dir = "/home/evan/catkin_ws/src/frontier_exploration/logs";
        FLAGS_alsologtostderr =  false;  //设置日志消息除了日志文件之外是否去标准输出
        FLAGS_colorlogtostderr = true;  //设置记录到标准输出的颜色消息（如果终端支持）
        FLAGS_v = 5;
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    tf::TransformListener tf_listener_;
    actionlib::SimpleActionServer<frontier_exploration::ExploreTaskAction> as_;

    boost::shared_ptr<costmap_2d::Costmap2DROS> explore_costmap_ros_;
    double frequency_, goal_aliasing_;
    bool success_, moving_;
    int retry_;

    //boost::mutex move_client_lock_;
    frontier_exploration::ExploreTaskFeedback feedback_;
   // actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_client_;
  //  move_base_msgs::MoveBaseGoal move_client_goal_;

    /**
     * @brief Execute callback for actionserver, run after accepting a new goal
     * @param goal ActionGoal containing boundary of area to explore, and a valid centerpoint for the area.
     */

    void executeCb(const frontier_exploration::ExploreTaskGoalConstPtr &goal)
    {

        success_ = false;
        moving_ = false;

        explore_costmap_ros_->resetLayers();

        //create costmap services
        ros::ServiceClient updateCostmapBoundaryPolygon = private_nh_.serviceClient<frontier_exploration::CostmapPolygon>("explore_costmap/explore_boundary/get_costmap_polygon");
        if(!updateCostmapBoundaryPolygon.waitForExistence()){

            as_.setAborted();
            return;
        }

        if(ros::ok() && as_.isActive()){
            frontier_exploration::CostmapPolygon srv;
            srv.request.costmap_boundary = goal->explore_boundary;
	    //input_rectangle_polygon_
	   // sweeps_areas_pub_.publish(sweeps_areas_);

            if(updateCostmapBoundaryPolygon.call(srv)){
                ROS_INFO("Region boundary set");
                 as_.setSucceeded();
            }else{
                ROS_ERROR("Failed to set region boundary");
                as_.setAborted();
                return;
            }
        }

   }


    /**
     * @brief Preempt callback for the server, cancels the current running goal and all associated movement actions.
     */
    void preemptCb(){
        if(as_.isActive()){
            as_.setPreempted();
        }

    }

    /**
     * @brief Feedback callback for the move_base client, republishes as feedback for the exploration server
     * @param feedback Feedback from the move_base client
     */
    void feedbackMovingCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){

        feedback_.base_position = feedback->base_position;
        as_.publishFeedback(feedback_);

    }

    /**
     * @brief Done callback for the move_base client, checks for errors and aborts exploration task if necessary
     * @param state State from the move_base client
     * @param result Result from the move_base client
     */
    void doneMovingCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result){

        if (state == actionlib::SimpleClientGoalState::ABORTED){
            ROS_ERROR("Failed to move");
            as_.setAborted();
        }else if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
            moving_ = false;
        }

    }

};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "explore_server");


    frontier_exploration::FrontierExplorationServer server(ros::this_node::getName());
    ros::spin();
    return 0;
}
