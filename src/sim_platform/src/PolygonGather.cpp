
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>

#include "../include/sim_platform/geometry_tools.h"

#include <ros/wall_timer.h>

#include <visualization_msgs/Marker.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

namespace gather_polygon
{

/**
 * @brief Client for FrontierExplorationServer that receives control points from rviz, and creates boundary polygon for frontier exploration
 */
class PolygonGather
{

private:

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf::TransformListener  tf_;

  std::string frame_map_;
  std::string frame_robot_;

  ros::Subscriber point_sub;  // 
  ros::Publisher point_viz_pub_, polygon_planner_pub;
  ros::WallTimer point_gather_timer_;
  geometry_msgs::PolygonStamped input_;

  /**
   * @brief Publish markers for visualization of points for boundary polygon.
   */
  // void vizPubCb()
  // {

  //   visualization_msgs::Marker points, line_strip;

  //   points.header = line_strip.header = input_.header;
  //   points.ns = line_strip.ns = "explore_points";

  //   points.id = 0;
  //   line_strip.id = 1;

  //   points.type = visualization_msgs::Marker::SPHERE_LIST;
  //   line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  //   if (!input_.polygon.points.empty())
  //   {
  //     points.action = line_strip.action = visualization_msgs::Marker::ADD;
  //     points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

  //     points.scale.x = points.scale.y = 0.1;
  //     line_strip.scale.x = 0.05;

  //     for(int i = 0; i < input_.polygon.points.size(); i ++)
  //     {
  //       geometry_msgs::Point32 point = input_.polygon.points.at(i);
  //       line_strip.points.push_back(costmap_2d::toPoint(point));
  //       points.points.push_back(costmap_2d::toPoint(point));
  //     }

  //   points.color.a = points.color.b = line_strip.color.b = line_strip.color.a = 1.0;

  //   }
  //   else
  //   {
  //     points.action = line_strip.action = visualization_msgs::Marker::DELETE;
  //   }

  //   point_viz_pub_.publish(points);
  //   point_viz_pub_.publish(line_strip);
  // }

  // /**
  //  * @brief Build boundary polygon from points received through rviz gui.
  //  * @param point Received point from rviz
  //  */
  // void pointCb(const geometry_msgs::PointStampedConstPtr& point)
  // {

  //   double average_distance = polygonPerimeter(input_.polygon) / input_.polygon.points.size();

  //   if (input_.polygon.points.empty())
  //   {
  //     //first control point, so initialize header of boundary polygon

  //     input_.header = point->header;
  //     input_.polygon.points.push_back(costmap_2d::toPoint32(point->point));

  //   }
  //   else if (input_.header.frame_id != point->header.frame_id)
  //   {
  //     ROS_ERROR("Frame mismatch, restarting polygon selection");
  //     input_.polygon.points.clear();

  //   }
  //   else if (input_.polygon.points.size() > 1
  //       && pointsNearby(input_.polygon.points.front(), point->point, average_distance * 0.1))
  //   {
  //     //check if last boundary point, i.e. nearby to first point

  //     if (input_.polygon.points.size() < 3)
  //       ROS_ERROR("Not a valid polygon, restarting");
  //     else
  //       polygon_planner_pub.publish(input_);

  //     input_.polygon.points.clear();
  //   }
  //   else
  //   {
  //     //otherwise, must be a regular point inside boundary polygon
  //     input_.polygon.points.push_back(costmap_2d::toPoint32(point->point));
  //     input_.header.stamp = ros::Time::now();
  //   }

  // }
  /**
   * @brief Get the current position in the frame of map using transform.
   * @param The position of robot
   */
  int GetPoseOnMap(geometry_msgs::Pose& pose,std::string map_frame_id,std::string robot_frame_id)
  {
      double yaw, pitch, roll;
    try
      {
        tf_.waitForTransform(map_frame_id, robot_frame_id, ros::Time(), ros::Duration(0.5));

          tf::StampedTransform transform;
          tf_.lookupTransform(map_frame_id, robot_frame_id, ros::Time(), transform);
          std::cout.precision(3);
          std::cout.setf(std::ios::fixed,std::ios::floatfield);

          tf::Vector3 v = transform.getOrigin();
          tf::Quaternion q = transform.getRotation();	
          transform.getBasis().getRPY(roll, pitch, yaw);

          pose.position.x = v.getX();
          pose.position.y = v.getY();
          pose.position.z = v.getZ();

          return 0;

      }
      catch(tf::LookupException& ex)
      {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
        return -1;
      }
      
      catch(tf::ConnectivityException& ex) 
      {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
        return -1;
      }
      
      catch(tf::ExtrapolationException& ex) 
      {
        ROS_ERROR("Extrapolation Error: %s\n", ex.what());
        return -1;
      }
      catch(tf::TransformException& ex)
      {
          std::cout << "Failure at "<< ros::Time::now() << std::endl;
          std::cout << "Exception thrown:" << ex.what()<< std::endl;
          std::cout << "The current list of frames is:" <<std::endl;
          std::cout << tf_.allFramesAsString()<<std::endl;    
          return -1;
      }
  }

  /**
   * @brief Update the current position using transform.
   * @param NULL
   */
   void pointUpdateCb()
   {
     geometry_msgs::Pose newPose;
     if(0 == GetPoseOnMap(newPose,frame_map_,frame_robot_))
     {
        geometry_msgs::Point32 newPoint;
        newPoint.x = newPose.position.x;
        newPoint.y = newPose.position.y;
        newPoint.z = newPose.position.z;        
        double average_distance = polygonPerimeter(input_.polygon) / input_.polygon.points.size();
        ROS_INFO("POLYGONGATHER: average_distance = %.4f",average_distance);
        if (input_.polygon.points.empty())
        {
          //first control point, so initialize header of boundary polygon

          input_.header.frame_id = frame_map_;
          input_.polygon.points.push_back(newPoint);

        }
        else if (input_.header.frame_id != frame_map_)
        {
          ROS_ERROR("Frame mismatch, restarting polygon selection");
          input_.polygon.points.clear();

        }
        else if (input_.polygon.points.size() > 1
            && pointsNearby(input_.polygon.points.front(), newPoint, average_distance ))
        {
          //check if last boundary point, i.e. nearby to first point

          if (input_.polygon.points.size() < 3)
            ROS_ERROR("Not a valid polygon, restarting");
          else
           {
             ROS_INFO("POLYGONGATHER: publishing polygon");
          //   while(nh_.ok())
             {
               polygon_planner_pub.publish(input_);
               ros::spinOnce();
             }
             
           } 

          input_.polygon.points.clear();
        }
        else//otherwise, must be a regular point inside boundary polygon
        {
          // Judge the distance with last point
          geometry_msgs::Point32 lastPoint;
          lastPoint = input_.polygon.points.back();
          float distance = pointsDistance(newPoint,lastPoint);
          if(distance > 0.2)
          {
            input_.polygon.points.push_back(newPoint);
            input_.header.stamp = ros::Time::now();
            ROS_INFO("POLYGONGATHER: Get new poistion(%.2f,%.2f,%.2f)",newPose.position.x,newPose.position.y,newPose.position.z);

          }
        }
      }
   }
public:
  /**
   * @brief Constructor for the client.
   */
  PolygonGather() :
      nh_(), private_nh_("~")
  {
    frame_map_ = "map";
    frame_robot_ = "base_footprint";

    point_gather_timer_ = nh_.createWallTimer(ros::WallDuration(0.1),
                                              boost::bind(&PolygonGather::pointUpdateCb, this));

    polygon_planner_pub = nh_.advertise<geometry_msgs::PolygonStamped>("heatmap_area", 10);
          
    // input_.header.frame_id = frame_map;
    // point_sub = nh_.subscribe("/odom", 10, &PolygonGather::pointCb, this);
    // polygon_planner_pub = nh_.advertise<geometry_msgs::PolygonStamped>("polygon_area", 10);
    // point_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("gather_polygon_marker", 10);
    // point_viz_timer_ = nh_.createWallTimer(ros::WallDuration(0.1),
    //                                        boost::bind(&PolygonGather::vizPubCb, this));
    // point_gather_timer_ = nh_.createWallTimer(ros::WallDuration(0.1),
    //                                        boost::bind(&PolygonGather::pointUpdateCb, this));
    // ROS_INFO("Please use the 'Polygon' tool in Rviz to select a polygon boundary.");
  }

};

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PolygonGather");

  gather_polygon::PolygonGather client;
  ros::spin();
  return 0;
}
