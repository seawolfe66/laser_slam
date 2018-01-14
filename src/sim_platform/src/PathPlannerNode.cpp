

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/String.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include "nav_msgs/Path.h"
#include <hector_move_base_msgs/MoveBaseGoal.h>
#include <hector_move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include "std_srvs/Empty.h"
//#include </usr/local/include/geos.h>
#include <geos.h>
#include <geos/geom/GeometryFactory.h>

#include <std_srvs/Trigger.h>

#include <boost/foreach.hpp>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "actionlib/client/simple_goal_state.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "actionlib/client/terminal_state.h"

#include "sim_platform/bowstatesResponse.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include "Eigen/Eigen"
#include <vector>
//#include <costmap_2d/costmap_2d.h>
using namespace Eigen;
using namespace std;

namespace path_planner{

//#pragma comment(lib,LIB_XD("geos_3_62"))


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client; 

struct bowstatesResponse{

    bool is_bowstate;
    geometry_msgs::PoseStamped start;
    geometry_msgs::PoseStamped goal;

}; 

class PathPlannerNode {

public :

PathPlannerNode(){

    ros::NodeHandle nh_("path_planner");
    nh_.param("~coverage_spacing", cut_spacing, 0.2); 
    heapmap_sub_ = nh_.subscribe("heatmap_area", 5, &PathPlannerNode::field_callback, this);
    path_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker",100);
    odom_sub_ = nh_.subscribe("/odom", 5, &PathPlannerNode::odom_callback, this);
    path_pub = nh_.advertise<nav_msgs::Path>("visualization_path",100);
    //Setup service for gmapping

    serviceLineProvider = nh_.advertiseService("provide_line", &PathPlannerNode::get_start_end_line, this);
    bow_pub = nh_.advertise<std_msgs::String>("bow_command",10);
     //clear_costmaps = rospy.ServiceProxy('move_base/clear_costmaps', Empty);
    nh_.serviceClient<std_srvs::Trigger>("move_base/clear_costmaps");

    factory = geos::geom::GeometryFactory::create();

    field_frame_id = "";
    testing = false;
    current_distance = 0; 
    just_reset = false; 
    timeout = false;
    start_path_following = false;
    get_origin_flag = false;
    init();

}
~PathPlannerNode(){

    delete move_base_client ; 
    move_base_client = NULL; 

}


void init(){

    previous_destination.target_pose.pose.position.x = -1; 
    previous_destination.target_pose.pose.position.y = -1; 
    previous_destination_flag = false;
    get_robot_pose_flag = false;
    //clear_costmaps = rospy.ServiceProxy('move_base/clear_costmaps', Empty);
    //mNode.serviceClient<std_srvs::Trigger>(NAV_STOP_SERVICE);
    move_base_client = new client("move_base", true);
    ros::Rate rate(10);
    while (ros::ok())
    {
      rate.sleep();
      if(start_path_following){
           heading = 0;
           setup_path_following(heading);
           while(ros::ok()){
                if(!step_path_following())
                    break; 

           } 
           start_path_following = false; 

      }
      ros::spinOnce();

    } 
}

private:
    void field_callback(const geometry_msgs::PolygonStamped & input_){
        field_shape  = input_; 
        field_frame_id = input_.header.frame_id; 
        start_path_following = true;


    }

  
    std::vector<float> calculate_headings(nav_msgs::Path & path){
        std::vector<float> new_path;

       //BOOST_FOREACH(geometry_msgs::PointStamped pose, path.poses){
        for(int i = 0; i< path.poses.size(); i++){
            if(i == 0){
                new_path.push_back(0);
                continue;
            }
            geometry_msgs::PoseStamped pose0 = path.poses[i];
            geometry_msgs::PoseStamped pose1 = path.poses[i-1];
            float dx = pose0.pose.position.x -  pose1.pose.position.x;
            float dy = pose0.pose.position.y -  pose1.pose.position.y;
            float heading = atan2(dy, dx);
            new_path.push_back(heading);
       }
        return new_path;
    }

    void odom_callback(const nav_msgs::Odometry& msg){
        robot_pose = msg; 
        get_robot_pose_flag = true;
    }
   
    void visualize_path_as_path( nav_msgs::Path & path, std::vector<std::string> status){
        nav_msgs::Path msg ; 
        msg.header.stamp = ros::Time::now(); 
        msg.header.frame_id = field_frame_id;

        for(int i = 0; i< path.poses.size(); i++){
            if (status.size() > 0){
               if( status[i] == "visited"){ 
                   if ( i+1 < status.size() ){
                       if (status[i+1] == "visited")
                                continue; 

                   }

               }
            }

            geometry_msgs::PoseStamped pose_msg; 
            pose_msg.header.stamp = msg.header.stamp; 
            pose_msg.header.frame_id = field_frame_id; 
            pose_msg.pose.position.x = path.poses[i].pose.position.x; //waypoint[0]
            pose_msg.pose.position.y = path.poses[i].pose.position.y;//waypoint[1]
            msg.poses[i] = pose_msg; 

        }
        path_pub.publish(msg);
       

    }

    void visualize_path_as_marker(nav_msgs::Path & path, std::vector<std::string> status){
        ros::Time now = ros::Time::now();
        for(int index = 0; index< path.poses.size(); index++){
            visualization_msgs::Marker waypoint_marker; 
            //
            waypoint_marker.header.stamp = now; 
            waypoint_marker.header.frame_id = field_frame_id; 
            waypoint_marker.ns = "waypoints"; 
            waypoint_marker.id = index; 
            waypoint_marker.type = visualization_msgs::Marker::ARROW;
            if( index == 0)
                waypoint_marker.type = visualization_msgs::Marker::CUBE; 
            waypoint_marker.action = visualization_msgs::Marker::MODIFY; 
            waypoint_marker.scale.x = 1;
            waypoint_marker.scale.y = 1;
            waypoint_marker.scale.z = 0.25; 

            geometry_msgs::Point point ; 
            point.x = path.poses[index].pose.position.x;
            point.y = path.poses[index].pose.position.y;
            point.z = 0; //path.poses[index].pose.position.z;
   
            waypoint_marker.pose.position = point;

            line_strip_points.push_back(point); 

            tf::Quaternion quat;
            quat.setEuler(0,0, path.poses[index].pose.position.z);

            waypoint_marker.pose.orientation.x = quat.x();
            waypoint_marker.pose.orientation.y = quat.y(); 
            waypoint_marker.pose.orientation.z = quat.z(); 
            waypoint_marker.pose.orientation.w = quat.w(); 

            std::string status1 = status[index];
            if (status1 == "not_visited")
              setColor(waypoint_marker, 1,0,0,0.5);
            else if (status1 == "visiting")
               setColor(waypoint_marker,0,1,0,0.5);
            else if( status1 == "visited")
               setColor(waypoint_marker,0,0,1,0.5);
            else{
                ROS_ERROR("Invalid path status.");
                setColor(waypoint_marker,1,1,1,0.5);
                }
            path_markers.markers.push_back(waypoint_marker);

        }

        visualization_msgs::Marker line_strip; 
        line_strip.header.stamp = now; 
        line_strip.header.frame_id = field_frame_id; 
        line_strip.ns = "lines"; 
        line_strip.id = 0; 
        line_strip.type = visualization_msgs::Marker::LINE_STRIP; 
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.scale.x = 0.1; 
        //line_strip.color = ColorRGBA(0,0,1,0.5)
        setColor(line_strip, 0,0,1,0.5);
        line_strip.points = line_strip_points;
        path_markers.markers.push_back(line_strip);
        path_marker_pub.publish(path_markers);


    }

    void setColor(visualization_msgs::Marker & maker, float r, float g, float b, float a){
   
        maker.color.r = r; 
        maker.color.g = g; 
        maker.color.b = b ; 
        maker.color.a = a; 
    }



    void visualize_path( nav_msgs::Path & path, std::vector<std::string> status ){

        visualize_path_as_path(path, status);
        visualize_path_as_marker(path, status);
    }


    void goal_rotate(){
        move_base_msgs::MoveBaseGoal destination;
        destination.target_pose.header.frame_id = field_frame_id;
        destination.target_pose.header.stamp = ros::Time::now();
        destination.target_pose.pose.position.x = -0.54;
        destination.target_pose.pose.position.y = 2;
        float rotate = 0;
        connected_to_move_base = false;
        ros::Duration(1.0);

        while( ! connected_to_move_base){
            sleep(1.0);
            if(!ros::ok()) return;
        }
    
        tf::Quaternion quat;         
         while (ros::ok()){
            rotate += 1.57; 

            quat.setEuler(0, 0, rotate);

            destination.target_pose.pose.orientation.x = quat.x();
            destination.target_pose.pose.orientation.y = quat.y();
            destination.target_pose.pose.orientation.z = quat.z();
            destination.target_pose.pose.orientation.w = quat.w();


            move_base_client->sendGoal(destination);
            move_base_client->waitForResult(ros::Duration(1.0));
           
            ros::Rate(1).sleep();
         }


    }



    void setup_path_following( float degrees=0){
        connected_to_move_base = false;    
        ros::Duration dur(1.0);
        if(testing){

            robot_pose.pose.pose.position.x = 0;
            robot_pose.pose.pose.position.y = 0;
        }
        while(field_shape.polygon.points.empty()){
            if(!ros::ok()) return;
            ROS_INFO("Qualification: waiting on the field shape");
            ros::Rate(1.0).sleep();
        }
       // while(robot_pose.pose.pose.position.x == 0 && robot_pose.pose.pose.position.y == 0){ //robot_pose is none
        while(get_robot_pose_flag == false){
             if(!ros::ok()) return;
             ROS_INFO("Qualification: waiting on initial robot pose.");
             ros::Rate(1.0).sleep(); 

        }
       // geometry_msgs::Point origin;
        //origin.x = robot_pose.pose.pose.position.x;
        //origin.y = robot_pose.pose.pose.position.y;

        geos::geom::Point * origin;
        geos::geom::Coordinate coord;
        coord.x = robot_pose.pose.pose.position.x;
        coord.y = robot_pose.pose.pose.position.y;
        origin = factory->createPoint(coord);




        //Coordinate* origin_xy = origin->getCoordinate();
       // origin->
       //origin_xy->x = robot_pose.pose.pose.position.x;
       //origin_xy->y = robot_pose.pose.pose.position.y;
        get_origin_flag = true;

        plan_path(field_shape, origin, degrees);
        ROS_INFO("Path Planner: path planning complete.");
        while(!connected_to_move_base){
            connected_to_move_base = move_base_client->waitForServer(dur);
            if (connected_to_move_base){
                std_msgs::String msg;
                std::stringstream ss;
                ss << "bow";
                msg.data = ss.str();
                bow_pub.publish(msg); //error
            }
            if(!ros::ok()) return;
            ROS_INFO("Path Planner: waiting on move_base");
       }
       return;

    }

 int get_next_waypoint_index(){
    for(int index = 0; index< path_status.size(); index++){        
        std::string status = path_status.at(index);
        if(status == "visited")
            continue;
        if(status == "visiting"){
            return index; 
        }
        if(status == "not_visited")
           return index; 
     }    

   }

    float distance(move_base_msgs::MoveBaseGoal & p1, move_base_msgs::MoveBaseGoal & p2 ){
        float dx = p2.target_pose.pose.position.x - p1.target_pose.pose.position.x;
        float dy = p2.target_pose.pose.position.y - p1.target_pose.pose.position.y; 
        return sqrt(dx*dx + dy*dy); 

    }
    bool step_path_following(){
        bool ret = false; 
        visualize_path(path, path_status);
         
        int current_waypoint_index = -1; 
        current_waypoint_index =  get_next_waypoint_index(); 
        if (current_waypoint_index == -1){
            ROS_INFO("Path Planner: Done.");
            return false;
        }
        
        if( current_waypoint_index == 0)
            path_status[current_waypoint_index] = "visited"; 

        geometry_msgs::PoseStamped current_waypoint = path.poses[current_waypoint_index];
        std::string current_waypoint_status = path_status[current_waypoint_index]; 
        //If the status is visited
        if (current_waypoint_status == "visited")
            return true;
        if (current_waypoint_status == "not_visited"){

            path_status[current_waypoint_index] = "visiting"; 
            move_base_msgs::MoveBaseGoal destination;
            destination.target_pose.header.frame_id = field_frame_id;
            destination.target_pose.header.stamp = ros::Time::now();
            // Set the target location
            destination.target_pose.pose.position.x = current_waypoint.pose.position.x;
            destination.target_pose.pose.position.y = current_waypoint.pose.position.y;
            //rotate = current_waypoint[2]; //perhaps error? 
            float rotate = 0; 
           // if (previous_destination.target_pose.pose.position.x == -1 && previous_destination.target_pose.pose.position.y == -1)
            if(previous_destination_flag == false){
                current_distance = 5.0;
            }else{
                current_distance = distance(previous_destination, destination); 
               // from math import atan2
                rotate = atan2(destination.target_pose.pose.position.y-previous_destination.target_pose.pose.position.y,
                                 destination.target_pose.pose.position.x - previous_destination.target_pose.pose.position.x) ; 
                } 



            tf::Quaternion quat;
            quat.setEuler(0, 0, rotate);

            destination.target_pose.pose.orientation.x = quat.x();
            destination.target_pose.pose.orientation.y = quat.y();
            destination.target_pose.pose.orientation.z = quat.z();
            destination.target_pose.pose.orientation.w = quat.w();

            move_base_msgs::MoveBaseGoal r_dest; 


           
            if(previous_destination_flag == true){
                r_dest = previous_destination;
                r_dest.target_pose.pose.orientation = destination.target_pose.pose.orientation;
            }
            else{
                r_dest = destination;
            }
            move_base_client->sendGoal(r_dest);
            if(move_base_client->waitForResult() == true)
                ROS_ERROR("Sending rotation ");
            current_destination = destination;
            move_base_client->sendGoal(destination);
            previous_destination = destination;
            previous_destination_flag == true;


        }
        actionlib::SimpleClientGoalState state = move_base_client->getState();
        std::string temp_state = state.getText();
        if( current_waypoint_status == "visiting") {

             if (temp_state == "ABORTED" || temp_state == "SUCCEEDED" )
                 path_status[current_waypoint_index] = "visited";
             else 
             {
                 ros::Duration duration(2.0);
                 int count = 0; 
                 move_base_client->waitForResult(); 
                 if (timeout == true){
                     if ( count == 6+int(current_distance*4) )
                     {

                          ROS_INFO("Path Planner: move_base goal timeout occurred, clearing costmaps");
                          move_base_client->cancelAllGoals();

                         std_srvs::Trigger srv;
                         if(!clear_costmaps.call(srv))
                         {
                            ROS_ERROR("Failed to send STOP_COMMAND to Navigator.");
                         }

                        // Wait 1 second
                        ros::Rate(1.0).sleep();
                        if(!just_reset){
                            just_reset = true;
                            path_status[current_waypoint_index] = "not_visited"; 
                        }
                        else
                        {
                            just_reset = false;
                            path_status[current_waypoint_index] = "visited" ; 
                        }
                        return true; 


                     }
                 }
                 path_status[current_waypoint_index] = "visited";

             }


        }
          

        return true; 
    }



      bool get_start_end_line(sim_platform::bowstatesResponse::Request &req, sim_platform::bowstatesResponse::Response &res) {
        int current_waypoint_index = -1;
        current_waypoint_index = get_next_waypoint_index();

       // bowstatesResponse res; 
        if( current_waypoint_index == -1){
            ROS_INFO("Path Planner: Done.");
            res.is_bowstate = false;
            return false;
        }

        geometry_msgs::PoseStamped current_waypoint = path.poses[current_waypoint_index];//path[current_waypoint_index];
        std::string current_waypoint_status = path_status[current_waypoint_index];

        if (current_waypoint_index == 0){
            res.is_bowstate = true; 
            return true;
        }

         if(current_waypoint_index == 1){
            res.is_bowstate = true; 
            res.start=req.pos; 
            move_base_msgs::MoveBaseGoal end;

            end.target_pose.header.frame_id = field_frame_id; 
            end.target_pose.header.stamp = ros::Time::now();
            // Set the target location
            end.target_pose.pose.position.x = current_waypoint.pose.position.x;
            end.target_pose.pose.position.y = current_waypoint.pose.position.y;
            res.goal = end.target_pose;
            return true;  

         }
         if (current_waypoint_status == "visited"){
             return false;

         }
         if (current_waypoint_status == "not_visited"){

             move_base_msgs::MoveBaseGoal destination;
             destination.target_pose.header.frame_id = field_frame_id;
             destination.target_pose.header.stamp = ros::Time::now();

             destination.target_pose.pose.position.x = current_waypoint.pose.position.x;
             destination.target_pose.pose.position.y = current_waypoint.pose.position.y; 
             res.is_bowstate = true;

           
             if(previous_destination_flag == false)
                res.start = req.pos;
             else{
                res.start = previous_destination.target_pose;
                res.goal   = destination.target_pose ;
             }
 

               return true; 


         }
        
        return true;
    }
    void plan_path(const geometry_msgs::PolygonStamped &field_polygon, geos::geom::Point * origin, float degrees){
        // This is called after a field polygon has been received.
        //Get the rotation to align with the longest edge of the polygon
         Eigen::Matrix3f rotation;
        rotation = rotation_tf_from_longest_edge(field_polygon);
        //rotation = RotationTransform(rotation.w + degrees)
        //this may be not right//
       // Eigen::Quaterniond rotation_q_;
        //rotation_q_(0)=sqrt(rotation.trace()+1)/2;
        //rotation_q_(1)=(rotation(1,2)-rotation(2,1))/(4*rotation_q_(0));
       // rotation_q_(2)=(rotation(2,0)-rotation(0,2))/(4*rotation_q_(0));
        //rotation_q_(3)=(rotation(0,1)-rotation(1,0))/(4*rotation_q_(0));
        //rotation = RotationTransform(tf::getYaw(rotation_q_) + degrees);
        float q = sqrt(rotation.trace()+1)/2;
        rotation = RotationTransform(q + degrees);
        //Rotate the field polygon
        geometry_msgs::PolygonStamped transformed_field_polygon;
        rotate_polygon_to(field_polygon, rotation);
        if(get_origin_flag == true){
            Eigen::MatrixXf point_mat(1,3);

            point_mat(0,0) = origin->getX();
            point_mat(0,1) = origin->getY();
            point_mat(0,2) = 0;
            Eigen::Matrix3f origin_;
            origin_ = rotation * point_mat.transpose();
            //geos::geom::Point * origin;
            geos::geom::Coordinate coord_;
            coord_.x = origin_(0,0);
            coord_.y = origin_(1,0);
            origin = factory->createPoint(coord_);

           // origin->getX() = origin_(0,0);
           // origin->getY() = origin_(1,0);

        }
        vector< geos::geom::Point * >* transformed_path = decompose(transformed_field_polygon, origin, cut_spacing);
        //Eigen::ArrayXXf transformed_path = decompose(transformed_field_polygon, origin, cut_spacing);

        //Rotate the transformed path back into the source frame
        //Eigen::ArrayXXf path = rotate_from(np.array(transformed_path), rotation);
        //Eigen::ArrayXXf path;
         //nav_msgs::Path path;
       //nav_msgs::Path path = rotate_from(transformed_path, rotation);
       nav_msgs::Path path = rotate_from(transformed_path, rotation);
        //Calculate headings and extend the waypoints with them
        //path = calculate_headings(path);
        std::vector<float> head_path = calculate_headings(path);
        // Set the path_status to 'not_visited'
       std::vector<std::string> path_status;


        for(int i = 0; i<head_path.size(); i++){
            path_status.push_back("not_visited");
        }
       // std::vector<std::string> path_status ;


        // Visualize the data
       visualize_path(path, path_status);
        return;


    }





    nav_msgs::Path rotate_from(vector< geos::geom::Point * >* points, Eigen::Matrix3f rotation_transform){
        if(points->size()<3){
            ROS_ERROR("rotate_from: takes an numpy.ndarray");
        }

        nav_msgs::Path points_path;
        points_path.header.stamp = ros::Time::now();
        points_path.header.frame_id = field_frame_id;
        vector< geos::geom::Point * >::iterator it;
        int j = 0;

        for(it = points->begin(); it!=points->end(); it++){
            geos::geom::Point * alone_point;
            alone_point = *it;
            //alone_point->getX()
            Eigen::VectorXf point_mat(3);
            //Eigen::MatrixXd point_mat(3,1);
            point_mat(0) = alone_point->getX();
            point_mat(1) = alone_point->getY();
            point_mat(2) = 0;
           // Eigen::Matrix3d transform = Eigen::MatrixXd::Random(3,3);
            Eigen::VectorXf new_point(3);
           // new_point = transform.inverse() * point_mat;
           // Eigen::MatrixXd new_point(3,1);
            //new_point = rotation_transform * point_mat;
             new_point = rotation_transform.inverse() * point_mat;
             Eigen::VectorXf current_point(4);
            current_point(0) = new_point(0,0);
            current_point(1) = new_point(1,0);
            current_point(2) = new_point(2,0);
            current_point(3) = -1;
            current_point = current_point.transpose();
            points_path.poses[j].pose.orientation.x= current_point(0);
            points_path.poses[j].pose.orientation.y = current_point(1);
            points_path.poses[j].pose.orientation.z = current_point(2);
            points_path.poses[j].pose.orientation.w = current_point(3);
            j = j + 1;

        }

        if(points_path.poses.size()==1){
            points_path.poses.clear();
            return points_path;
        }else{

              return points_path;
        }
        //return points_path;

    }


    /*geometry_msgs::Point selected;
          std::vector<geometry_msgs::Point> tempfrontierlist(frontier_list_new);
          frontier_list_new.clear();

          std::vector<geometry_msgs::Point>::iterator it;
          std::vector<geometry_msgs::Point>::iterator cancel;

          if(number == 0){
              ROS_ERROR("No frontiers found, return false");
              return false;
          }

          geometry_msgs::PointStamped in, out;
          selected = tempfrontierlist.front();

          frontier_list_copy.push_back(selected); //push the first element into the orderlist
          tempfrontierlist.erase(tempfrontierlist.begin());

          double distance = 0;
          int index = 0;

          while(!tempfrontierlist.empty()){

              geometry_msgs::Point last = frontier_list_copy.back();
              geometry_msgs::Point first = tempfrontierlist.front();

              distance = sqrt(pow((double(last.x)-double(first.x)),2.0) + pow((double(last.y)-double(first.y)),2.0));
              cancel = tempfrontierlist.begin();

              for(it = tempfrontierlist.begin(); it!=tempfrontierlist.end(); it++){

                  double temp = sqrt(pow((double(last.x)-double((*it).x)),2.0) + pow((double(last.y)-double((*it).y)),2.0));

                  if (temp <= distance){
                     selected = *it;
                     distance = temp;
                     cancel = it;
                  }
               }
               frontier_list_copy.push_back(selected);
               tempfrontierlist.erase(cancel);

            }

    nav_msgs::Path rotate_from(vector< geos::geom::Point * >* points, Eigen::Matrix3f rotation_transform){

        if(points->size()<3){
            ROS_ERROR("rotate_from: takes an numpy.ndarray");
        }
        nav_msgs::Path new_points;

        for(int i = 0; i<points->size(); i++){
            //float point_mat[3][1];
            Eigen::Vector3f point_mat;
            point_mat(0) = points
            point_mat(1) = points(1,i);
            point_mat(2) = 0;
            Eigen::VectorXf new_point(3);
            new_point = rotation_transform.inverse() * point_mat;
            Eigen::VectorXf current_point(4);
            current_point(0) = new_point(0);
            current_point(1) = new_point(1);
            current_point(2) = new_point(2);
            current_point(3) = -1;
            current_point = current_point.transpose();
            new_points(0,i) = current_point(0);
            new_points(1,i) = current_point(1);
            new_points(2,i) = current_point(2);
            new_points(3,i) = current_point(3);

        }
        if(new_points.cols()==1 || new_points.rows()==1){
            return;
        }else{
            return ;
                    //return new_points;
        }

    }*/

    vector< geos::geom::Point * >* decompose(geometry_msgs::PolygonStamped polygon, geos::geom::Point * origin , double width){
            //Decompose the field into a list of points to cover the field
            vector<const Geometry * >* p = generate_intersections(polygon, width);
            if(get_origin_flag == false){
                //Find map size and origin by finding min/max points of polygon
                double min_x = std::numeric_limits<double>::infinity();
                double min_y = std::numeric_limits<double>::infinity();
                double max_x = -std::numeric_limits<double>::infinity();
                double max_y = -std::numeric_limits<double>::infinity();


                BOOST_FOREACH(geometry_msgs::Point32 point, polygon.polygon.points){

                    min_x = std::min(min_x,(double)point.x);
                    min_y = std::min(min_y,(double)point.y);
                    max_x = std::max(max_x,(double)point.x);
                    max_y = std::max(max_y,(double)point.y);
                }

                //geos::geom::Point * origin;
                geos::geom::Point * origin_point;
                geos::geom::Coordinate coord;
                coord.x = min_x;
                coord.y = min_y;
                origin_point = factory->createPoint(coord);




                return order_points(p, origin_point);
                //;
               // return order_points(p, polygon.bounds[0:2]);
            }else{
                //;
                return order_points(p, origin);
            }

        }

    geometry_msgs::PolygonStamped rotate_polygon_to(geometry_msgs::PolygonStamped polygon_, Eigen::Matrix3f rotation_transform){
         //Takes a polygon and a rotation, returns a rotated polygon
         Eigen::ArrayXXf points;
         for(int i = 0; i < polygon_.polygon.points.size(); i++){
             points(0,i) = polygon_.polygon.points[i].x;
             points(1,i) = polygon_.polygon.points[i].y;
         }

         Eigen::ArrayXXf tf_points = rotate_to(points, rotation_transform);
         //return;
         return ndarray2polygon(tf_points);
     }
   Eigen::ArrayXXf rotate_to(Eigen::ArrayXXf points, Eigen::Matrix3f rotation_transform){
        if(points.cols()<3){
            ROS_ERROR("rotate_to: takes an numpy.ndarray");
        }
        //Rotates an ndarray of given points(x,y) to a given rotation
        Eigen::ArrayXXf new_points;
        for(int i = 0; i<points.cols(); i++){
            //float point_mat[3][1];
            Eigen::VectorXf point_mat(3);
            point_mat(0) = points(0,i);
            point_mat(1) = points(1,i);
            point_mat(2) = 0;
            Eigen::VectorXf new_point(3);
            new_point = rotation_transform * point_mat;
            new_points(0,i) = new_point(0);
            new_points(1,i) = new_point(1);
            new_points(2,i) = new_point(2);
            new_points(3,i) = -1;

        }
        if(new_points.cols()==1 || new_points.rows()==1){
            //return;
            ROS_ERROR("rotate_to maybe error");
        }else{

            return new_points;
        }
    }




   geometry_msgs::PolygonStamped ndarray2polygon(Eigen::ArrayXXf points){
        if(points.cols()<3){
            ROS_ERROR("ndarray2polygon: takes an numpy.ndarray");
        }
        geometry_msgs::PolygonStamped polygon_;
        for(int i = 0; i<points.cols(); i++){
            //std::vector<geometry_msgs::Point32
            geometry_msgs::Point32 point_;
            point_.x= points(0,i);
            point_.y= points(1,i);
            polygon_.polygon.points.push_back(point_);
        }
        return polygon_;

    }

    Eigen::Matrix3f rotation_tf_from_longest_edge( const geometry_msgs::PolygonStamped & polygon_){
        //Returns a rotation tf for the longest edge of the given polygon
        double max_distance = 0.0;
       // float max_points[2][2];
        Eigen::MatrixXd max_points(2,2);
        //int points_size_ = polygon_.points.size();
        for(int i = 0, j = polygon_.polygon.points.size()-1; i < polygon_.polygon.points.size(); j = i++){
            double distance = sqrt((polygon_.polygon.points[j].x-polygon_.polygon.points[i].x)*(polygon_.polygon.points[j].x-polygon_.polygon.points[i].x)+(polygon_.polygon.points[j].y-polygon_.polygon.points[i].y)*(polygon_.polygon.points[j].y-polygon_.polygon.points[i].y));
            if(max_distance == 0.0 || max_distance<distance){
                max_distance = distance;
                max_points(0,1) = polygon_.polygon.points[i].y;
                max_points(1,1) = polygon_.polygon.points[j].y;
                max_points(0,0) = polygon_.polygon.points[i].x;
                max_points(1,0) = polygon_.polygon.points[j].x;
            }
        }
        // Calculate the angle and return the rotation tf
        float dy = float(max_points(0,1) - max_points(1,1));
        float dx = float(max_points(0,0) - max_points(1,0));
        //float degrees_ = (180*atan(dy/dx))/3.14;
        //return RotationTransform((degrees(atan(dy/dx))));
        return RotationTransform(atan(dy/dx));
    }
    Eigen::Matrix3f RotationTransform(float angle_){
        //Represents a rotational transform
        Eigen::Matrix3f Rotation_;
        Rotation_(0,0) = cos(angle_);
        Rotation_(0,1) = -sin(angle_);
        Rotation_(0,2) = 0.0;
        Rotation_(1,0) = sin(angle_);
        Rotation_(1,1) = cos(angle_);
        Rotation_(1,2) = 0.0;
        Rotation_(2,0) = 0.0;
        Rotation_(2,1) = 0.0;
        Rotation_(2,2) = 1.0;
        return Rotation_;

    }

    vector<const Geometry * >* generate_intersections(geometry_msgs::PolygonStamped  poly, double width){
        //Subdivide a filed into coverage lines
        

        //Find map size and origin by finding min/max points of polygon
        double min_x = std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();


        BOOST_FOREACH(geometry_msgs::Point32 point, poly.polygon.points){

            min_x = std::min(min_x,(double)point.x);
            min_y = std::min(min_y,(double)point.y);
            max_x = std::max(max_x,(double)point.x);
            max_y = std::max(max_y,(double)point.y);
        }
        Eigen::Vector4f starting_breakdown;
        starting_breakdown(0) = min_x;
        starting_breakdown(1) = min_y;
        starting_breakdown(2) = max_x;
        starting_breakdown(3) = max_y;


        geos::geom::CoordinateArraySequence *points =new CoordinateArraySequence();;
        
        
        points->add(Coordinate(starting_breakdown(0),starting_breakdown(1)));
        points->add(Coordinate(starting_breakdown(0),starting_breakdown(1) + starting_breakdown(3) - starting_breakdown(1)));

        LineString *line=factory->createLineString(points);
        //create geos polygon
         vector< Geometry * > *holes1 = new vector<Geometry*>(); 
         CoordinateArraySequence coords1;

         BOOST_FOREACH(geometry_msgs::Point32 point32, poly.polygon.points){
             Coordinate obj ; 
             obj.x = point32.x; 
             obj.y = point32.y;
             obj.z = point32.z;
             coords1.add(obj);
         }
         holes1->push_back( factory->createLinearRing() );

        geos::geom::Polygon *polygon = factory->createPolygon(factory->createLinearRing(coords1), holes1); 
        Geometry *bounded_line = NULL;  
    
        try{
            bounded_line = polygon->intersection(line);
        }catch(const std::exception &e){
            ROS_ERROR(">>>there is intersection error>>>");

        }
        
        vector<const Geometry * > *lines = new vector<const Geometry*>();

        Geometry * bounded_lineM = NULL;  
        int iterations = int(ceil((starting_breakdown(2) - starting_breakdown(0)) / width)) + 1;

        for(int k = 1; k < iterations; k++){
           // Geometry* bounded_line_temp = line->parallel_offset(k * width, "right");
          geos::geom::CoordinateArraySequence *pointsM =new CoordinateArraySequence();;

          pointsM->add(Coordinate(starting_breakdown(0) + k * width,starting_breakdown(1)));
          pointsM->add(Coordinate(starting_breakdown(0) + k * width,starting_breakdown(1) + starting_breakdown(3) - starting_breakdown(1)));
          bounded_lineM = factory->createLineString(pointsM);
          
          if(polygon->intersects(bounded_lineM)){
                bounded_line = polygon->intersection(bounded_lineM);
            }else{
                ROS_ERROR("Problem looking for intersection.");
                continue;
            }
            lines->push_back(bounded_line);
            delete bounded_lineM;
            bounded_lineM = NULL;  
        }
        return lines;    


    }
    vector<const Geometry * >* sort_to(geos::geom::Point *point, vector<const Geometry * >* list){

         vector<const Geometry * > *l = new vector<const Geometry*>();

         vector<const Geometry * > *temp = new vector<const Geometry*>();

         for(int i = 0; i< list->size(); i++){
            const Geometry * element = list->at(i);
             temp->push_back(element->clone());
         }

         vector<const Geometry * >::iterator it;
         vector<const Geometry * >::iterator cancel;
         const Geometry *selected = NULL;

         double distance = 0;
         while(!temp->empty()){

             const Geometry * first = temp->front();
             distance = first->distance(point);

             for(it = temp->begin(); it!=temp->end(); it++){

                    double tempL = (*it)->distance(point);

                    if (tempL <= distance){ //rising order
                       selected = *it;
                       distance = tempL;
                       cancel = it;
                    }
             }
             l->push_back(selected);
             temp->erase(cancel);

        }
        delete temp;
        return l;


    }
    vector< geos::geom::Point * >* order_points(vector< const Geometry * >* lines, geos::geom::Point *initial_origin){
        //Return a list of points in a given coverage path order
        geos::geom::Point * origin;
        origin = initial_origin;
        vector< geos::geom::Point * >*  results;
        //std::vector<std::string> path_status;
        //lines.getLength()
        // while(!tempfrontierlist.empty()){
        //int k = 0;
        while(1){

            if(lines->empty()){
                break;
            }
            lines = sort_to(origin, lines);
            //lines->
            // vector< Geometry * >* f =lines->at()

            //Geometry * f = lines->at(lines->begin());
            const Geometry * f = lines->front();
           // virtual const Geometry* getGeometryN(std::size_t /*n*/) const { return this; };
            // Geometry * f = *(lines->begin());
            lines->erase(lines->begin());

            //f = lines.pop(0);
           // f->getGeometryType();


            if (f->getGeometryType() == "GeometryCollection"){
                continue;
            }


            if (f->getGeometryType() == "MultiLineString"){
                //Geometry* ls = f->getGeometryN()
                //const MultiLineString *ls = dynamic_cast<const MultiLineString *>(f);
                //std::size_t getNumGeometries() ;
                for(int i=0;i<f->getNumGeometries();i++){
                    const Geometry* ls = f->getGeometryN(i);
                    const LineString * new_line = dynamic_cast<const LineString *>(ls);
                    lines->push_back(new_line);
                }

                //ls->getGeometryN()
                //ls->begin()



               /* MultiLineString::iterator it;
                for(it = ls->begin(); it!=ls->end(); it++){
                    LineString new_line = *it;
                    lines->push_back(new_line);*/

              //  }
                continue;

            }


            if (f->getGeometryType() == "Point" || f->getGeometryType() == "MultiPoint"){
                continue;
            }
            CoordinateSequence* g = f->getCoordinates();

            std::vector<geometry_msgs::Point> point_list;

            point_list.clear();

            for(int i = 0; i < g->getSize();i++){

                 geometry_msgs::Point point_;
                 point_.x = g->getX(i);
                 point_.y = g->getY(i);

                 point_list.push_back(point_);


            }


            //(start, end) = get_furthest(point_list, origin);
            std::vector<geometry_msgs::Point> l = get_furthest(point_list, origin);
            results->push_back(origin);
            geos::geom::Coordinate coord2;
            coord2.x = l.front().x;
            coord2.y = l.front().y;
            geos::geom::Point * start = factory->createPoint(coord2); 
            results->push_back(start);


            int number = l.size();
            if(number > 1){
                geos::geom::Coordinate coord3;
                geometry_msgs::Point point3 = l.at(number -1 );
                coord3.x = point3.x;
                coord3.y = point3.y;
                geos::geom::Point * end = factory->createPoint(coord3);
                origin = end;     
            }


        }
        return results;
    }


std::vector<geometry_msgs::Point> get_furthest(std::vector<geometry_msgs::Point> & point_list, geos::geom::Point * origin ){


         std::vector<geometry_msgs::Point> l; // = new std::vector<geometry_msgs::Point>();

         std::vector<geometry_msgs::Point> temp;

         for(int i = 0; i< point_list.size(); i++){
             geometry_msgs::Point element = point_list.at(i); 
             
             temp.push_back(element);
         }


         vector<geometry_msgs::Point >::iterator it;
         vector<geometry_msgs::Point>::iterator cancel;
         geometry_msgs::Point selected ;

         geos::geom::Point * line_point;

         double distance = 0;
         while(!temp.empty()){

             geometry_msgs::Point first = temp.front();
             geos::geom::Coordinate coord;
             coord.x = first.x;
             coord.y = first.y;    
             line_point = factory->createPoint(coord);
             distance = origin->distance(line_point);
             delete line_point; 
             line_point = NULL; 

             for(it = temp.begin(); it!=temp.end(); it++){

                   // double tempL = (*it).distance(point);
                    geometry_msgs::Point tempP = *it; 
                    geos::geom::Coordinate coordT;
                    coordT.x = (*it).x;                        
                    coordT.y = (*it).y;
                    line_point = factory->createPoint(coordT);
                    double tempL = origin->distance(line_point);       

                    if (tempL >= distance){ 
                       selected = *it;
                       distance = tempL;
                       cancel = it;
                    }
                    delete line_point;
                    line_point = NULL; 
             } 
             l.push_back(selected);
             temp.erase(cancel);

        }
        return l;




}


    double cut_spacing;
    ros::Subscriber heapmap_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher path_marker_pub;
    ros::Publisher path_pub ;
    ros::Publisher bow_pub; 
    ros::ServiceServer serviceLineProvider;
    std::string  field_frame_id ;
    geometry_msgs::PolygonStamped field_shape; //? chenrui, do not know what the type is.
    nav_msgs::Path path ; 
    visualization_msgs::MarkerArray path_markers; 
    bool start_path_following;
    nav_msgs::Odometry robot_pose;
    bool get_robot_pose_flag;
   // hector_move_base_msgs::MoveBaseGoal current_destination;
    move_base_msgs::MoveBaseGoal current_destination;
    bool testing; 
    double current_distance;
    move_base_msgs::MoveBaseGoal previous_destination;
    bool previous_destination_flag;
    int heading ; 
    bool just_reset; 
    bool timeout;
    std::vector<std::string> path_status ;  
    std::vector<geometry_msgs::Point> line_strip_points;
    bool connected_to_move_base; 
    //actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client(const std::string & name,  bool spin_thread = true);
    client *move_base_client;

    geos::geom::GeometryFactory::unique_ptr factory; 
    ros::ServiceClient clear_costmaps;
    bool get_origin_flag;
};

}
int main(int argc, char** argv)                                                                                                                
{
  ros::init(argc, argv, "PathPlannerNode");

  path_planner::PathPlannerNode client;
  ros::spin();
  return 0;
}
	
