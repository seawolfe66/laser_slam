//=================================================================================================
// Copyright (c) 2017, Evan Zhu , @ PartnerX
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef COVERAGE_EXPLORATION
#define COVERAGE_EXPLORATION

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
//#include <costmap_2d/costmap_2d_ros.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>


#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>


#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <dynamic_reconfigure/server.h>
#include "coverage_exploration/CoverageExplorationConfig.h"

namespace coverage_exploration{

	/**
	* @brief Thread-safe implementation for coverage_exploration.
	*/
	template<typename T, typename S>
	double pointsDistance(const T &one, const S &two){
		return sqrt(pow(one.x-two.x,2.0) + pow(one.y-two.y,2.0) + pow(one.z-two.z,2.0));
	}

	template<typename T, typename S>
	bool pointsNearby(const T &one, const S &two, const double &proximity){
		return pointsDistance(one, two) <= proximity;
	}
	
     class Node{

	/**
	* @brief The node to record the position of encoutering obstacle and being out of boundary or over line when walking in the area with four direction status.
	*/
		#define CROSS_COUNT	4
		public:
			enum NodeDir{
				X_N = 0,
				Y_N,
				X_P,
				Y_P
			};
			enum NodeType{
				NT_NORMAL = 0,
				NT_SUB,
				NT_ABANDON
			};
			Node(){
				for(int i = 0;i < CROSS_COUNT;i ++)
					input[i]=output[i]= false;
				type = NT_NORMAL;
			};
			const Node& operator =(const Node& node) 
			{
				pose = node.pose;
				dir = node.dir;
				for(int i = 0;i < CROSS_COUNT;i ++)
				{	
					input[i]= node.input[i];
					output[i]= node.output[i];
				}
				return *this;
			};
			geometry_msgs::Pose pose;
			geometry_msgs::Pose joint;
			NodeDir		dir;
			bool		input[CROSS_COUNT],output[CROSS_COUNT];
			NodeType	type;
	};

	class Area{

	/**
	* @brief We specify a boundary where allowing robot to walk along wall and also record the walking status including current robot aim direction and joint position .
	*/
		#define JOINT_COUNT	4
	public:
		enum ObstacleLoc{
			OS_RIGHT = 0x01,
			OS_FRONT = 0x02,
			OS_LEFT = 0x04,
			OS_BACK = 0x08
		};
		enum BoundaryLoc{
			BL_X_N = 0x01,
			BL_X_P = 0x02,
			BL_Y_N = 0x04,
			BL_Y_P = 0x08
		};
		Area(){ cur_joint_index_ = 0;}
		bool IsOverLineWithJointIndex(geometry_msgs::Pose pose,int dest_index)
		{
			if(dest_index >= JOINT_COUNT)
				dest_index = 0;
			geometry_msgs::Pose des_pos = joints_[dest_index];
		//	ROS_INFO("Overline: aim to position(%.2f,%.2f),current position(%.2f,%.2f)",des_pos.position.x,des_pos.position.y,
		//  pose.position.x,pose.position.y);
			Node::NodeDir dir = GetDirFromJointIndex(dest_index);
			switch(dir)
			{
				case Node::X_N:
					return des_pos.position.x >= pose.position.x;
				break; 
				case Node::Y_N:
					return des_pos.position.y >= pose.position.y;
				break;
				case Node::X_P:
					return des_pos.position.x <= pose.position.x;
				break;
				case Node::Y_P:
					return des_pos.position.y <= pose.position.y;
				break;
			}
		}
		bool IsOverLine(geometry_msgs::Pose pose)
		{
			int dest_index = cur_joint_index_;
			return IsOverLineWithJointIndex(pose,dest_index);
		}
		static bool IsNearEnough(const geometry_msgs::Pose& pose1,const geometry_msgs::Pose& pose2,double tol = 0.05)
		{
			return pointsNearby(pose1.position,pose2.position,tol);
		}
		int IsOutofBox(geometry_msgs::Pose pose,float tol = 0.1)
		{
			double min_x ,min_y,max_x,max_y;
			min_x = max_x = joints_[0].position.x;
			min_y = max_y = joints_[0].position.y;
			for(int i = 1;i < JOINT_COUNT; i ++)
			{
				if(min_x > joints_[i].position.x)
					min_x = joints_[i].position.x;
				if(min_y > joints_[i].position.y)
					min_y = joints_[i].position.y;
				if(max_x < joints_[i].position.x)
					max_x = joints_[i].position.x;
				if(max_y < joints_[i].position.y)
					max_y = joints_[i].position.y;
			}
			int index = 0;
			double t_min_x = pose.position.x - min_x;
			double t_max_x = pose.position.x - max_x;
			double t_min_y = pose.position.y - min_y;
			double t_max_y = pose.position.y - max_y;
			double t_dis = 0;
			if(t_min_x < -1*tol)
			{
				t_dis = fabs(t_min_x);
				index = BL_X_N;
			}
			if(t_max_x > tol)
			{
				if(t_dis < fabs(t_max_x))
				{
					t_dis = fabs(t_max_x);
					index = BL_X_P;	
				}	
			}
			if(t_min_y < -1*tol)
			{
				if(t_dis < fabs(t_min_y))
				{
					index = BL_Y_N;	
					t_dis = fabs(t_min_y);
				}		
			}
			if( t_max_y > tol)
			{
				if(t_dis < fabs(t_min_y))
				{
					index = BL_Y_P;		
					t_dis = fabs(t_min_y);
				}	
			}
			return index;
		}
		static void GetNextDir(Node::NodeDir& dir)
		{
			switch(dir)
			{
				case Node::X_N:
					dir = Node::Y_N;
				break; 
				case Node::Y_N:
					dir = Node::X_P;
				break;
				case Node::X_P:
					dir = Node::Y_P;
				break;
				case Node::Y_P:
					dir = Node::X_N;
				break;
			}
		}
		static void GetPrevDir(Node::NodeDir& dir)
		{
			switch(dir)
			{
				case Node::X_N:
					dir = Node::Y_P;
				break; 
				case Node::Y_N:
					dir = Node::X_N;
				break;
				case Node::X_P:
					dir = Node::Y_N;
				break;
				case Node::Y_P:
					dir = Node::X_P;
				break;
			}
		}
		bool IsOver()
		{
			return cur_joint_index_ > JOINT_COUNT;
		}
		int GetLineIndexFromDir(Node::NodeDir dir)
		{
			Node::NodeDir s_dir = start_dir_;;
			for(int i = 0;i < JOINT_COUNT; i ++)
			{
				if(dir == s_dir)
					return i;
				GetNextDir(s_dir);
			}	
		}
		static Node::NodeDir GetDirFromBoundary(int boundary)
		{
			if(boundary & BL_X_N)
				return Node::X_N;
			if(boundary & BL_Y_N)
				return Node::Y_N;
			if(boundary & BL_X_P)
				return Node::X_P;
			if(boundary & BL_Y_P)
				return Node::Y_P;
		}
		static Node::NodeDir GetDirFromYaw(double yaw)
		{
			double offset = 0.2;
			if(yaw >= M_PI/2 + offset && yaw < 3*M_PI/2 - offset)
                return Node::X_N;
			else if(yaw >= M_PI + offset && yaw < 2*M_PI - offset)
                return Node::Y_N;
			else if(yaw >= 0 + offset && yaw < M_PI - offset)
                return Node::Y_P;
            else 
                return Node::X_P;
		}
		static double GetYawFromDir(Node::NodeDir dir)
		{
			switch(dir)
			{
				case Node::X_N:
					return M_PI;
					break;
				case Node::Y_N:
					return 3*M_PI/2;
					break;
				case Node::X_P:
					return 0;
					break;
				case Node::Y_P:
					return M_PI/2;
					break;
			}
		}
		void SetCurrentJointIndex(int joint_index)
		{
			ROS_ASSERT(joint_index < JOINT_COUNT);
			cur_joint_index_ = joint_index;

		}
		geometry_msgs::Pose GetCurrentJoint()
		{
			ROS_ASSERT(cur_joint_index_ < JOINT_COUNT);
			return joints_[cur_joint_index_];
		}
		int GetCurrentJointIndex()
		{
			return cur_joint_index_;
		}
		int GetCurrentLineIndex()
		{
			if(cur_joint_index_ > 0)
				return cur_joint_index_ - 1;
			return cur_joint_index_;
		}
		Node::NodeDir GetCurrentDir()
		{
			if(cur_joint_index_ == 0)
				return GetDirFromLineIndex(JOINT_COUNT - 1);
			return GetDirFromLineIndex(cur_joint_index_ - 1);
		}
		Node::NodeDir GetDirFromLineIndex( int index)
		{
			ROS_ASSERT(index < JOINT_COUNT);
			Node::NodeDir dir = start_dir_;
			for(int i = 0;i < index; i ++)
				GetNextDir(dir);

			return dir;
		}
		Node::NodeDir GetDirFromJointIndex( int index)
		{
			ROS_ASSERT(index <= JOINT_COUNT);
			if(index == 0)
				index = JOINT_COUNT;
			Node::NodeDir dir = start_dir_;
			for(int i = 0;i < index - 1; i ++)
				GetNextDir(dir);

			return dir;
		}
		bool GetCurrentJoint(geometry_msgs::Pose& pose)
		{
			if(cur_joint_index_ == JOINT_COUNT)
				pose = joints_[0];
			else
				pose = joints_[cur_joint_index_];
			
			return true;
		}
		bool GetNextJoint(geometry_msgs::Pose& pose)
		{
			cur_joint_index_ ++;
			if(IsOver())
			{
				ROS_WARN("IsOver,Current joint index %d",cur_joint_index_);
				return false;
			}
			if(cur_joint_index_ == JOINT_COUNT)
				pose = joints_[0];
			else
				pose = joints_[cur_joint_index_];
			
			return true;
		}
		geometry_msgs::Pose joints_[JOINT_COUNT];
		int cur_joint_index_;
		geometry_msgs::PolygonStamped contour_;
		Node::NodeDir  start_dir_;
		Node			start_node_;
	};

    class CoverageAction {
        public: 
	 #define NO_FINDING_ORIGINAL	
            enum RETENUM{
                C_SUCCESS = 0,
                C_NOT_INITIALIZED = -1,
                C_NOT_FOUND = -2,
                C_INVALID_PARAM = -3,
                C_UNKNOWN_ERROR = -4,
                C_FAIL = -5,
				C_NOT_IN_POSITION = -6,
				C_FAIL_TO_MOVE = -7
            };
            CoverageAction(ros::NodeHandle &n,std::string global_frame,std::string local_frame,std::string base_link) ;
            ~CoverageAction(){};
            /*! \brief Explore the next area by walking around the area to find a close contour. This function will firstly find the available node 
            *   according to the order being stored in the nodes_ vector. 
            * 
            * @return 0 success, -1 not initialized, -2 not found an available node.
            */
            int exploreNextArea(geometry_msgs::Pose& pose,	geometry_msgs::PolygonStamped& output);
			 /*! \brief Explore the next area by walking around the area to find a close contour. This function will firstly find the available node 
            *   according to computing the minimum distance to the start_point in the node vector list.. 
            * @param start_point specify the current position which is the start position to explore . 
			*                    Then it will find the closed node in the vector to this start_point.
            * @return 0 success, -1 not initialized, -2 not found an available node.
            */
            //int exploreNearestArea(const geometry_msgs::Pose& start_point);
			/*! \brief Clear the node list and set initialized_ flag to false.
            * 
            */
            int reset();
			/*! \brief Initialization operation including specifying origianl position and set the initialized_ flag to true if succeed.
            * 			It must be called firstly before using other functionalities.
            */
            void initialize(const geometry_msgs::PoseStamped& initial_pose);
			/*! \brief Find a position that is the minimum distance to obstacle around the robot and set this position as the local frame original position, 
            * 			create a timer to publish the transform from local frame to global frame.
            */
			#ifndef NO_FINDING_ORIGINAL
            bool findOriginalPosition();
			#endif 
        protected:
			
			/*! \brief Find the available direction on the specified node , and start to walk in the area then generate new nodes and being stored to node list.
            *          Meanwhile the node passed will be updated by setting the output direction status.
            * @param t_node. A node object created by walking in the area and usually stored in the node list.
            *
            */
            int exploreOnNode(int node_index);
            /*! \brief The callback to receive the message from /scan topic.
            * 
            * @param msg Message, published from laser scanning .
            *
            */
            void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

            /*! \brief Moving to specific position using move_base goal command in the global map frame(/map).
            *
            * @param pose the goal position 
            * @param timeout the timeout time for waiting for result.
            * @return 0 Finish the move_base goal command , successfully reach the goal position. others preempted or aborted .
            */
            int MoveTo(geometry_msgs::Pose pose,float timeout=30);
            /*! \brief Get current robot position in the local frame (/base_map) .
            *
            * @param(in) pose accept the current position 
            * @return 0 Obtain an effective position , others exception occurs.
            */
            int getCurrentPose(geometry_msgs::Pose& pose);
            /*! \brief Judge an obstacle is in front of robot by checking the distance from laser scan on the ahead direction .
            *
            * @return true Obstacle exists , false no obstacle ahead.
            */
            bool IsFrontierObstacle();
            /*! \brief Start to find all available areas in the room according to obstacles information and user specific vaild rect profile (2x2 meter).
            *
            * @return true done , false not finished.
            */
            //bool findArea();
            /*! \brief rotating robot to specfic degree .
            *
            * @return true done , false not finished.
            */
            bool TurnTo(double toAngle);
            /*! \brief rotating robot specific degree range from current orientation .
            *
            * @return true done , false not finished.
            */
            bool Turn(double angle);
            /*! \brief Get current orietantion of robot with yaw value.
            *
            * @return the orientation degree based on the base_map frame 
            */
            double GetCurrentYaw();
            /*! \brief A time to receive the transform from /map to /base_Map.
            *
            */
			#ifndef NO_FINDING_ORIGINAL
            void Timer();
			#endif
            /*! \brief Transform the position from base_frame_id to target_frame_id
            * @param(in,out) dest_pose the position accepted on the target_frame_id and passed as base position on the based_frame_id. 
            * @param based_frame_id the frame id to be transformed.
            * @param target_frame_id the frame id transforming to 
            * @return 0 successful , others fail 
            */
            int transformPose(geometry_msgs::Pose& dest_pose, std::string based_frame_id, std::string target_frame_id);
            /*! \brief Add new node or update the node with same position to node list which composes of all joints .
            *
            * @return true the new node is already added to the node list , will update the value , false add a new node to list
            */
            bool AddNode(std::vector<Node>& nodes,const Node& node_new,bool bOrder = false);
            /*! \brief Get the status of obstacles around robot.
            *
            * @return the value composed of four direction values , could use offset operation to get obstacle status for each direction 
            */
            int GetAroundObstacleStatus();
            /*! \brief Judge the obstacle status in specific direction.
            *
            * @return true obstacle found, false no obstacle. 
            */
            bool IsObstacle(Area::ObstacleLoc loc );
            /*! \brief Execute the robot moving operation including pulishing cmd_vel topic and doing logical judgement for each node.
            * @param area , the area where robot need to walk .
            * @param nodes_new , the node list generated when walking , the joint will be as the new node added to the node list.
            * @return true successfully walk over the area , false error occurs.
            */
            bool walkInArea(Area& area,std::vector<Node>& nodes_new);
            /*! \brief move forward by publishing cmd_vel topic.
            * @param dest_yaw orientation in 2D planar of destination .
            *
            */
            void moveForward(double dest_yaw);
            /*! \brief Get the yaw value of robot orientaion from geometry_msgs message type.
            *
            * @param pose , the position required to get the orientaion 
            * @return the yaw value. 
            */
            double GetYaw(const geometry_msgs::Pose& pose);
            /*! \brief Doing the PID algorithm for walking along the wall with constance distance from the wall.
            *
            *
            */
            void walkAlongWall();
            /*! \brief move backward by publishing cmd_vel topic..
            *
            *
            */
            void moveBackward();
            /*! \brief stop the moving by publishing cmd_vel topic..
            *
            *
            */
            void moveStop();
			/*! \brief Judge the robot is nearby the wall through sensor detection.
            *
            *
            */
			bool IsNearbyWall();
			/*! \brief Call back to configure parameters dynamically.
            *
            *
            */
			void dynParamCallback(coverage_exploration::CoverageExplorationConfig &config, uint32_t level);
			/*! \brief Add position to polygon if far beyond enough to the last position
            * @param curPose new position added 
            *
            */
			void addPoseToPolygon(const geometry_msgs::Pose& curPose);
        private:
            std::vector<Node>   nodes_; 			//!< A vector list to store all nodes when walking in the area.
            std::string         frame_local_map_;   //!< The frame id for local localization.May be the same as global frame id.
            std::string         frame_global_map_;  //!< The frame id for global localization.
            std::string         frame_base_link_;   //!< The frame id of robot body.
            bool                initialized_;		//!< The flag to indicate the initialization status of this class object.
            Node                node_original_;     //!< The node as the original position when robot powering on or being specified by user.
		
			double m_scan_front_dist;	//!< Distance, from robot head to the obstacle in front scanned by laser.
			double m_scan_right_dist;	//!< Distance, from robot right side to the obstacle on the right scanned by laser.
			double m_scan_left_dist;	//!< Distance, from robot left side to the obstacle on the left scanned by laser.
			double m_scan_back_dist;    //!< Distance, from robot back to the obstacle in behind scanned by laser.
			double m_scan_sensor_dist;  //!< Distance, from robot in the right direction with degree specified by m_sensor_angle to obstacle.
            double m_scan_min_dist;  	//!< Distance, the minimum distance around the robot to obstacle.
	        double m_scan_min_angle;	//!< Angle, the angle scanning to the closest obstacle around the robot. 
           
		   // Configurable
		    double m_sensor_angle;		//!< Angle, the orietation of senser to detect the wall.
			double m_area_bound_width;	//!< Distance, the width of boundary of area.
			double m_area_bound_height; //!< Distance, the height of boundary of area.
			double m_wall_distance;     //!< Distance, required distance to wall in direction of sensor.
			double m_obstacle_threshold;//!< Distance, the threshold of distance to obstacle .
			double m_node_offset_threshold; //!< Distance, the threshold of distance as new node with offset from current node to out of box of area .
			double m_node_replace_threshold;//!< Distance, the threshold of distance from current node to the node with same joint.
			double m_in_position_threshold; //!< Distance, the threshold of distance judging the current position is considered as the aim position.

			double m_vel_rot;			//!< Velocity, the angular speed when the robot is rotating .
			double m_tol_rot;			//!< Angle, the angle of tolerance when arriving the goal orientation.
			double m_vel_move;			//!< Velocity, the linear speed when robot is moving.
			double m_move_coef_rot;		//!< Proportion coefficient. The coefficent of rotating speed when moving for keeping straight line.

			double m_area_out_threshold; //!<Distance, the distance to the boundary where is the out of area  .
			double m_area_abort_threshold;//!<Distance, the distance of the point of obstacle beyond the point of out of boundary on the same line.
			
		// 
			tf::TransformListener 		tf_listener_; //!< TF listener to obtain transform message.
			actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_client_; //!< Move_base client to send goal.
			ros::Publisher 				pub_cmd_vel_; //!< A publisher to publish cmd_vel topic.
			ros::Subscriber 			sub_scan_;	  //!< A subscriber to receive message from laser scanning.
	        ros::NodeHandle& 		    nh_;		  //!< The node handle for this class to manage this node.
			// If using find original position funtion.
			#ifndef NO_FINDING_ORIGINAL
			bool   						found_orig_;  //!< The flag that indicating the original position is specified or not.
			geometry_msgs::Pose 		orig_pose_;   //!< The robot position as the originall position. If not using node_original_.
   			ros::Timer 					timer_;		  //!< The timer to publish the transform from global_map_ to local_map_.
			#endif
			//
			boost::shared_ptr< dynamic_reconfigure::Server<coverage_exploration::CoverageExplorationConfig> > dyn_server_; //!< Dynamic paremeter server.
			//
			geometry_msgs::PolygonStamped polygon_output_;  //!< the area polygon outputed after finding out an area
			bool 		  gather_pose_; 					//!< the flag to gather the current position into polygon
	 
	 };
}



#endif
