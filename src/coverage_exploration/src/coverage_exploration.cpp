#include "coverage_exploration/coverage_exploration.h"

namespace coverage_exploration{
        CoverageAction::CoverageAction(ros::NodeHandle &n,std::string global_frame,std::string local_frame,std::string base_link) : 
                    frame_local_map_(local_frame),
					frame_global_map_(global_frame),
					frame_base_link_(base_link),
					tf_listener_(ros::Duration(10.0)),
            		nh_(n),
            		move_client_("move_base",true)
    {
         m_area_bound_width = 2.0;
         m_area_bound_height = 2.0;
        // m_sensor_angle = -45*M_PI/180.0;
        // m_wall_distance = 0.35;
        //
        #ifndef NO_FINDING_ORIGINAL
        m_scan_min_dist = 100;
        m_scan_min_angle = 0;
        #endif
        sub_scan_ = nh_.subscribe("/scan", 10, &CoverageAction::scanCallback, this);
        pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 100);
        dyn_server_.reset(new dynamic_reconfigure::Server<coverage_exploration::CoverageExplorationConfig>(nh_));

        dyn_server_->setCallback(boost::bind(&CoverageAction::dynParamCallback, this, _1, _2));
        gather_pose_ = false;

    };
    void CoverageAction::addPoseToPolygon(const geometry_msgs::Pose& curPose)
    {

        geometry_msgs::Point32 newPoint,lastPoint;
        newPoint.x = curPose.position.x;
        newPoint.y = curPose.position.y;
        if( polygon_output_.polygon.points.empty())
            polygon_output_.polygon.points.push_back(newPoint);
        else {
            lastPoint = polygon_output_.polygon.points.back();
            float distance = pointsDistance(newPoint,lastPoint);
            if(distance > 0.1)
            {
                polygon_output_.polygon.points.push_back(newPoint);
            }
        }
            
    }
    int CoverageAction::exploreNextArea(geometry_msgs::Pose& pose,	geometry_msgs::PolygonStamped& output)
    {
        static int order_index = 0;
         ROS_WARN("exploring next area, nodes size %d",(int) nodes_.size());
        if(initialized_ == false)
        {
            ROS_WARN("CoverageAction is not initialized");
            return C_NOT_INITIALIZED;
        }

        int ret = -1;
        Area area;
        // Explore with the order of node list and one by one after finishing all direction of one node
        for(int i = 0;i < nodes_.size();i ++ )
        {
            ret = exploreOnNode(i);
            pose.position = nodes_.at(i).pose.position;
            pose.orientation = nodes_.at(i).pose.orientation;
            if(ret == C_SUCCESS || ret == C_NOT_IN_POSITION)
                break;
        }
        if(ret == C_SUCCESS)
        {
            polygon_output_.header.frame_id = frame_global_map_;
            polygon_output_.header.stamp = ros::Time::now();
            output = polygon_output_;
        }

        
        return ret;
    }

    int CoverageAction::reset()
    {
        nodes_.clear();
        initialized_ = false;
    }
    void CoverageAction::initialize(const geometry_msgs::PoseStamped& initial_pose)
    {
        if(!initialized_)
        {
            nodes_.clear();
            geometry_msgs::Pose orig_pose = initial_pose.pose;
            if(! tf_listener_.waitForTransform(frame_local_map_, initial_pose.header.frame_id, ros::Time(), ros::Duration(5)))
            {

                ROS_ERROR("CoverageAction initialization fail when waiting for transform of initial position");
                    return ;
            }
            transformPose(orig_pose,initial_pose.header.frame_id,frame_local_map_);
            node_original_.pose = orig_pose;
            node_original_.joint = orig_pose;
            // The X axis negative direciton is default direction to start the node.
            node_original_.dir = Node::X_N;//Area::GetDirFromYaw(GetYaw(initial_pose));
            node_original_.input[node_original_.dir] = true;
        //  node_original_.output[Node::Y_P] = true;
            nodes_.push_back(node_original_);
            initialized_ = true;
        }
        
    }
    int CoverageAction::exploreOnNode(int node_index)
    {
        ROS_ASSERT(node_index < nodes_.size());

        Area area;
        bool bAvaiable = false;
        Node::NodeDir dir_start;
        Node& t_node = nodes_.at(node_index); 
        if(t_node.type == Node::NT_ABANDON)
            return C_NOT_FOUND;
        for(int i = Node::X_N ; i < Node::X_N + 4;i ++)
        {
            Node::NodeDir dir_next = (Node::NodeDir)i;
            area.GetNextDir(dir_next);
            Node::NodeDir dir_behind = dir_next;
            area.GetNextDir(dir_behind);

            if(t_node.input[i] && !t_node.output[i])
            {    
                dir_start = (Node::NodeDir)i;
                bAvaiable = true;
                break;
            }


        }
        if(!bAvaiable)
            return C_NOT_FOUND;
        
        area.start_dir_ = dir_start;
        
        ROS_INFO("EXPLORING ON NODE %d typde %d in direction %d ,pose(%.2f,%.2f),joint(%.2f,%.2f),INPUT[%d,%d,%d,%d],OUTPUT[%d,%d,%d,%d]",
                    node_index,(int)t_node.type,(int)dir_start,t_node.pose.position.x,t_node.pose.position.y,t_node.joint.position.x,t_node.joint.position.y,t_node.input[0],t_node.input[1],t_node.input[2],t_node.input[3],
                    t_node.output[0],t_node.output[1],t_node.output[2],t_node.output[3]);
        
        
      //  ROS_INFO("EXPLORING ON NODE %d pose(%.2f,%.2f),joint(%.2f,%.2f)",node_index,t_node.pose.position.x,t_node.pose.position.y,t_node.joint.position.x,t_node.joint.position.y);
        geometry_msgs::Pose destPose;
        destPose = t_node.pose;
        destPose.orientation = tf::createQuaternionMsgFromYaw(area.GetYawFromDir(area.start_dir_));
        if(0 != MoveTo(destPose))
        {
            return C_FAIL_TO_MOVE;
        }
        
        geometry_msgs::Pose curPose;
        getCurrentPose(curPose);
        if(!Area::IsNearEnough(curPose,t_node.pose,0.1))
        {
            ROS_INFO("Current position(%.2f,%.2f) is not on the node position(%.2f,%.2f)",curPose.position.x,curPose.position.y,t_node.pose.position.x,t_node.pose.position.y);
            return C_NOT_IN_POSITION;
        }    

        double stride_width = m_area_bound_width;
        double stride_height = m_area_bound_height;
        
        if(area.start_dir_ == Node::X_N )
        {
            area.joints_[0] = t_node.joint;
            area.joints_[0].orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
            area.joints_[1] = area.joints_[0];
            area.joints_[1].position.x -= stride_width;
            area.joints_[1].orientation = tf::createQuaternionMsgFromYaw(M_PI);
            area.joints_[2] = area.joints_[1];
            area.joints_[2].position.y -= stride_height;
            area.joints_[2].orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);
            area.joints_[3] = area.joints_[2];
            area.joints_[3].position.x += stride_width;
            area.joints_[3].orientation = tf::createQuaternionMsgFromYaw(0);
        }
        else if(area.start_dir_ == Node::Y_N )
        {
            area.joints_[0] = t_node.joint;
            area.joints_[0].orientation = tf::createQuaternionMsgFromYaw(M_PI);
            area.joints_[1] = area.joints_[0];
            area.joints_[1].position.y -= stride_height;
            area.joints_[1].orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);
            area.joints_[2] = area.joints_[1];
            area.joints_[2].position.x += stride_width;
            area.joints_[2].orientation = tf::createQuaternionMsgFromYaw(0);
            area.joints_[3] = area.joints_[2];
            area.joints_[3].position.y += stride_height;
            area.joints_[3].orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
        }
        else if(area.start_dir_ == Node::X_P )
        {
            area.joints_[0] = t_node.joint;
            area.joints_[0].orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);
            area.joints_[1] = area.joints_[0];
            area.joints_[1].position.x += stride_width;
            area.joints_[1].orientation = tf::createQuaternionMsgFromYaw(0);
            area.joints_[2] = area.joints_[1];
            area.joints_[2].position.y += stride_height;
            area.joints_[2].orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
            area.joints_[3] = area.joints_[2];
            area.joints_[3].position.x -= stride_width;
            area.joints_[3].orientation = tf::createQuaternionMsgFromYaw(M_PI);
        }
        else if(area.start_dir_ == Node::Y_P )
        {
            area.joints_[0] = t_node.joint;
            area.joints_[0].orientation = tf::createQuaternionMsgFromYaw(0);
            area.joints_[1] = area.joints_[0];
            area.joints_[1].position.y += stride_height;
            area.joints_[1].orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
            area.joints_[2] = area.joints_[1];
            area.joints_[2].position.x -= stride_width;
            area.joints_[2].orientation = tf::createQuaternionMsgFromYaw(M_PI);
            area.joints_[3] = area.joints_[2];
            area.joints_[3].position.y -= stride_height;
            area.joints_[3].orientation = tf::createQuaternionMsgFromYaw(-M_PI/2);
        }

        
        double offset_x = 0;
        double offset_y = 0;
        if(area.IsOutofBox(t_node.pose,m_node_offset_threshold))
        {
            
            offset_x = t_node.pose.position.x - t_node.joint.position.x;
            offset_y = t_node.pose.position.y - t_node.joint.position.y;

            for(int i = 0;i < JOINT_COUNT; i ++)
            {
                area.joints_[i].position.x += offset_x;
                area.joints_[i].position.y += offset_y;
            }
            ROS_WARN(" start node is out of box with offset(%.2f,%.2f) ",offset_x,offset_y);
        }
        ROS_INFO(" new area to walk ,Joint1(%.2f,%.2f),Joint2(%.2f,%.2f),Joint3(%.2f,%.2f),Joint4(%.2f,%.2f) ",
            area.joints_[0].position.x,area.joints_[0].position.y,
            area.joints_[1].position.x,area.joints_[1].position.y,
            area.joints_[2].position.x,area.joints_[2].position.y,
            area.joints_[3].position.x,area.joints_[3].position.y);

        area.start_node_ = t_node;
        std::vector<Node> nodes_new;
        if(walkInArea(area,nodes_new))
        {

            for(int i = 0;i < nodes_new.size();i ++)
            {
                nodes_new.at(i).joint.position.x -= offset_x;
                nodes_new.at(i).joint.position.y -= offset_y;
                AddNode(nodes_,nodes_new.at(i));
            }
            nodes_.at(node_index).dir = area.start_dir_;
            nodes_.at(node_index).output[area.start_dir_] = true;
            ROS_INFO("End the node(%.2f,%.2f) index %d with INPUT[%d,%d,%d,%d],OUTPUT[%d,%d,%d,%d]",
                            t_node.joint.position.x,t_node.joint.position.y,node_index,
                            t_node.input[0],t_node.input[1],t_node.input[2],t_node.input[3],
                            t_node.output[0],t_node.output[1],t_node.output[2],t_node.output[3]);
            return C_SUCCESS;

        }
        else
        {
            ROS_WARN("explore area failed");
            return C_FAIL;
        }
    }
    void CoverageAction::dynParamCallback(coverage_exploration::CoverageExplorationConfig &config, uint32_t level)
    {
        ROS_INFO_ONCE("Dynamic parameter call back");

       // m_area_bound_width = config.area_width;
       // m_area_bound_height = config.area_height;
       
       m_sensor_angle = config.sensor_angle * (M_PI/180.0);
        m_wall_distance = config.wall_distance;
        m_obstacle_threshold = config.obstacle_threshold;
        m_node_offset_threshold = config.node_offset_threshold;
        m_node_replace_threshold = config.node_replace_threshold;
        m_in_position_threshold = config.in_position_threshold;
        m_vel_rot = config.vel_rot;
        m_tol_rot = config.tol_rot;
        m_vel_move = config.vel_move;
        m_move_coef_rot = config.move_coef_rot;
        m_area_out_threshold = config.area_out_threshold;
        m_area_abort_threshold = config.area_abort_threshold;
        
    }
    int CoverageAction::transformPose(geometry_msgs::Pose& dest_pose, std::string based_frame_id, std::string target_frame_id)
    {
        geometry_msgs::PoseStamped point;
        geometry_msgs::PoseStamped dest_point;

        point.header.stamp = ros::Time();
        point.header.frame_id = based_frame_id;
        point.pose = dest_pose;

        try
        {
            tf_listener_.transformPose(target_frame_id, point, dest_point);
            dest_pose = dest_point.pose;
        }
        catch(tf::TransformException& ex)
        {
            std::cout<<"error in transfrom from "<<target_frame_id<<" to "<<based_frame_id<<std::endl; 
            std::cout << "Exception thrown:" << ex.what()<< std::endl;
            return -1;
        }

        return 0;
    }
    int CoverageAction::getCurrentPose(geometry_msgs::Pose& pose)
    {

        double yaw, pitch, roll;
        std::string map_frame_  = frame_global_map_;
        std::string base_link_frame_ = frame_base_link_;
        try
        {
        //    ROS_INFO("WAITING FOR TRANSFORM FROM %s TO %s",map_frame_.c_str(),base_link_frame_.c_str());
            tf_listener_.waitForTransform(map_frame_, base_link_frame_, ros::Time(), ros::Duration(0.5));

            tf::StampedTransform transform;
            tf_listener_.lookupTransform(map_frame_, base_link_frame_, ros::Time(), transform);
            // std::cout.precision(3);
            // std::cout.setf(std::ios::fixed,std::ios::floatfield);

            poseTFToMsg(transform,pose);
        //    ROS_INFO("TRANSFOR_BEFORE %.2f %.2f",pose.position.x,pose.position.y);

            transformPose(pose,frame_global_map_,frame_local_map_);
        //   ROS_INFO("TRANSFOR_AFTER %.2f %.2f",pose.position.x,pose.position.y);

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
            std::cout << tf_listener_.allFramesAsString()<<std::endl;    
            return -1;
        }
    }
    int CoverageAction::MoveTo(geometry_msgs::Pose pose,float timeout)
    {   
        if(!move_client_.waitForServer(ros::Duration(10)))
        {
            ROS_ERROR("Can't connected to move base server");
            return -1;		
        }
        ROS_INFO("MoveBase moving to (%.2f,%.2f) on %s",pose.position.x,pose.position.y,frame_local_map_.c_str());

        move_base_msgs::MoveBaseGoal goal;
        transformPose(pose,frame_local_map_,frame_global_map_);
        goal.target_pose.header.frame_id = frame_global_map_;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose =  pose;
        move_client_.sendGoal(goal);
    //    goal_pub.publish(goal.target_pose);	// Must hide if using waitforresult , preempted due to confiction with rviz command after receiving this topic by rviz.
        geometry_msgs::Point32 goalPoint;
        goalPoint.x = pose.position.x;
        goalPoint.y = pose.position.y;
        while(ros::ok())
        {

            ros::spinOnce();
            actionlib::SimpleClientGoalState curState = move_client_.getState();
            // std::string strState = curState.toString();
            // std::cout<<"Current state: "<<strState<<std::endl;
            if(curState == actionlib::SimpleClientGoalState::SUCCEEDED)
        //  if(move_client_.waitForResult(ros::Duration(0.01,0)))
            {
                ROS_INFO("MOVE_BASE DONE");
                return 0;
            }
            else if(curState == actionlib::SimpleClientGoalState::ABORTED || curState == actionlib::SimpleClientGoalState::PREEMPTED)
            {
                ROS_INFO("MOVE_BASE ABORTED");
                return -2;
            }    
        }
        
        return -3;
    }
    double CoverageAction::GetCurrentYaw()
    {
        double curYaw;
        geometry_msgs::Pose curPose;
        getCurrentPose(curPose);
        curYaw = tf::getYaw(curPose.orientation);
        if(curYaw < 0)
            curYaw += 2* M_PI;
        return curYaw;
    }
    bool CoverageAction::Turn(double angle)
    {
        double curYaw = GetCurrentYaw();
        return TurnTo(angle + curYaw);
    }
    bool CoverageAction::TurnTo(double toAngle)
    {
        ROS_INFO("ROTATING TO %.2f",toAngle);
    //   return Turn(toAngle-GetCurrentYaw());
        if(toAngle > 2*M_PI)
            toAngle -= 2*M_PI;
        geometry_msgs::Twist msg;
        double curYaw = GetCurrentYaw();
        double delta = toAngle - curYaw;
        if(fabs(delta) > M_PI)
            delta *= -1;
        bool inPosition = false;
        double rot = m_vel_rot;
        while( ros::ok())
        {
            double rotYaw = GetCurrentYaw();
            double er = fabs(toAngle - rotYaw);
            if(er > M_PI)
                er = 2*M_PI - er;
            if(er < m_tol_rot)
            {
                inPosition= true;
                    break;
            }
            if(er < 0.1)
                rot = m_vel_rot/2;
            if(delta > 0)
                msg.angular.z = rot;
            else  
                msg.angular.z = -rot;
            msg.linear.x = 0;
            pub_cmd_vel_.publish(msg);
            ros::spinOnce();
        //    ROS_INFO("current angle: %.2f",rotYaw);
        }
        return inPosition;
    }
    bool CoverageAction::AddNode(std::vector<Node>& nodes,const Node& node_new,bool bOrder)
    {
        bool bfound = false;
        if( node_new.type == Node::NT_NORMAL)
        {
            for(int i= 0;i < nodes.size();i ++)
            {
                Node &node = nodes.at(i);
                if(node.type == Node::NT_ABANDON)
                {
                    ROS_INFO("AddNode -- This node index %d is abandon",i); 
                    continue;
                }  
                if(node.type == Node::NT_SUB && Area::IsNearEnough(node.pose,node_new.pose) && Area::IsNearEnough(node.joint,node_new.joint) )
                {
                    ROS_INFO("AddNode --input and output of type 1 is merged with type 0 of node index %d",i);
                    for(int j = 0;j < CROSS_COUNT ; j++)
                    {
                        node.input[j] |= node_new.input[j];
                        node.output[j] |= node_new.output[j];
                    }
                    continue;
                }   
                if(node.type == Node::NT_NORMAL && Area::IsNearEnough(node.joint,node_new.joint) )
                {
                    ROS_INFO("AddNode -- Found old node (%.2f,%.2f),index %d,direction %d",node.pose.position.x,node.pose.position.y,i,node.dir);
                    if(bOrder)
                    {
                     //   ROS_INFO("AddNode -- the old node is abandon");
                        node.type = Node::NT_ABANDON;
                        continue; //break;
                    }
                    double distance = pointsDistance(node.pose.position,node.joint.position);
                    double dis_new = pointsDistance(node_new.pose.position,node.joint.position);
                    double dis_comp = pointsDistance(node_new.pose.position,node.pose.position);
                    if(dis_comp > m_node_replace_threshold/*&& node.dir == node_new.dir*/) // New Point replace old one 
                    {
                        
                        if(dis_new <= distance )//|| (bOrder/* && (node.dir == node_new.dir)*/))
                        {
                            node.pose = node_new.pose;
                            node.dir = node_new.dir;
                            for(int j = 0;j < CROSS_COUNT ; j++)
                            {
                                node.input[j] = node_new.input[j];
                                node.output[j] = node_new.output[j];
                            }
                            ROS_INFO("-- replace the old node , new distance %.2f, old distance %.2f ",dis_new,distance);
                        }
                        else 
                            ROS_INFO("-- not a valid node ");
                    }
                    else 
                    {
                        for(int j = 0;j < CROSS_COUNT ; j++)
                        {
                            node.input[j] |= node_new.input[j];
                            node.output[j] |= node_new.output[j];
                        }
                    }
                    
                    ROS_INFO("AddNode -- Found node (%.2f,%.2f) type %d on joint(%.2f,%.2f): ,INPUT[%d,%d,%d,%d],OUTPUT[%d,%d,%d,%d]",
                                    node.pose.position.x,node.pose.position.y,(int)node.type,node.joint.position.x,node.joint.position.y,node.input[0],node.input[1],node.input[2],node.input[3],
                                    node.output[0],node.output[1],node.output[2],node.output[3]);
                    bfound = true;
                    continue; //break;
                }
            
            }
        }
        
        if(bfound == false)
        {    
            nodes.push_back(node_new);
            ROS_INFO("AddNode -- Add node(%.2f,%.2f) type %d on joint(%.2f,%.2f) to node list to size %d,INPUT[%d,%d,%d,%d],OUTPUT[%d,%d,%d,%d] ",node_new.pose.position.x,node_new.pose.position.y,
                                        (int)node_new.type,node_new.joint.position.x,node_new.joint.position.y,(int)nodes.size()
                                    ,node_new.input[0],node_new.input[1],node_new.input[2],node_new.input[3],
                                node_new.output[0],node_new.output[1],node_new.output[2],node_new.output[3]);
        }
        return bfound;
    }
    bool CoverageAction::IsObstacle(Area::ObstacleLoc loc )
    {
        switch(loc)
        {
            case Area::OS_RIGHT:
                return m_scan_right_dist <  m_obstacle_threshold;
            break;
            case Area::OS_FRONT:
                return m_scan_front_dist <  m_obstacle_threshold  ;
            break;
            case Area::OS_BACK:
            break;
            case Area::OS_LEFT:
                return m_scan_left_dist <  m_obstacle_threshold;
            break;
        }
        return false;
    }
    int CoverageAction::GetAroundObstacleStatus()
    {
        const int offset_right = 0;
        const int offset_front = 1;
        const int offset_left = 2;
        const int offset_back = 3;
        int status = 0;
        status = IsObstacle(Area::OS_RIGHT) << offset_right |
                    IsObstacle(Area::OS_LEFT) << offset_left |
                        IsObstacle(Area::OS_FRONT) << offset_front |
                            IsObstacle(Area::OS_BACK) << offset_back;
        return status;
    }
    void CoverageAction::moveForward(double dest_yaw)
    {
    //  ROS_INFO("moveing forward");
        double cur_yaw = GetCurrentYaw();

        geometry_msgs::Twist msg;
        msg.linear.x = m_vel_move;
        double delta = dest_yaw - cur_yaw;
        if(fabs(delta) > M_PI)
            delta = delta > 0 ? delta - 2*M_PI : delta + 2*M_PI;
        msg.angular.z = delta * m_move_coef_rot;
        if(msg.angular.z > 1)
            msg.angular.z = 1.0;
        if(msg.angular.z < -1)
            msg.angular.z = -1.0;
    //  ROS_INFO("MOVE FORWARD -- YAW DELTA %.2f, current yaw %.2f,dest_yaw %.2f",delta,cur_yaw,dest_yaw);
        pub_cmd_vel_.publish(msg);
    }
    void CoverageAction::moveStop()
    {
    //  ROS_INFO("moveing stop");
        geometry_msgs::Twist msg;
        msg.linear.x = 0;
        msg.angular.z = 0;
        pub_cmd_vel_.publish(msg);
    }
    void CoverageAction::moveBackward()
    {
    //   ROS_INFO("moveing backward");
        geometry_msgs::Twist msg;
        msg.linear.x = -m_vel_move;
        msg.angular.z = 0;
        pub_cmd_vel_.publish(msg);
    }
    double CoverageAction::GetYaw(const geometry_msgs::Pose& pose)
    {
        double dest_yaw = tf::getYaw(pose.orientation);
        if(dest_yaw < 0)
            dest_yaw += 2*M_PI;
        return dest_yaw;
    }
    void CoverageAction::walkAlongWall()
    {
        /*  PID walking along wall with constant distance */
        const double wall_distance = m_wall_distance;
        const double sensor_distance = m_scan_sensor_dist;
        const double front_distance = m_scan_front_dist;

        geometry_msgs::Twist twist;
        const double max_vel_linear = 0.2;
 //       const double min_vel_linear = 0.01;
        const double max_ang_linear = 0.6;

        static double last_vel_linear = 0;
        static double last_dis = 0;
        static double last_error = 0,enum_error = 0;
        static double Kp = 5,Ki = 1,Kd = 1,Kf = 0.8,Kfi = 0.1,Kfd = 0.1;

        double vel_ang,vel_linear = max_vel_linear;
        double error = wall_distance - sensor_distance ;
        double delta_error = error - last_error ;
        enum_error += delta_error;
        //
        vel_ang = Kp * error + Kd * delta_error + Ki * enum_error;
        if(front_distance < wall_distance )
        {
                vel_linear *= 0.05;
                vel_ang = 0.3;
        }
        else if(front_distance < 1.5*wall_distance)
            vel_linear *= 0.3;
        else if(front_distance < 2*wall_distance)
            vel_linear *= 0.5;
  

    //  vel_linear += enum_error * Kfi + Kfd * delta_error - Kf * error;// + Kfd * fabs(m_scan_front_dist - wall_distance);
        if(vel_linear > max_vel_linear)
            vel_linear = max_vel_linear;
        if(vel_linear < -max_vel_linear)
            vel_linear = -max_vel_linear;
            // if(fabs(vel_linear) < min_vel_linear)
            //      vel_linear = vel_linear > 0?1:-1 * min_vel_linear ;
        if(vel_ang > max_ang_linear)
            vel_ang = max_ang_linear;
        if(vel_ang < -max_ang_linear)
            vel_ang = -max_ang_linear;

        twist.angular.z = vel_ang;
        twist.linear.x = (vel_linear + last_vel_linear)/2;
        
        //
        last_error = error;
        last_vel_linear = vel_linear;

        // Publish messages 
        // ROS_INFO("\nERROR:%.2f,DeltaError:%.2f,EnumError:%.2f ---\n SensorDistance:%.2f FrontDistance:%.2f Angular:%.2f Linear:%.2f",
        //             error,delta_error,enum_error,m_scan_sensor_dist,m_scan_front_dist,vel_ang,vel_linear);
        pub_cmd_vel_.publish(twist);
    }
    
    bool CoverageAction::walkInArea(Area& area,std::vector<Node>& nodes_new)
    {
        geometry_msgs::Pose curPose,recPose,jointPose = area.joints_[0],lastObstaclePose,lastFollowPose;
        bool bNextPose = true,bRecovery = false,bOverLine = false,bObstacle = false,bUsePose = false;
        enum WalkStyle
        {
            WS_STOP,
            WS_FOLLOW,
            WS_FORWARD,
            WS_RECOVERY
        };
        WalkStyle walkStyle = WS_STOP;
        double dest_yaw = GetYaw(jointPose);

        if(!polygon_output_.polygon.points.empty())
          polygon_output_.polygon.points.clear();
        ROS_INFO("Walking in the area");

        while(ros::ok())
        {
            ros::Duration(0.01).sleep();
            ros::spinOnce();
            //
            getCurrentPose(curPose);

            //
            addPoseToPolygon(curPose);
            //

            if(bNextPose)
            {
                if(area.GetNextJoint(jointPose))
                    ROS_INFO("MoveTo the %d joint as next pose(%.2f,%.2f)",area.GetCurrentJointIndex(),jointPose.position.x,jointPose.position.y);
                else 
                {
                    ROS_WARN("Finish a node area ");
                    break;
                }
                dest_yaw = GetYaw(jointPose);
                TurnTo(dest_yaw);
                walkStyle = WS_FORWARD;
                bNextPose = false;
                bOverLine = false;
            }
            if(walkStyle == WS_FOLLOW)
                walkAlongWall();
            else if(walkStyle == WS_FORWARD)
             {
                  moveForward(dest_yaw);
                 bObstacle = false;
             }  
            
            int boundary = 0;
            if(area.GetCurrentJointIndex() == JOINT_COUNT)
            {
                switch(area.GetCurrentDir())
                {
                    case Node::X_N:
                        bOverLine = area.start_node_.pose.position.x >= curPose.position.x;
                    break; 
                    case Node::Y_N:
                        bOverLine = area.start_node_.pose.position.y >= curPose.position.y;
                    break;
                    case Node::X_P:
                        bOverLine = area.start_node_.pose.position.x <= curPose.position.x;
                    break;
                    case Node::Y_P:
                        bOverLine = area.start_node_.pose.position.y <= curPose.position.y;
                    break;
                }
            } 
            if(area.IsOverLine(curPose) || bOverLine) {
                ROS_WARN("Arrive the end line of area on the position(%.2f,%.2f)",jointPose.position.x,jointPose.position.y);
                moveStop();
                // 
                if(area.GetCurrentJointIndex() <= JOINT_COUNT)
                {
                    Node node;
                    node.pose = curPose;
                    if(bUsePose)
                     {
                         node.joint = node.pose;
                         bUsePose = false;
                     }   
                    else 
                        node.joint = jointPose;
                    node.dir = area.GetCurrentDir();
                    node.input[node.dir] = true;

                    Node::NodeDir left_dir = node.dir;
                    Area::GetNextDir(left_dir); 
                    node.output[left_dir] = true;
                    if( IsObstacle(Area::OS_RIGHT))// ||walkStyle == WS_FOLLOW
                    {
                        //  Get the directon on the right
                        Node::NodeDir right_dir = node.dir;
                        Area::GetPrevDir(right_dir); 
                        node.output[right_dir] = true;
             
                        ROS_INFO("OVERLINE - Right obstacle ");
                    }
 
                    if(IsObstacle(Area::OS_FRONT))
                    {
                        node.output[node.dir] = true;
                        ROS_INFO("OVERLINE - Front obstacle ");
                    }

                    AddNode(nodes_new,node,true);
                   // nodes_new.push_back(node);
                    ROS_INFO("Add a new node on the position(%.2f,%.2f) with direction %d,obstacle status %d",curPose.position.x,curPose.position.y,node.dir,GetAroundObstacleStatus());

                }
                
                bNextPose = true;
                walkStyle = WS_STOP;
            //
                
            }
            else if((boundary = area.IsOutofBox(curPose,m_area_out_threshold)) > 0  && walkStyle != WS_FORWARD){
                 double cur_yaw = GetYaw(curPose);
                 Node::NodeDir dir_cur = area.GetDirFromYaw(cur_yaw);
                bool bAvailable = false;
                // Ensure the current orietation is same as the boundary 's normal vector . If we can ensure robot is always move forward then this line can be 
                double offset = 0.2;
                if( (cur_yaw >= M_PI/2 + offset && cur_yaw < 3*M_PI/2 - offset) && (boundary & Area::BL_X_N ) || 
                            (cur_yaw >= 3*M_PI/2 + offset || cur_yaw < M_PI/2- offset) && (boundary & Area::BL_X_P ) || 
                              (cur_yaw >= M_PI + offset&& cur_yaw < 2*M_PI- offset) && (boundary & Area::BL_Y_N)  ||
                                (cur_yaw >= 0+ offset && cur_yaw < M_PI- offset) && (boundary & Area::BL_Y_P) )
                {
                    bAvailable = true;
                }
                if( !bRecovery && bAvailable)
                {
                    moveStop();
                    ROS_WARN("Arrive the boundary of area on the position(%.2f,%.2f) on boundary %d with direction %d from yaw %.2f",curPose.position.x,curPose.position.y,boundary,dir_cur,cur_yaw);

                    Node::NodeDir next_dir ,dir= area.GetDirFromBoundary(boundary);
                    next_dir = dir;
                    area.GetNextDir(next_dir);
                    int line_index = area.GetLineIndexFromDir(next_dir);
                    ROS_INFO("Robot is on the line index %d,joint index %d",line_index,area.GetCurrentJointIndex()); 
                
                    bool bSameLine = false;
                    // This line to avoid the infinite loop when line index computed is smaller than current line index. 
                    if(line_index < area.GetCurrentLineIndex())
                    {
                        ROS_WARN("Line index computed is smaller than line index planned , finish this area");
                        break;
                    }
                    else if(line_index == area.GetCurrentLineIndex() && bObstacle) // The encounter  line is same as the out of boundary line.
                    {
                        bSameLine = true;
                        double s_o = pointsDistance(area.joints_[area.GetCurrentLineIndex()].position,lastObstaclePose.position);
                        double b_s = pointsDistance(area.joints_[area.GetCurrentLineIndex()].position,curPose.position);
                        
                        // Judge if it is closed through the distance to start point of this line
                        if(b_s - s_o < -m_area_abort_threshold) 
                        {
                            ROS_WARN("Same line index so it is closed , need break. b_s %.2f , s_o %.2f",b_s,s_o);
                            break;
                        }
                        
                    }
                   // Judge the new position is over line .If yes, continue and leave it to next loop or add a new node with sub type.
                    if(area.IsOverLineWithJointIndex(curPose,line_index) )
                    {
                        ROS_WARN("Boundary is over line");
                
                        area.SetCurrentJointIndex(line_index);
                        area.GetCurrentJoint(jointPose);

                        bool bEx = false;
                        for(int i = 0;i < nodes_new.size();i ++)
                        {
                            if(Area::IsNearEnough(jointPose,nodes_new.at(i).joint))
                            {
                                bEx = true;
                                break;
                            }
                        }
                        // bOverLine = true;
                        if(bSameLine)
                        {
                            ROS_INFO("Boundary in same line , Add a new node with sub type");
                            
                            Node node;
                            node.pose = curPose;
                            node.joint = jointPose;
                            node.dir = area.GetCurrentDir();
                            node.input[node.dir] = true;
                            node.type = Node::NT_SUB;
                            if(IsObstacle(Area::OS_FRONT))
                            {
                                node.output[node.dir] = true;
                                ROS_INFO("same line - Front obstacle ");
                            }
                            AddNode(nodes_new,node);
                            
                        } 
                        else if(!bEx)
                        {
                            ROS_INFO(" Boundary -- add new node with joint(%.2f,%.2f)",jointPose.position.x,jointPose.position.y);
                            continue;
                        } 
                        
                    }
                    // Set a new joint position with current line index.
                    area.SetCurrentJointIndex(line_index);

                    // Get Next joint position and set recovery until in the box .
                    if(!area.GetNextJoint(jointPose))
                        ROS_ERROR(" Set wrong joint index ");
                     ROS_INFO("After recurrence ,start direction %d,Get line index %d ,current direction %d , joint index:%d,jointPose((%.2f,%.2f)",area.start_dir_,line_index,dir,area.GetCurrentJointIndex(),jointPose.position.x,jointPose.position.y);
                    dest_yaw = GetYaw(jointPose);
                    TurnTo(dest_yaw);
                    walkStyle = WS_FORWARD;
                    
                    bRecovery = true;
                }
            }
            else if( ( IsFrontierObstacle()) && walkStyle != WS_FOLLOW){
                bObstacle = true;
                lastObstaclePose = curPose;
                ROS_WARN("Encounter obstacle ahead ,obstacle distance %.2f",m_scan_front_dist);
                bool bOverCurPose = false;

                bool bEx = false;
                for(int i = 0;i < nodes_new.size();i ++)
                {
                    if(Area::IsNearEnough(jointPose,nodes_new.at(i).joint))
                    {
                        bEx = true;
                        break;
                    }
                }
                
                if( pointsDistance(curPose.position,jointPose.position) < m_in_position_threshold || ((area.GetCurrentJointIndex() == JOINT_COUNT) && pointsDistance(curPose.position,area.start_node_.pose.position) < m_in_position_threshold) )
                {
                     ROS_WARN("Over current pose , dir %d, start node pose(%.2f,%.2f),current pose(%.2f,%.2f)",area.GetCurrentDir(),area.start_node_.pose.position.x,area.start_node_.pose.position.y,curPose.position.x,curPose.position.y);

                     bOverCurPose = true;
                }
                else  if(!bEx)
                {
                    ROS_WARN(" Obstacle -- add new node with joint(%.2f,%.2f)",jointPose.position.x,jointPose.position.y);
                    
                    Node node;
                    node.pose = curPose;
                    node.joint = jointPose;
                    node.dir = area.GetCurrentDir();
                    node.input[node.dir] = true;
                    node.output[node.dir] = true;
                    Node::NodeDir left_dir = node.dir;
                    Area::GetNextDir(left_dir); 
                    node.output[left_dir] = true;
                    AddNode(nodes_new,node);
                    //  

                }
                if(bOverCurPose)
                {
                     bOverLine = true;
                }
                else
                {
                    lastFollowPose = curPose;                
                    ROS_INFO("Turn on wall following");
                    walkStyle = WS_FOLLOW;
                }
               
            }
            else if(!bRecovery && (IsNearbyWall()/* || IsObstacle(Area::OS_RIGHT)*/) && walkStyle != WS_FOLLOW){
                ROS_INFO("Nearby wall");
                lastFollowPose = curPose;
                walkStyle = WS_FOLLOW;
            }
            if(bRecovery &&  area.IsOutofBox(curPose,0) == 0)
            {
                ROS_INFO("Recovery from outofbox on the position(%.2f,%.2f)",curPose.position.x,curPose.position.y);
                bRecovery = false;
            }
        }
        if(nodes_new.size() > 4)
        {
             ROS_WARN("There is extra node added ");
             for(int i = 0;i < nodes_new.size();i ++)
             {
                 Node t_node = nodes_new.at(i);
                 ROS_INFO("node %d - dir %d, pose(%.2f,%.2f), joint(%.2f,%.2f), INPUT[%d,%d,%d,%d],OUTPUT[%d,%d,%d,%d]",
                                i,t_node.dir,t_node.pose.position.x,t_node.pose.position.y,t_node.joint.position.x,t_node.joint.position.y,t_node.input[0],t_node.input[1],t_node.input[2],t_node.input[3],
                                t_node.output[0],t_node.output[1],t_node.output[2],t_node.output[3]);
             }
        }
        return true;
    }

    void CoverageAction::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
     //   ROS_INFO("Scan callback ! ");
        int size = msg->ranges.size();
        int right_index = -1,left_index = -1,front_index = -1,sensor_index=-1,min_index = 0;
        double dis_front=0,dis_sensor = 0;
        for(int i=0; i<size; i++)
        {
            double angle = msg->angle_min +  (i)*msg->angle_increment;

            if (msg->ranges[i] < msg->ranges[min_index] && msg->ranges[i] > 0.01){

                    min_index = i;
                }
            if(angle >= -M_PI/2 && right_index == -1)
                right_index = i;
            else if(angle >= M_PI/2 && left_index == -1)
                left_index = i;
            else if(angle  >= -10*msg->angle_increment && angle <= 10*msg->angle_increment)
            {
                if(front_index == -1 || dis_front > msg->ranges[i])
                {
                dis_front = msg->ranges[i];
                front_index = i;
                }
            }
            if(angle  >= m_sensor_angle-2*msg->angle_increment && angle <= (m_sensor_angle+2*msg->angle_increment))
            {
                if(sensor_index == -1 || dis_sensor > msg->ranges[i])
                {
                    dis_sensor = msg->ranges[i];
                    sensor_index = i;
                }
            }
        //    ROS_INFO("ANGLE %.2f : %.2f",angle,msg->ranges[i]);
        }
        
        m_scan_sensor_dist = msg->ranges[sensor_index];
        m_scan_front_dist = msg->ranges[front_index];
        m_scan_right_dist = msg->ranges[right_index];
        m_scan_left_dist = msg->ranges[left_index];

        #ifndef NO_FINDING_ORIGINAL
        if(found_orig_)
        {
            if(m_scan_min_dist > msg->ranges[min_index])
                {
                    m_scan_min_dist =  msg->ranges[min_index];
                    geometry_msgs::Pose curPose;
                    getCurrentPose(curPose);
                    double curYaw = tf::getYaw(curPose.orientation);
                    m_scan_min_angle = (min_index-size/2)*msg->angle_increment + curYaw;
                    if(m_scan_min_angle > M_PI)
                        m_scan_min_angle -= 2*M_PI;
                    else if(m_scan_min_angle < -M_PI)
                        m_scan_min_angle += 2*M_PI;
                // ROS_INFO("min: angle=%f, distance=%f, curYaw=%f", m_scan_min_angle, m_scan_min_dist, curYaw);
                }
        //  ROS_INFO("SCAN DISTANCE: FRONT-%.2f RIGHT-%.2f LEFT-%.2f",m_scan_front_dist,m_scan_right_dist,m_scan_left_dist);
        }
        #endif

    }
    #ifndef NO_FINDING_ORIGINAL
    bool CoverageAction::findOriginalPosition()
    {
        m_scan_min_dist = 100;
        found_orig_ = true;
        
        // Firstly rotating 180 degree 
        geometry_msgs::Pose curPose;
        getCurrentPose(curPose);
        double curYaw = tf::getYaw(curPose.orientation);
        // tf::Pose pose;
        // tf::poseMsgToTF(curPose, pose);
        // double yaw_angle = tf::getYaw(pose.getRotation());
        double rotYaw = curYaw + M_PI;

        geometry_msgs::Pose rotPose = curPose;
        tf::Quaternion quat = tf::createQuaternionFromYaw(rotYaw);

        rotPose.orientation.x = quat.x();
        rotPose.orientation.y = quat.y();
        rotPose.orientation.z = quat.z();
        rotPose.orientation.w = quat.w();
        ROS_INFO("Current position (%.2f,%.2f,%.2f) , Current yaw %.2f, rot yaw %.2f",curPose.position.x,curPose.position.y,curPose.position.z,curYaw,rotYaw);
    
        // if(0 != MoveTo(rotPose))
        // {
        //  //   return false;
        // }
        rotYaw = curYaw;
        geometry_msgs::Twist msg;
        double delta = 0;
        ROS_INFO("Start rotating PI");
        Turn(M_PI);

        ROS_INFO("End rotating ");
        const double offset = m_obstacle_threshold + 0.05;
        if(m_scan_min_dist < offset)
        {
            ROS_INFO("Distance to original position is too short relative to %.2f",offset);
            return true;
        } 
        else if(m_scan_min_dist == 100)
        {
            ROS_WARN("The shortest distance not founded");
            return false;
        }
        //
        

        double orig_yaw = m_scan_min_angle + M_PI/2;
        geometry_msgs::Pose origPose = curPose;
        origPose.position.x = curPose.position.x + (m_scan_min_dist - offset)* cos(m_scan_min_angle);
        origPose.position.y = curPose.position.y + (m_scan_min_dist - offset)* sin(m_scan_min_angle);
        quat = tf::createQuaternionFromYaw(orig_yaw);
        origPose.orientation.x = quat.x();
        origPose.orientation.y = quat.y();
        origPose.orientation.z = quat.z();
        origPose.orientation.w = quat.w();
        ROS_INFO("Original position (%.2f,%.2f,%.2f) min_angle:%.2f min_distance:%.2f",origPose.position.x,origPose.position.y,orig_yaw,m_scan_min_angle,m_scan_min_dist);
        orig_pose_ = origPose;
        found_orig_ = false;
        if(0 != MoveTo(origPose))
        {
            ROS_INFO("Moveto  time out");
            return false;
        }
        
        frame_local_map_ = "/base_map";
        timer_ = nh_.createTimer(ros::Duration(0.1),boost::bind(&CoverageAction::Timer,this));

        return true;
    }
    #endif 
    bool CoverageAction::IsNearbyWall()
    {
        return m_scan_sensor_dist <=  m_wall_distance;
    }
    bool CoverageAction::IsFrontierObstacle()
    {
        // To simulate the touch sensor so substracting 0.1 .
         return m_scan_front_dist <  m_obstacle_threshold - 0.1 ;
    }
    #ifndef NO_FINDING_ORIGINAL
    void CoverageAction::Timer()
    {
    //  ROS_INFO("TIMER");
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(orig_pose_.position.x,orig_pose_.position.y, 0) );
        tf::Quaternion q;
        q.setRPY(0, 0, m_scan_min_angle- M_PI/2);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),frame_global_map_, frame_local_map_));
    }   
    #endif
}
