#include "node_wallFollowing.h"
//#include <node_wallFollowing.h>
#include <math.h>
//Constructor and destructor
NodeWallFollowing::NodeWallFollowing():
    error_left(0),
    direction(1),
    error_front(0),
    sum(0),
    distMin(0),//minimum distance masured by sensor
    angleMin(0), //angle, at which was measured the shortest distance
    init_yaw(0),
    flag_turn(0),
    point_count(0),
    count(0){
    #if 0
    server_ = new dynamic_reconfigure::Server<dem_wall_following::psdtuneConfig>;
    f_ = boost::bind(&NodeWallFollowing::reconfigureCB, this, _1, _2);
    server_->setCallback(f_);
    #endif
    switch_state_=false;
    robot_state=ID_init;
    stop=0;
    first=true;
    robot_twist.angular=0;
    robot_twist.linear=0;
    bumper.left=false;
    bumper.center=false;
    bumper.right=false;
    result=WAIT;
    ros::NodeHandle nh_("~/");
    nh_.param("rotate_kp",rotateKp,8.0);
    nh_.param("dist_fwall", frontWall,0.3);
    nh_.param("dist_lwall",leftWall,0.3);
    nh_.param("Kp_fw", Kp,4.0);
    nh_.param("Ki_fw", Ki,0.0);
    nh_.param("Kp_lw",Kp_lw,10.0);
    nh_.param("Kp_pw",Kp_pw,-5.0);
    nh_.param("Kp_aw",Kp_aw,30.0);
    nh_.param("angle_accuracy",angleAccuracy,0.2);
    nh_.param("max_speed",maxSpeed,0.25);
    nh_.param("midd_speed",middSpeed,0.12);
    nh_.param("low_speed",lowSpeed,0.08);
    nh_.param("min_radius",minRadius,0.25);
    nh_.param("max_angular",maxAngular,1.0);

    //ROS_INFO("construction success");
    pubMessage = nh_.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC, PUBLISHER_BUFFER_SIZE);
    scan_sub = nh_.subscribe(SUBSCRIBER_TOPIC, 10*SUBSCRIBER_BUFFER_SIZE, &NodeWallFollowing::messageCallback, this);
    bumper_sub = nh_.subscribe(SUBSCRIBER_BUMPER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &NodeWallFollowing::bumperCallback, this);
    bow_sub_ = nh_.subscribe(SUBSCRIBER_BOW_TOPIC, SUBSCRIBER_BUFFER_SIZE, &NodeWallFollowing::bowCmdCallback, this);
    odom_sub_=nh_.subscribe(SUBSCIBER_ODOM_TOPIC,2*SUBSCRIBER_BUFFER_SIZE,&NodeWallFollowing::odomCallback, this);

}
#if 0
void NodeWallFollowing::reconfigureCB(dem_wall_following::psdtuneConfig &config, uint32_t level){
   frontWall = config.wDst;
   Kp=config.P;
   Ki=config.S;
   maxSpeed=config.maxSp;
   direction=config.wDir;
   ROS_INFO("Reconfigure Request: %f %f %f %f %f %d %f",
             config.wDst, config.P, config.S,
             config.D, config.maxSp, config.wDir,
             config.an);
}
#endif
NodeWallFollowing::~NodeWallFollowing(){
}
//Subscriber
void NodeWallFollowing::messageCallback(const sensor_msgs::LaserScan & scan){

    front_distance=angleRanging(scan,0);
    left_distance=angleRanging(scan,60);
    vertical_distance=angleRanging(scan,90);
    int size=scan.ranges.size();
    int minIndex=0;
    for(int i=0;i<=size;i++){
        if(scan.ranges[i]<scan.ranges[minIndex]&&scan.ranges[i]>0.01)
            minIndex=i;
    }
    angleMin=minIndex*scan.angle_increment;
    distMin=scan.ranges[minIndex];
    ROS_INFO("angleMin:%f distMin:%f",angleMin,distMin);
    if(front_distance>=ultraRange)
        front_distance=ultraRange;
    if(left_distance>=wallRange)
        left_distance=wallRange;
    ROS_INFO("front_distance:%f,left_distance:%f",front_distance,left_distance);
    error_left=(left_distance-leftWall);
    error_front=(front_distance-ultraRange);
    sum=sum+error_left;

    robotStateDetect();
    robotStateExecute();
}
void NodeWallFollowing::bumperCallback(const kobuki_msgs::BumperEventConstPtr& bmsg ){

    if(bmsg->state == kobuki_msgs::BumperEvent::PRESSED)
    {
        switch (bmsg->bumper)
        {
            case kobuki_msgs::BumperEvent::LEFT:
                if(bumper.left == false)
                {
                    //ROS_WARN("bumperCB: [left bumper]");
                    bumper.left = true;
                }
                break;

            case kobuki_msgs::BumperEvent::CENTER:
                if(bumper.center == false)
                {
                    //ROS_WARN("bumperCB: [center bumper]");
                    bumper.center = true;
                }
                break;

            case kobuki_msgs::BumperEvent::RIGHT:
                if(bumper.right == false)
                {
                        //ROS_WARN("bumperCB: [right bumper]");
                    bumper.right = true;
                }
                break;
        }
    }
}
/**
* @brief subscribe odom
* @param[in] odomtry
* @param[out] none
* @note
*/
void NodeWallFollowing::odomCallback(const nav_msgs::Odometry& odom_try){
    tf::StampedTransform tfPose;
    try{
        mTransformListener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(10.0) );
        mTransformListener.lookupTransform("odom","base_link",ros::Time(0),tfPose);
    }
    catch(tf::TransformException ex){
        ROS_WARN("Failed to compute odometry pose, skipping scan (%s)", ex.what());
        return;
    }
    robot_twist.linear=odom_try.twist.twist.linear.x;
    robot_twist.angular=odom_try.twist.twist.angular.z;
    if(first){
        init_pose.x=tfPose.getOrigin().x();
        init_pose.y=tfPose.getOrigin().y();
        ROS_INFO("init_pose.x is %f ,init_pose.y is %f",init_pose.x,init_pose.y);
        first=false;
    }
   //ROS_INFO("robot pose x:%f y:%f",tfPose.getOrigin().x(),tfPose.getOrigin().y());
    if(point_count>1200){
        if((fabs(tfPose.getOrigin().x()-init_pose.x)<minRadius)&&(fabs(tfPose.getOrigin().y()-init_pose.y)<minRadius))
            result=SUCCESS;
        else
            result=WAIT;
    }
    else
        result=WAIT;
    point_count++;
    current_yaw=tf::getYaw(tfPose.getRotation());
    current_yaw = (current_yaw > PI) ? (current_yaw - 2 * PI) : ((current_yaw < -PI) ? (current_yaw + 2 * PI) : current_yaw);
    //ROS_INFO("current yaw is %f",current_yaw);
    monitorCircle(tfPose.getOrigin().x(),tfPose.getOrigin().y());
}
void NodeWallFollowing::bowCmdCallback(const std_msgs::String& move_cmd){
    if (move_cmd.data == "bow")
    {
        switch_state_=true;
    }
    else
        switch_state_=false;

    if(switch_state_)
    {
        pubMessage.shutdown();
        ROS_INFO("The velocity for walk along the wall is shut down!");
    }

}
/**
* @brief  robot state detection
* @param[in] none
* @param[out] none
* @note
*/
void NodeWallFollowing::robotStateDetect(){

    if(distMin>frontWall){
        ROS_INFO("looking wall...");
        robot_state=ID_lookingWall;
    }
    else{
        if(flag_turn){
            robot_state=ID_frontObs;
        }
        else{
            if(bumper.left||bumper.center){
                ROS_INFO("bumper trigger ..........");
               if(count>=5){
                    bumper.left=false;
                    bumper.center=false;
                    count=0;
               }
               else{
                    count++;
                    robot_state=ID_bumper;
               }
            }
            else{
                if(front_distance<0.7*frontWall){
                    ROS_INFO("front and left obstacle......");
                    flag_turn=1;
                    init_yaw=current_yaw-PI/2;
                    init_yaw = (init_yaw > PI) ? (init_yaw - 2 * PI) : ((init_yaw < -PI) ? (init_yaw + 2 * PI) : init_yaw);
                }
                else if(fabs(left_distance-leftWall)>0.1){
                    ROS_INFO("parallwall.......");
                    robot_state=ID_parallWall;
                }
                else{
                    robot_state=ID_followingWall;
                    ROS_INFO("following wall ......");
                }
                #if 0
                    if(fabs(left_distance-leftWall)>0.1){
                        ROS_INFO("parallwall.......");
                        robot_state=ID_parallWall;
                    }
                    else{
                        if((front_distance<0.8*frontWall)&&(left_distance<1.2*leftWall)){
                            ROS_INFO("front and left obstacle......");
                            flag_turn=1;
                            init_yaw=current_yaw-PI/2;
                            init_yaw = (init_yaw > PI) ? (init_yaw - 2 * PI) : ((init_yaw < -PI) ? (init_yaw + 2 * PI) : init_yaw);
                        }
                        else{
                            robot_state=ID_followingWall;
                            ROS_INFO("following wall ......");

                        }
                    }
                #endif
            }
        }
    }
}
/**
* @brief  robot state execute
* @param[in] none
* @param[out] none
* @note
*/
void NodeWallFollowing::robotStateExecute(){

    switch(robot_state){
        case ID_lookingWall:
                lookingWall(angleMin,distMin);
              break;
        case ID_parallWall:
                parallaWall(left_distance,front_distance);
              break;
        case ID_followingWall:
                followingWall();
              break;
        case ID_frontObs:
                if(adjustAngle())
                    flag_turn=0;
              break;
        case ID_bumper:
                bumperWall(vertical_distance);
             break;
    }

}
void NodeWallFollowing::monitorCircle(double x,double y){
    std::vector<Pose>::iterator it;
    Pose odom_position;
    odom_position.x=x;
    odom_position.y=y;
     //ROS_INFO("odom_posetion.x is %f,odom_posetion.y is %f,i is %d",odom_position.x,odom_position.y,i);
    if(robot_pose.size()>200){
        it=robot_pose.begin()+robot_pose.size()-50;
        if((fabs(odom_position.x-(*it).x)<0.0001)&&(fabs(odom_position.y-(*it).y)<0.0001))
            stop=0;
            //ROS_INFO("robot do not move ");
        else{
            //ROS_INFO("monitor begin .......");
            for(it=robot_pose.begin();it!=robot_pose.begin()+robot_pose.size()-200;it++){
                if((fabs(odom_position.x-(*it).x)<0.02)&&(fabs(odom_position.y-(*it).y)<0.02)){
                    stop=1;
                   // ROS_INFO("  you just draw a circle : odom_position.x is %f,odom_position.y is %f",odom_position.x,odom_position.y);
                }
            }
        }
    }
    if(robot_pose.size()>1500)
        robot_pose.erase(robot_pose.begin());
    robot_pose.push_back(odom_position);
}
/**
* @brief adjust rotate angle
* @param[in] none
* @param[out] none
* @note
*/
bool NodeWallFollowing::adjustAngle(){
    //ROS_INFO("adjust angle..........flag_turn:%d",flag_turn);
    geometry_msgs::Twist msg;
    float angle_diff=-current_yaw+init_yaw;
    angle_diff = (angle_diff > PI) ? (angle_diff - 2 * PI) : ((angle_diff < -PI) ? (angle_diff + 2 * PI) : angle_diff);
    if(fabs(angle_diff)>angleAccuracy){
        //ROS_INFO("init_yaw is %f",init_yaw);
        //ROS_INFO("curren_orien is %f",current_yaw);
        //ROS_INFO("angle_diff is %f",angle_diff);
        if(robot_twist.angular<rotateKp*angle_diff)
            msg.angular.z=robot_twist.angular+0.1;
        else if(robot_twist.angular>rotateKp*angle_diff)
            msg.angular.z=robot_twist.angular-0.1;
        msg.linear.x=0.0;
        twist_mutex_.lock();
        pubMessage.publish(msg);
        twist_mutex_.unlock();
        return false;
    }
    else{
        //ROS_INFO("Rotate finish.........");
        return true;
    }
}
double NodeWallFollowing::angleRanging(const sensor_msgs::LaserScan& scan, double angle){
    double ret_distance;
    double distance;

    int count = 0;

    double begin = D2R((angle - 1.5));
    double end   =  D2R((angle + 1.5));
    double angle_range = scan.angle_max - scan.angle_min;
    size_t index_begin = (size_t) ((begin - scan.angle_min) / angle_range * scan.ranges.size());
    size_t index_end   = (size_t) ((end   - scan.angle_min) / angle_range * scan.ranges.size());

    double min_distance = scan.range_max;
    for(size_t i=index_begin; i < index_end; i++)
    {
        if(scan.ranges[i] < scan.range_min || scan.ranges[i] > scan.range_max)
            continue;
        if(min_distance > scan.ranges[i])
            min_distance = scan.ranges[i];

        distance += scan.ranges[i];
        count ++;
    }

    if(count > 0)
    {
        distance = distance /count;
        ret_distance = 0.5 * distance + 0.5* min_distance;
    }
    else
    {
        ret_distance = min_distance;
    }


    return min_distance;

}
void NodeWallFollowing::lookingWall(double angle,double distance){
    double w;
    double vel;
    geometry_msgs::Twist msg;
    w=Kp_lw*(angle-PI);
    if(fabs(w)>maxAngular){
        if(w>0)
            w=maxAngular;
        else
            w=-maxAngular;
    }
    msg.angular.z=w;
 #if 0
    if(robot_twist.angular<w)
        msg.angular.z=robot_twist.angular+0.1;
    else if(robot_twist.angular>w)
        msg.angular.z=robot_twist.angular-0.1;
 #endif

    if(fabs(distance-front_distance)<0.1)
        msg.linear.x=0.1;
#if 0
    {
        if(distance>2*frontWall)
            msg.linear.x=accSpeed(maxSpeed);
        else if(distance>frontWall)
            msg.linear.x=accSpeed(middSpeed);
        else
            msg.linear.x=accSpeed(lowSpeed);
    }
    else
            msg.linear.x=accSpeed(lowSpeed);
#endif

    ROS_INFO("Lookingwall: msg.angular.z is %f,msg.linear.x is%f",msg.angular.z,msg.linear.x);
    twist_mutex_.lock();
    pubMessage.publish(msg);
    twist_mutex_.unlock();
}
void NodeWallFollowing::parallaWall(double left,double front){
    double w;
    geometry_msgs::Twist msg;
    if(left>front){
        w=Kp_pw*(left-leftWall);
    }
    else if(left<front||((left>0.8)&&(front>0.8))){
        w=Kp_pw*(-left+leftWall);
    }
    if(fabs(w)>maxAngular){
        if(w>0)
            w=maxAngular;
        else
            w=-maxAngular;
    }
        msg.angular.z=w;
    #if 0
    if(robot_twist.angular<w)
        msg.angular.z=robot_twist.angular+0.15;
    else if(robot_twist.angular>w)
        msg.angular.z=robot_twist.angular-0.15;
    #endif

    if(angleMin>4*PI/3&&angleMin<2*PI&&(front_distance>4*distMin)){
        ROS_INFO("ooooh......");
        msg.linear.x=accSpeed(0.6*lowSpeed);
    }
    if((front_distance-frontWall)>0.1){
        msg.linear.x=0.05;
    }
    twist_mutex_.lock();
    pubMessage.publish(msg);
    twist_mutex_.unlock();
    ROS_INFO("parallaWall: msg.angular.z is %f,msg.linear.x is%f",msg.angular.z,msg.linear.x);
}
void NodeWallFollowing::bumperWall(double distance){
    double w;
    geometry_msgs::Twist msg;
    w=Kp_aw*(distMin-distance);
    if(fabs(w)>maxAngular){
        if(w>0)
            w=maxAngular;
        else
            w=-maxAngular;
    }

    if(robot_twist.angular<w)
        msg.angular.z=robot_twist.angular+0.25;
    else if(robot_twist.angular>w)
        msg.angular.z=robot_twist.angular-0.25;

    if(angleMin>4*PI/3&&angleMin<2*PI&&(front_distance>4*distMin)){
       // ROS_INFO("ooooh......");
        msg.linear.x=accSpeed(0.8*lowSpeed);
    }
    twist_mutex_.lock();
    pubMessage.publish(msg);
    twist_mutex_.unlock();

   ROS_INFO("avoidWall: msg.angular.z is %f,msg.linear.x is%f",msg.angular.z,msg.linear.x);
}
void NodeWallFollowing::followingWall(){
    //preparing messagekobuki_msgs/BumperEvent
    double w;
    geometry_msgs::Twist msg;

        w=Kp*error_left+Ki*sum;
    if(fabs(w)>maxAngular){
        if(w>0)
            w=maxAngular;
        else
            w=-maxAngular;
    }
    if(robot_twist.angular<w)
        msg.angular.z=robot_twist.angular+0.1;
    else if(robot_twist.angular>w)
        msg.angular.z=robot_twist.angular-0.1;
    ROS_INFO("msg.angular.z is %f,robot_twist.angular is%f",msg.angular.z,robot_twist.angular);
    if(left_distance<1.5*leftWall){
        if(front_distance>1.5*frontWall){
            msg.linear.x=accSpeed(maxSpeed);
        }
        else if(front_distance>1.0*frontWall)
            msg.linear.x= accSpeed(middSpeed);
        else if(front_distance>0.6*frontWall)
            msg.linear.x= accSpeed(lowSpeed);
        else{
            msg.linear.x= 0;
        }

    }
    else{
        if(front_distance>1.0*frontWall)
            msg.linear.x= accSpeed(middSpeed);
        else if(front_distance>0.8*frontWall)
            msg.linear.x= accSpeed(lowSpeed);
    }

    ROS_INFO("FollowingWall: msg.angular.z is %f,msg.linear.x is%f",msg.angular.z,msg.linear.x);
    //publishing message
    twist_mutex_.lock();
    pubMessage.publish(msg);
    twist_mutex_.unlock();
}
double NodeWallFollowing::accSpeed(double speed){
    double vel;
    if(robot_twist.linear<speed)
        vel=robot_twist.linear+0.025;
    else if(robot_twist.linear>speed)
        vel=robot_twist.linear-0.025;
    else
        vel=robot_twist.linear;
    return vel;
}
int NodeWallFollowing::finishtask(){
    return result;
}
