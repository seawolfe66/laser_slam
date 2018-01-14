#ifndef SR_NODE_WALL_FOLLOWING
#define SR_NODE_WALL_FOLLOWING

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "dem_wall_following/psdtuneConfig.h"
#include "kobuki_msgs/BumperEvent.h"
#include "dynamic_reconfigure/server.h"
#include <std_msgs/String.h>
#include  <vector>
#include <boost/thread.hpp>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>


//common define 
#define  D2R(angle)   ( (angle) * 0.017453)
#define  R2D(angle)   ( (angle) * 57.2957795)
#define  ultraRange    6.0
#define  wallRange	   6.0
//#define  sim_test	    1 

#define PI 3.141592		//!<Mathematical constant (default value: 3.141592).


#define SUBSCRIBER_BUFFER_SIZE 10	//!<Size of buffer for subscriber.

#define PUBLISHER_BUFFER_SIZE 1000	//!<Size of buffer for publisher.

// #define PUBLISHER_TOPIC "/syros/baserostopic info /mobile_base/events/bumper _cmd_vel"

//#define PUBLISHER_TOPIC "/robot0/cmd_vel"
#define PUBLISHER_TOPIC "/cmd_vel_mux/input/teleop"
// #define SUBSCRIBER_TOPIC "/syros/laser_laser"

//#define SUBSCRIBER_TOPIC "/robot0/laser_0"
#define SUBSCRIBER_TOPIC "/scan"

#define SUBSCRIBER_BUMPER_TOPIC "/mobile_base/events/bumper"

#define SUBSCRIBER_BOW_TOPIC "/bow_command"

#define SUBSCIBER_ODOM_TOPIC "/odom"

/*! \brief Demonstration task: "Wall Following"
 * 
 * This class controls robot. Robot finds nearest wall and goes along it.
 */
typedef struct pose{
    float x;
    float y;
}Pose;
typedef struct{
	float linear;
	float angular;
}twist;
typedef struct{
	bool left;
	bool center;
	bool right;
}BUMPER;
enum StateType
{
	ID_init, 	      	/*0*/
    ID_lookingWall, 	/*1*/
    ID_parallWall,		 /*2*/
    ID_followingWall, 	/*3*/
    ID_bumper, 			/*4*/
    ID_frontObs 		/*5*/
};

 class NodeWallFollowing
{
public:
	
	enum RESULT {
		WAIT,					/*0*/
		SUCCESS,      			/*1*/
		FAIL,                 	/*2*/ 
		NEXT,					/*3*/
		ALTERNATIVE				/*4*/
	
	};
	/*! \brief A constructor.
	 * 
	 * @param pub Publisher, which can send commands to robot.
	 * @param wallDist Desired distance from the wall.
	 * @param maxSp Maximum speed, that robot can go.
	 * @param dir 1 for wall on the right side of the robot (-1 for the left one).
	 * @param pr P constant for PSD regulator.
	 * @param su S constant for PSD regulator.
	 * @param di D constant for PSD regulator.
	 * @param an Angle coeficient for regulator.
	 */
	NodeWallFollowing(void);
	
	/*! \brief A destructor.
	 */
	~NodeWallFollowing();

	/*! \brief This method publishes commands for robot.
	 *
	 * Commands are generated from data, which are stored in variables
	 * (#angleMin, #angleMax, #distMin, #distMax). If the robot is far from 
	 * nearest obstacle (distMin is bigger than desired distance plus 
	 * constant), it turns to the obstacle and goes there. If it is near 
	 * the desired distance, it turns and goes along the obstacle. 
	 * The whole time, when robot is going along the obstacle (wall), it 
	 * tries to keep desired distance and angle from the wall. The higher
	 * is the diference between desired and actual values, the lower is 
	 * speed.
	 */
	/*! \brief This method reads data from sensor and processes them to variables.
	 * 
	 * This method finds maximum and minimum distance in data from sensor
	 * and stores these values (with appropriate angles) into variables: 
	 * #angleMin, #angleMax, #distMin, #distMax.
	 * 
	 * @param msg Message, which came from robot and contains data from
	 * laser scan.
	 */
	
    void messageCallback(const sensor_msgs::LaserScan& scan);
    void bumperCallback(const kobuki_msgs::BumperEventConstPtr & bmsg);
  //  void reconfigureCB(dem_wall_following::psdtuneConfig &config, uint32_t level);
    void bowCmdCallback(const std_msgs::String& move_cmd);
	void odomCallback(const nav_msgs::Odometry& odom_try);
	int  finishtask();


//variables
private:

	void robotStateDetect(void);
	double angleRanging(const sensor_msgs::LaserScan& scan, double angle);
	void lookingWall(double angle,double distance);
	void parallaWall(double left,double front);
	void followingWall();
	void monitorCircle(double x,double y);
	void robotStateExecute(void);
	void bumperWall(double distance);
	bool adjustAngle(void);
	double accSpeed(double speed);

	double frontWall;//!<Desired distance from the wall.
	double leftWall;
	double stopDistance;//!<Desired  stop distance from the wall
	double error_left;			//!<Difference between desired distance from the wall and actual distance.
	double error_front;
	double sum;		//!<Sum of #r.
	double maxSpeed;	//!<Maximum speed of robot.
	double middSpeed;
	double lowSpeed;	//!<low　Speed of robot
	double maxAngular;

	double left_distance;
	double front_distance;
	double  vertical_distance;
	double rotateKp;
	//double accSpeed;
	double Kp;			//!<P constant for PSD regulator.
	double Ki;			//!<S constant for PSD regulator.

	double Kp_lw;
	double Kp_pw;
	double Kp_aw;
	int direction;		//!<1 for wall on the right side of the robot (-1 for the left one).
	double angleMin;	//!<Angle, at which was measured the shortest distance.
	double distMin;		//!<Minimum distance masured by ranger.
	double distFront;	//!<Distance, measured by ranger in front of robot.
						//!<If the obstacle is in front of robot, change to 
    dynamic_reconfigure::Server<dem_wall_following::psdtuneConfig> *server_;
	dynamic_reconfigure::Server<dem_wall_following::psdtuneConfig>::CallbackType f_;
	
	tf::TransformListener mTransformListener;
	
	bool switch_state_;
	
    boost::mutex twist_mutex_;
	double  init_yaw;
	double  current_yaw;
	int flag_turn;
	int stop;
	int count;
	int point_count;
	bool first;
	pose init_pose;
	std::vector<Pose> robot_pose; 
	twist robot_twist;
	double angleAccuracy;
	BUMPER bumper;
	StateType  robot_state;
	RESULT  result;
	double minRadius;
	ros::Subscriber scan_sub,bumper_sub,bow_sub_,odom_sub_;
	ros::Publisher pubMessage;   	//!<Object for publishing messages.
	//int flag_topic;
	
	//tf::Stamped<tf::Pose> current_pose;
};

#endif
