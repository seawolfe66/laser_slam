
#include <string>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace robot_fake_localization
{
  const double PI=3.1415926;
    class Fake_localization
    {
    public:
        Fake_localization();
        ~Fake_localization();
        void init();
        void velocityCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel);
        void integrate(double linear, double angular, double dt);
        void sum();
        void naviAndTfPub();

        void updateLoop(double frequency);
        float update_frequency;
        bool update_thread_shutdown_;
        boost::thread* update_thread_;  ///< @brief A thread for updating the pose
        
          /// fame name of header and child
        std::string fix_frame_id_;
        std::string child_frame_id_;
  
        ros::Subscriber velocitySub;
        ros::Publisher odomPublisher;
        tf::TransformBroadcaster tfBroadcaster;
        
    public:
        double x_init, y_init, heading_init, x_integrate, y_integrate, heading_integrate, x_sum, y_sum, heading_sum;
        double linear_, angular_;
    };

    Fake_localization::Fake_localization(){
        ros::NodeHandle nh;

        init();
        
        velocitySub = nh.subscribe("cmd_vel", 10, &Fake_localization::velocityCallback, this);
        odomPublisher = nh.advertise<nav_msgs::Odometry>("Odometry", 10);
        update_thread_ = new boost::thread(boost::bind(&Fake_localization::updateLoop, this, update_frequency));
  }

    Fake_localization::~Fake_localization()
    {
        update_thread_shutdown_ = true;
        if (update_thread_ != NULL)
        {
            update_thread_->join();
            delete update_thread_;
        }
    }

    void Fake_localization::init()
    {
      // fix_frame_id_ = "gridmap";
      //   child_frame_id_ = "base";
      //   linear_ = 0.0;
      //   angular_ = 0.0;
      //   x_init = 17.0;
      //   y_init = 14.6;

        fix_frame_id_ = "odom";
        child_frame_id_ = "base_footprint";
        linear_ = 0.0;
        angular_ = 0.0;
        x_init = 0.0;
        y_init = 0.0;
        heading_init = 0.0;
        x_integrate = y_integrate = heading_integrate = 0.0;
        x_sum = y_sum = heading_sum = 0.0;
        update_frequency = 30;
        update_thread_shutdown_ = false;
    }

    void Fake_localization::velocityCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
    {
      linear_ = cmd_vel->linear.x;
      angular_ = cmd_vel->angular.z;
      ROS_INFO("Received control command: [%f, %f]", linear_,angular_);
    }

    /**
     * \brief Other possible integration method provided by the class
     * \param linear speed
     * \param angular speed
     */
    void Fake_localization::integrate(double linear, double angular, double dt)
    {
      if (fabs(angular) < 1e-3)
      {
        x_integrate += linear * cos(heading_integrate)*dt;
        y_integrate += linear * sin(heading_integrate)*dt;
      }
      else
      {
        /// Exact integration (should solve problems when angular is zero):
        double heading_old;
        const double r = linear / angular;

        heading_old = heading_integrate;
        heading_integrate += angular*dt;
        heading_integrate = (heading_integrate > PI) ? (heading_integrate - 2 * PI) : ((heading_integrate < -PI) ? (heading_integrate + 2 * PI) : heading_integrate);
        x_integrate += r * (sin(heading_integrate) - sin(heading_old));
        y_integrate += -r * (cos(heading_integrate) - cos(heading_old));

      }
    }

    void Fake_localization::sum()
    {
        x_sum = x_init + x_integrate;
        y_sum = y_init + y_integrate;
        heading_sum = heading_init + heading_integrate;
        heading_sum = (heading_sum > PI) ? (heading_sum - 2 * PI) : ((heading_sum < -PI) ? (heading_sum + 2 * PI) : heading_sum);
        
    }

    void Fake_localization::naviAndTfPub()
    {
        /// Compute and store orientation info
        geometry_msgs::Quaternion orientation;

        /// Populate odom message and publish
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = fix_frame_id_;
        odom.child_frame_id = child_frame_id_;
        odom.pose.pose.position.x = x_sum;
        odom.pose.pose.position.y = y_sum;
        orientation = tf::createQuaternionMsgFromYaw(heading_sum);
        odom.pose.pose.orientation = orientation;
        odom.twist.twist.linear.x = linear_;
        odom.twist.twist.angular.z = angular_;
        odomPublisher.publish(odom);

        /// Publish tf /fix frame 
        geometry_msgs::TransformStamped fix_frame;
        fix_frame.header.stamp = ros::Time::now();
        fix_frame.header.frame_id = fix_frame_id_;
        fix_frame.child_frame_id = child_frame_id_;
        fix_frame.transform.translation.x = x_sum;
        fix_frame.transform.translation.y = y_sum;
        orientation = tf::createQuaternionMsgFromYaw(heading_sum);
        fix_frame.transform.rotation = orientation;
        tfBroadcaster.sendTransform(fix_frame);

        geometry_msgs::TransformStamped map_frame;
        map_frame.header.stamp = ros::Time::now();
        map_frame.header.frame_id = "map";
        map_frame.child_frame_id = fix_frame_id_;
        map_frame.transform.translation.x = 0;
        map_frame.transform.translation.y = 0;
        orientation = tf::createQuaternionMsgFromYaw(0);
        map_frame.transform.rotation = orientation;
        tfBroadcaster.sendTransform(map_frame);
    }

    void Fake_localization::updateLoop(double frequency)
    {
    // the user might not want to run the loop every cycle
      if (frequency == 0.0)
        return;

      ros::NodeHandle nh;
      ros::Rate r(frequency);
      double dt = 1/frequency;
      while (nh.ok() && !update_thread_shutdown_)
      {
        double linear = linear_;
        double angular = angular_;

        /// Integrate Fake_localization:
        integrate(linear, angular, dt);
        sum();
        naviAndTfPub();
        
        r.sleep();
      }

    }

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_localization");

  robot_fake_localization::Fake_localization fake_localization;
  ros::NodeHandle n;

  ros::Rate loop_rate(15); // 15Hz

  while (n.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return (0);
}
