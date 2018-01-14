#include <cleaner_run_time/cleaner_run.h>

int main(int argc, char** argv) {
    // in order to replace the original move_base seamlessly we use its name
    ros::init(argc, argv, "cleaner");

    tf::TransformListener tf(ros::Duration(10));


    // a thread is started in the constructor
    cleaner_state_machine::CleanerRun cleaner_run("cleaner",tf);

    ros::Rate rate = ros::Rate(10.0);
    
    while (ros::ok()) {
        cleaner_run.runStep();
        ros::spinOnce();
        rate.sleep();
    }

    return(0);

}
