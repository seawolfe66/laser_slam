This package is for changing planner method of local and global planner respectively when moving using move_base .

Global Planner :

  *  There are two files to implement global planning changing .
     1. cleaner_global_planner.cpp 
     2. cleaner_global_planner.h 

  *  Class name for base_global_planner parameter of move_base : cleaner_global_planner/CleanerGlobalPlanner

  *  We provide a service to change the planner dynamically.You can call service using ServiceClient with server name and pass the global planner name used for move_base.
     e.g. 
            ros::NodeHandle private_nh_("~");
            ros::ServiceClient globalPlanner = private_nh_.serviceClient<cleaner_planner_plugin::ChangePlanner>("/move_base/CleanerGlobalPlanner/change_planner");
            cleaner_planner_plugin::ChangePlanner srv_global;
            srv_global.request.newPlanner = "global_planner/GlobalPlanner";// Here you can replace it to the name you want to use
            if(globalPlanner.call(srv_global))
                ROS_INFO("Change global planner to %s from %s successfully",srv_global.request.newPlanner.c_str(),srv_global.response.oldPlanner.c_str());
     Notes:  If you just want to get current global planner name then you can set the newPlanner as empty ,then the call function will return the response with current planner name specified by the oldPlanner.
Local Planner :

  *  There are two files to implement Local planning changing .
     1. cleaner_Local_planner.cpp 
     2. cleaner_Local_planner.h 

  *  Class name for base_Local_planner parameter of move_base : cleaner_local_planner/CleanerLocalPlanner

  *  We provide a service to change the planner dynamically.You can call service using ServiceClient with server name and pass the Local planner name used for move_base.
     e.g. 
           ros::NodeHandle private_nh_("~");
           ros::ServiceClient localPlanner = private_nh_.serviceClient<cleaner_planner_plugin::ChangePlanner>("/move_base/CleanerLocalPlanner/change_planner");
           cleaner_planner_plugin::ChangePlanner srv_local;
           srv_local.request.newPlanner = "ftc_local_planner/FTCPlanner"; // Here you can replace it to the name you want to use 
           if(localPlanner.call(srv_local))
                ROS_INFO("Change local planner to %s from %s successfully",srv_local.request.newPlanner.c_str(),srv_local.response.oldPlanner.c_str());
    Notes:  If you just want to get current local planner name then you can set the newPlanner as empty ,then the call function will return the response with current planner name  specified by the oldPlanner.
