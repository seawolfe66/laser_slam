#include <frontier_exploration/bounded_explore_layer.h>

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/footprint.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <frontier_exploration/Frontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/frontier_search.h>
#include <frontier_exploration/geometry_tools.h>
#include <visualization_msgs/Marker.h>
#include <glog/logging.h>
#include <glog/vlog_is_on.h>
PLUGINLIB_EXPORT_CLASS(frontier_exploration::BoundedExploreLayer, costmap_2d::Layer)

namespace frontier_exploration
{

    using costmap_2d::LETHAL_OBSTACLE;//254
    using costmap_2d::NO_INFORMATION;//255
    using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;//253
    using costmap_2d::FREE_SPACE;//0

    BoundedExploreLayer::BoundedExploreLayer(){}

    BoundedExploreLayer::~BoundedExploreLayer(){
       // polygonService_.shutdown();
       // frontierService_.shutdown();
        costmapPolygonService_.shutdown();
        delete dsrv_;
        dsrv_ = 0;
    }

    void BoundedExploreLayer::onInitialize(){

        ros::NodeHandle nh_("~/" + name_);
        frontier_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("frontiers",5);
        frontier_point_cloud_pub1 = nh_.advertise<sensor_msgs::PointCloud>("polygon_point",50);
        sweeps_areas_sub_ = nh_.subscribe("/RobotSweepsAreas", 10,  &BoundedExploreLayer::syncSweepAreas, this);
        point_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("frontier_polygonline", 10);
        //publish RectanglePolygon
	rectanglePolygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("/RectanglePolygon",50);
        configured_ = false;
        marked_ = false;

        bool explore_clear_space;
        nh_.param("explore_clear_space", explore_clear_space, true);
        if(explore_clear_space){
            default_value_ = NO_INFORMATION;
        }else{
            default_value_ = FREE_SPACE;
        }

        matchSize();

        nh_.param<bool>("resize_to_boundary", resize_to_boundary_, true);
        nh_.param<std::string>("frontier_travel_point", frontier_travel_point_, "closest");

        costmapPolygonService_ = nh_.advertiseService("get_costmap_polygon", &BoundedExploreLayer::getCostmapPolygonService, this);


        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh_);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
                    &BoundedExploreLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);


    }


    void BoundedExploreLayer::matchSize(){
        Costmap2D* master = layered_costmap_->getCostmap();
        resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
                  master->getOriginX(), master->getOriginY());
    }


    void BoundedExploreLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
        enabled_ = config.enabled;
    }



    /** get the frontier polygon */
    bool BoundedExploreLayer::getFrontierPolygon(geometry_msgs::Polygon &frontier_polygon){
        frontier_list_copy.clear();
        int number = frontier_list_new.size();
       
        //Frontier selected;
        geometry_msgs::Point selected; 
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
        //get the pos into the polygon and point clound
        int max_all;
        max_all = frontier_list_copy.size();
        sensor_msgs::PointCloud cloud;
        cloud.header.stamp = ros::Time::now();
        cloud.header.frame_id = "/map";

        cloud.points.resize(max_all);

        //we'll also add an intensity channel to the cloud
        cloud.channels.resize(1);
        cloud.channels[0].name = "intensities";
        cloud.channels[0].values.resize(max_all);

         
         BOOST_FOREACH(geometry_msgs::Point frontier, frontier_list_copy){
            //load frontier into visualization poitncloud
            cloud.points[index].x = frontier.x;
            cloud.points[index].y = frontier.y;
            cloud.points[index].z = frontier.z;
            cloud.channels[0].values[index] = 100;
            index++; 

            frontier_polygon.points.push_back(costmap_2d::toPoint32(frontier));
     
        }
         //show the clound point 
        frontier_point_cloud_pub1.publish(cloud);
        //show the polygon line as following

        visualization_msgs::Marker points, line_strip;

        points.ns = line_strip.ns = "explore_points";
        line_strip.header.stamp = ros::Time::now();
        line_strip.header.frame_id = "/map";
        points.header.frame_id = "/map";

        points.id = 0;
        line_strip.id = 1;

        points.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        //line_strip.type = visualization_msgs::Marker::LINE_LIST; 
        if(!frontier_polygon.points.empty()){

            points.action = line_strip.action = visualization_msgs::Marker::ADD;
            points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

            points.scale.x = points.scale.y = 0.03;
            line_strip.scale.x = 0.01;

            BOOST_FOREACH(geometry_msgs::Point32 point, frontier_polygon.points){
                line_strip.points.push_back(costmap_2d::toPoint(point));
            }

           ROS_ERROR(">>>>chenrui>>>write the line strip>>>"); 
           line_strip.points.push_back(costmap_2d::toPoint(frontier_polygon.points.front()));
           if(line_strip.points.size()%2 != 0)
               line_strip.points.push_back(costmap_2d::toPoint(frontier_polygon.points.front()));

           points.color.a = line_strip.color.a = 0.5;
            points.color.r = line_strip.color.r =  points.color.g = line_strip.color.g = 1.0;
           
            
        }
        point_viz_pub_.publish(points);
        point_viz_pub_.publish(line_strip);
        return true;

    }


    bool BoundedExploreLayer::getCostmapPolygonService(frontier_exploration::CostmapPolygon::Request &req, frontier_exploration::CostmapPolygon::Response &res){
        return getCostmapPolygon(req.costmap_boundary, res.sweeps_boundary);
    }

    void BoundedExploreLayer::syncSweepAreas(const sweeps_areas::sweeps & areas){
          sweeps_areas_polygon = areas;
    }

    void BoundedExploreLayer::reset(){

        //reset costmap_ char array to default values
        marked_ = false;
        configured_ = false;
        memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
    }

    bool BoundedExploreLayer::getCostmapPolygon(geometry_msgs::PolygonStamped costmap_polygon_stamped, geometry_msgs::PolygonStamped& sweeps_polygon_stamped){
        //clear existing boundary, if any
        polygon_.points.clear();
        //error if no transform available between polygon and costmap
        if(!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), costmap_polygon_stamped.header.frame_id,ros::Time::now(),ros::Duration(10))) {
            ROS_ERROR_STREAM("Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< costmap_polygon_stamped.header.frame_id);
            return false;
        }
	//publish input_rectangle_polygon_
	input_rectangle_polygon_ = costmap_polygon_stamped;
	rectanglePolygon_pub_.publish(input_rectangle_polygon_);
	ROS_ERROR("publish rectangle polygon");


        //Transform all points of boundary polygon into costmap frame
        geometry_msgs::PointStamped in, out;
        in.header = costmap_polygon_stamped.header;
        BOOST_FOREACH(geometry_msgs::Point32 point32, costmap_polygon_stamped.polygon.points){
            in.point = costmap_2d::toPoint(point32);
            tf_listener_.transformPoint(layered_costmap_->getGlobalFrameID(),in,out);
            polygon_.points.push_back(costmap_2d::toPoint32(out.point));
        }
        //if empty boundary provided, set to whole map
        if(polygon_.points.empty()){
            geometry_msgs::Point32 temp;
            temp.x = getOriginX();
            temp.y = getOriginY();
            polygon_.points.push_back(temp);
            temp.y = getSizeInMetersY();
            polygon_.points.push_back(temp);
            temp.x = getSizeInMetersX();
            polygon_.points.push_back(temp);
            temp.y = getOriginY();
            polygon_.points.push_back(temp);
        }
        if(resize_to_boundary_){
            updateOrigin(0,0);

            //Find map size and origin by finding min/max points of polygon
            double min_x = std::numeric_limits<double>::infinity();
            double min_y = std::numeric_limits<double>::infinity();
            double max_x = -std::numeric_limits<double>::infinity();
            double max_y = -std::numeric_limits<double>::infinity();

            BOOST_FOREACH(geometry_msgs::Point32 point, polygon_.points){
                min_x = std::min(min_x,(double)point.x);
                min_y = std::min(min_y,(double)point.y);
                max_x = std::max(max_x,(double)point.x);
                max_y = std::max(max_y,(double)point.y);
            }


            //resize the costmap to polygon boundaries, don't change resolution
            int size_x, size_y;
            start_pose.header.stamp = ros::Time::now();
            start_pose.header.frame_id = layered_costmap_->getGlobalFrameID();
            start_pose.pose.position.x = (min_x + max_x)/2.0;
            start_pose.pose.position.y = (max_y + min_y)/2.0;
            start_pose.pose.position.z = 0;

            worldToMapNoBounds(max_x - min_x, max_y - min_y, size_x, size_y);
            layered_costmap_->resizeMap(size_x, size_y, layered_costmap_->getCostmap()->getResolution(), min_x, min_y);
            matchSize();
        }
        configured_ = true;
        marked_ = false;
        //wait for costmap to get marked with boundary
        ros::Rate r(10);
        while(!marked_){
            ros::spinOnce();
            r.sleep();
        }
        //initialize frontier search implementation
        FrontierSearch frontierSearch(*(layered_costmap_->getCostmap()),frontier_list_new);
        //FrontierSearch frontierSearch(*(layered_costmap_->getCostmap()));
        //get list of frontiers from search implementation
        frontier_list.clear();
        frontier_list = frontierSearch.searchFrom(start_pose.pose.position);

        if(frontier_list.size() == 0){
            ROS_DEBUG("No frontiers found, exploration complete");
            return false;
        }

        //create placeholder for selected frontier
        Frontier selected;
        selected.min_distance = std::numeric_limits<double>::infinity();

        //pointcloud for visualization purposes
        pcl::PointCloud<pcl::PointXYZI> frontier_cloud_viz;
        pcl::PointXYZI frontier_point_viz(100);
        int max;

        BOOST_FOREACH(Frontier frontier, frontier_list){
            //load frontier into visualization poitncloud
            frontier_point_viz.x = frontier.initial.x;
            frontier_point_viz.y = frontier.initial.y;
            frontier_cloud_viz.push_back(frontier_point_viz);
            //check if this frontier is the nearest to robot
            if (frontier.min_distance < selected.min_distance){
                selected = frontier;
                max = frontier_cloud_viz.size()-1;
            }
        }

        //color selected frontier
        frontier_cloud_viz[max].intensity = 100;

        //publish visualization point cloud
        sensor_msgs::PointCloud2 frontier_viz_output;
        pcl::toROSMsg(frontier_cloud_viz,frontier_viz_output);
        frontier_viz_output.header.frame_id = layered_costmap_->getGlobalFrameID();
        frontier_viz_output.header.stamp = ros::Time::now();
        frontier_cloud_pub.publish(frontier_viz_output);

        output_polygon_.points.clear();
        getFrontierPolygon(output_polygon_);

        sweeps_polygon_stamped.polygon = output_polygon_;
        sweeps_polygon_stamped.header.frame_id = layered_costmap_->getGlobalFrameID();
        sweeps_polygon_stamped.header.stamp = ros::Time::now();

        return true;

    }


    void BoundedExploreLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y){

        //check if layer is enabled and configured with a boundary
        if (!enabled_ || !configured_){ return; }
        //update the whole costmap
        *min_x = getOriginX();
        *min_y = getOriginY();
        *max_x = getSizeInMetersX()+getOriginX();
        *max_y = getSizeInMetersY()+getOriginY();
    }

    void BoundedExploreLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        //check if layer is enabled and configured with a boundary
        if (!enabled_ || !configured_){ return; }
        //draw lines between each point in polygon
        MarkCell marker(costmap_, LETHAL_OBSTACLE);

        //circular iterator
        for(int i = 0, j = polygon_.points.size()-1; i < polygon_.points.size(); j = i++){

            int x_1, y_1, x_2, y_2;
            worldToMapEnforceBounds(polygon_.points[i].x, polygon_.points[i].y, x_1,y_1);
            worldToMapEnforceBounds(polygon_.points[j].x, polygon_.points[j].y, x_2,y_2);

            raytraceLine(marker,x_1,y_1,x_2,y_2);
        }
        //update the master grid from the internal costmap
        mapUpdateKeepObstacles(master_grid, min_i, min_j, max_i, max_j);
    }

    void BoundedExploreLayer::mapUpdateKeepObstacles(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        if (!enabled_)
            return;

        //ROS_ERROR("begin to BoundedExploreLayer::mapUpdateKeepObstacles");
        unsigned char* master = master_grid.getCharMap();
        unsigned int span = master_grid.getSizeInCellsX();
        double resolution_explore = master_grid.getResolution();

        for (int j = min_j; j < max_j; j++)
        {
            unsigned int itt = span*j+min_i;
            for (int i = min_i; i < max_i; i++)
            {
                //only update master grid if local costmap cell is lethal/higher value, and is not overwriting a lethal obstacle in the master grid
                if(master[itt] != LETHAL_OBSTACLE && (costmap_[itt] == LETHAL_OBSTACLE || costmap_[itt] > master[itt])){
               // if(master[it] != NO_INFORMATION && (costmap_[it] == NO_INFORMATION || costmap_[it] > master[it])){
                    master[itt] = costmap_[itt];

                }
                itt++;
            }

        }
         double map_origin_x = master_grid.getOriginX();
         double map_origin_y = master_grid.getOriginY();
         double sweeps_areas_polygon_origin_x = sweeps_areas_polygon.sweeps_areas.info.origin.position.x;
         double sweeps_areas_polygon_origin_y = sweeps_areas_polygon.sweeps_areas.info.origin.position.y;
         int map_origin_index_x = (int)((map_origin_x - sweeps_areas_polygon.sweeps_areas.info.origin.position.x)/resolution_explore);
         int map_origin_index_y = (int)((map_origin_y - sweeps_areas_polygon.sweeps_areas.info.origin.position.y)/resolution_explore);

        for(int ii = 0; ii < sweeps_areas_polygon.num; ii++){
            int sweeps_x_;
            int sweeps_y_;
            sweeps_x_ = (int)((sweeps_areas_polygon.sweeps_poses.poses[ii].position.x - map_origin_x)/resolution_explore);
            sweeps_y_ = (int)((sweeps_areas_polygon.sweeps_poses.poses[ii].position.y - map_origin_y)/resolution_explore);
            if((sweeps_x_ > min_i && sweeps_x_<max_i) && (sweeps_y_ > min_j && sweeps_y_<max_j)){
                unsigned int it_ = span*sweeps_y_ + sweeps_x_;
                master[it_] = 200;
            }
        }
        marked_ = true;

    }

}
