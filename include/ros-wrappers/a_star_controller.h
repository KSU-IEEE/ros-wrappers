#ifndef A_STAR_CONTROLLER_H
#define A_STAR_CONTROLLER_H

// a_star includes
#include <mapping/map.h>
#include <mapping/map_builder.h>
#include <node/a_star_node.h>
#include <node/a_star_logic.h>

// ros includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <behaviors/coordinate.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

// random includes
#include <vector>
#include <utility>

namespace controller_nodes {
class aStarNodelet : public nodelet::Nodelet {
public:
    // init and deinit
    aStarNodelet();
    ~aStarNodelet();

    //override
    void onInit() override;

    ros::NodeHandle nh;

    // ros subscribers and call backs
    ros::Subscriber sub_goTo, sub_motorDone, sub_pose, sub_addGhost, sub_rmGhost;
    void goTo_cb (const behaviors::coordinate::ConstPtr& value);
    void motorDone_cb(const std_msgs::Bool::ConstPtr& value);
    void pose_cb(const behaviors::coordinate::ConstPtr& pose);
    void addGhost_cb(const std_msgs::Int8::ConstPtr& val);
    void rmGhost_cb(const std_msgs::Int8::ConstPtr& val);
    
    // ros publishers
    ros::Publisher pub_motorForward, pub_finishedMove, pub_heading;
    ros::Publisher pub_turnR, pub_turnL, pub_turn180;

    // utils
    void send_next();
private:
    // list of directions to go to
    std::vector<float> listDir_;

    controllers::map configMap_;

    float currX_, currY_;
    a_star::direction heading_;

    // things for it publish
    float heading;

    // to load from params
    float botWidth;
};
}

#endif // A_STAR_CONTROLLER_H