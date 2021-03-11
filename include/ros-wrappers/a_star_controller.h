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

// random includes
#include <vector>

namespace controller_nodes {
class aStarNodelet : public nodelet::Nodelet {
public:
    // init and deinit
    aStarNodelet();
    ~aStarNodelet();

    //override
    void onInit() override;


    // ros subscribers and call backs
    ros::Subscriber sub_goTo, sub_motorDone, sub_heading, sub_pose;
    void goTo_cb (const behaviors::coordinate::ConstPtr& value);
    void motorDone_cb(const std_msgs::Bool::ConstPtr& value);
    void heading_cb(const std_msgs::Float64::ConstPtr& head);
    void pose_cb(const behaviors::coordinate::ConstPtr& pose);
    
    // ros publishers
    ros::Publisher pub_motorTurn, pub_motorForward,
                   pub_finishedMove;

    // utils
    void send_next();
private:
    // list of directions to go to
    std::vector<float> listDir_;

    controllers::map configMap_;

    float currX_, currY_;
    a_star::direction heading_;
};
}