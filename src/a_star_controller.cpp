#include <ros-wrappers/a_star_controller.h>

// export 
PLUGINLIB_EXPORT_CLASS(controller_nodes::aStarNodelet, nodelet::Nodelet);

namespace controller_nodes {
// initializers and deinitializers
aStarNodelet::aStarNodelet(){
    controllers::map real;
    real = map_builder::build_real_pman_map();
    configMap_ = map_builder::build_config_space_map(real);
}
aStarNodelet::~aStarNodelet(){}

// callback functions 
void aStarNodelet::goTo_cb (const behaviors::coordinate::ConstPtr& target) {
    listDir_ = a_star::a_star(currX_, currY_, heading_, target->X, target->Y, configMap_);
}

void aStarNodelet::motorDone_cb(const std_msgs::Bool::ConstPtr& value) {
    if (value->data && listDir_.size() > 0) {
        send_next();
    }
}

void aStarNodelet::send_next() {
    float dir = listDir_.at(0);

    // decode 
    // if (dir >= 0) {
    //     std_msgs::Float64 msg;
    //     msg.data = dir;
    //     pub_motorForward.publish(dir);
    // } else if (int(dir) == -1 ) {
    //     std_msgs::Float64 msg;
    //     msg.data = 90;
    //     pub_motorTurn.publish(90);
    // } else if (int(dir) == -2 ) {
    //     std_msgs::Float64 msg;
    //     msg.data = 180;
    //     pub_motorTurn.publish(msg);
    // } else  {
    //     std_msgs::Float64 msg;
    //     msg.data = -90;
    //     pub_motorTurn.publish(msg);
    // }

    // pop the front
    listDir_.erase(listDir_.begin());
}

//override
void aStarNodelet::onInit(){
    // initialize 
}


}