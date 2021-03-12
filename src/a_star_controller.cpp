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
    send_next();
}

void aStarNodelet::motorDone_cb(const std_msgs::Bool::ConstPtr& value) {
    if (value->data && listDir_.size() > 0) {
        send_next();
    } else {
        std_msgs::Bool yes;
        yes.data = true;
        pub_finishedMove.publish(yes);
    }
}

void aStarNodelet::pose_cb(const behaviors::coordinate::ConstPtr& pose) {
    // have to update pose because the config map is whack
    // y is good
    currY_ = pose->Y;
    currX_ = pose->X + (.5 * botWidth);
}

void aStarNodelet::addGhost_cb(const std_msgs::Int8::ConstPtr& val) {
    std::pair<float, float> loc;
    float width = 10;
    float height = 1;
    switch(val->data) {
        case 1:
            loc.first = 24.5;
            loc.second = 26.5;
            break;
        case 2:
            loc.first =  59.5;
            loc.second = 26.5;
            break;
        case 3:
            loc.first = 24.5;
            loc.second = 11.5;
            break;
        case 4:
            loc.first = 59.5;
            loc.second = 11.5;
            break;
        case 5:
            loc.first = 46.5;
            loc.second = 12;
            height = 10;
            width = 1;
    }

    float x = 0;
    float y =0;
    // convert into x and y
    for (int i = 0; i < loc.first; i += .5) {
        x++;
    }
    for (int i = 0; i < loc.second; i += .5) {
        y++;
    }
    configMap_.addWall(x, y, width, height);

}
void aStarNodelet::rmGhost_cb(const std_msgs::Int8::ConstPtr& val) {
    std::pair<float, float> loc;
    float width = 10;
    float height = 1;
    switch(val->data) {
        case 1:
            loc.first = 24.5;
            loc.second = 26.5;
            break;
        case 2:
            loc.first =  59.5;
            loc.second = 26.5;
            break;
        case 3:
            loc.first = 24.5;
            loc.second = 11.5;
            break;
        case 4:
            loc.first = 59.5;
            loc.second = 11.5;
            break;
        case 5:
            loc.first = 46.5;
            loc.second = 12;
            height = 10;
            width = 1;
    }

    float x = 0;
    float y =0;
    // convert into x and y
    for (int i = 0; i < loc.first; i += .5) {
        x++;
    }
    for (int i = 0; i < loc.second; i += .5) {
        y++;
    }
    configMap_.rmWall(x, y, width, height);
}

void aStarNodelet::send_next() {
    float dir = listDir_.at(0);

    // decode 
    std_msgs::Float64 msg;
    if (dir >= 0) {
        // have to change if moving in X bc of the whacky config map
        if (int(heading) == 90 || int(heading) == 270 ) dir - (.5 * botWidth);
        msg.data = dir;
        pub_motorForward.publish(msg);
    } else if (int(dir) == -1 ) {
        std_msgs::Bool turnR;
        turnR.data = true;
        pub_turnR.publish(turnR);

        // update heading 
        msg.data = int(heading + 90) % 360;
        pub_heading.publish(msg);
    } else if (int(dir) == -2 ) {
        std_msgs::Bool turn180;
        turn180.data = true;
        pub_turn180.publish(turn180);
        
        // update heading 
        msg.data = int(heading + 180) % 360;
        pub_heading.publish(msg);
    } else  {
        std_msgs::Bool turnL;
        turnL.data = true;
        pub_turnL.publish(turnL);

        // update heading 
        msg.data = int(heading - 90);
        if (heading < 0) heading = 270; // only one edge case where it goes neg
        pub_heading.publish(msg);
    }

    // pop the front
    listDir_.erase(listDir_.begin());
}

//override
void aStarNodelet::onInit(){
    // initialize subs
    sub_goTo = nh.subscribe("goTo", 1000, &aStarNodelet::goTo_cb, this);
    sub_motorDone = nh.subscribe("/bot/doneMove", 1000, &aStarNodelet::motorDone_cb, this);
    sub_pose = nh.subscribe("position", 1000, &aStarNodelet::pose_cb, this);
    sub_addGhost = nh.subscribe("ghostLocation", 1000, &aStarNodelet::addGhost_cb, this);
    sub_rmGhost = nh.subscribe("grabbedGhost", 1000, &aStarNodelet::rmGhost_cb, this);

    // init publishers
    pub_motorForward = nh.advertise<std_msgs::Float64>("/bot/move", 1000);
    pub_turnR = nh.advertise<std_msgs::Bool>("/bot/turnRight", 1000);
    pub_turnL = nh.advertise<std_msgs::Bool>("/bot/turnLeft", 1000);
    pub_turn180 = nh.advertise<std_msgs::Bool>("/bot/turn180", 1000);
    pub_finishedMove = nh.advertise<std_msgs::Bool>("moveDone", 1000);
    pub_heading = nh.advertise<std_msgs::Float64>("heading", 1000);

    // grab params 
    nh.getParam("startingHeading", heading);
    nh.getParam("botWidth", botWidth);
}


}