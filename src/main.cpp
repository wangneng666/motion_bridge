#include "motion.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_bridge");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(6);
    spinner.start();
    motion mt(&node_handle);
    ros::waitForShutdown();
    return 0;
}
