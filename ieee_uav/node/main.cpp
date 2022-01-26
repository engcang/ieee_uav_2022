#include "include/ieee_unav.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trot-q_node");

    ieee_uav_class ieee_uav_();

    signal(SIGINT, signal_handler); // to exit program when ctrl+c

    ros::AsyncSpinner spinner(6); // Use 8 threads -> 3 callbacks + 2 Timer callbacks + 1 spare threads for publishers
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
