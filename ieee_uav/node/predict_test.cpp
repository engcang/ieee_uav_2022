// #include "../include/bezier_test/predict.h"
#include <bezier_test/predict.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "predict_test");
    cout << "1" << endl;
    bezier_traj_class obj();
    cout << "2" << endl;

    signal(SIGINT, signal_handler); // to exit program when ctrl+c

    ros::AsyncSpinner spinner(6); // Use 8 threads -> 3 callbacks + 2 Timer callbacks + 1 spare threads for publishers
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
        
