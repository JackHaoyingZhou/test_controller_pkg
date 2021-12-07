
 #include "ros/ros.h"
#include "controller_modules/ControllerManager.h"
#include "controller_modules/PDController.h"
#include "controller_modules/GravityCompensationController.h"
#include "controller_modules/JointControl.h"
#include "controller_modules/ModelController.h"

#include "Eigen/Core"

#include "boost/shared_ptr.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Control_Manager");
    ros::NodeHandle n;


    // set up gains for the kuka arm
    Eigen::MatrixXd Kp = Eigen::MatrixXd::Ones(8,8);
    Eigen::MatrixXd Kd = Eigen::MatrixXd::Zero(8,8);
    
    Kp(0,0) = 0.01;
    Kp(1,1) = 0.01;
    Kp(2,2) = 0.01;
    Kp(3,3) = 0.01;
    Kp(4,4) = 0.01;
    Kp(5,5) = 0.01;
    Kp(6,6) = 0.01;
    Kp(7,7) = 0.01;

    Kd(0,0) = 0.0004;
    Kd(1,1) = 0.0004;
    Kd(2,2) = 0.0004;
    Kd(3,3) = 0.004;
    Kd(4,4) = 0.0004;
    Kd(5,5) = 0.0004;
    Kd(6,6) = 0.00001;
    Kd(7,7) = 0.00001;
  
    PDController controller(Kp,Kd);

    // create a control manager
    ControllerManager manager(&n);

    // create and add a basic PD controll for the kuka
    boost::shared_ptr<ControllerBase> PD_controller(&controller);
    manager.addController("kuka_PD", PD_controller);

    // create and add a model controller for the kuka
    boost::shared_ptr<ControllerBase> kuka_model_controller(new ModelController("kuka", &n,&controller ));
    manager.addController("kuka_model_controller", kuka_model_controller);

    // create and add a gravity controller for the kuka
    boost::shared_ptr<ControllerBase> kuka_grav(new GravityCompensationController("kuka",&n));
    manager.addController("kuka_gravity", kuka_grav);

    // create and add a gravity controller for a custom robot
    boost::shared_ptr<ControllerBase> my_robot_grav(new GravityCompensationController("my_robot",&n));
    manager.addController("my_robot_gravity", my_robot_grav);
    
    

    ros::spin();


    return 0;
}
