//
// Created by naveed on 15. 4. 28.
//

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

// Omega6
#include "../include/omega_device/dhdc.h"
#include "../include/omega_device/drdc.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "omega_talker");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("omega_input", 1000);

    ros::Rate loop_rate(10);


    // CODE FROM EXAMPLE
    double px, py, pz;
    double fx, fy, fz;
    double t1,t0  = dhdGetTime ();

    // center of workspace
    double nullPose[DHD_MAX_DOF] = { 0.0, 0.0, 0.0,  // base  (translations)
                                     0.0, 0.0, 0.0,  // wrist (rotations)
                                     0.0 };          // gripper


    int major, minor, release, revision;
    dhdGetSDKVersion (&major, &minor, &release, &revision);
    printf ("Force Dimension - Auto Center Gravity %d.%d.%d.%d\n", major, minor, release, revision);
    printf ("(C) 2014 Force Dimension\n");
    printf ("All Rights Reserved.\n\n");

    // required to change asynchronous operation mode
    dhdEnableExpertMode ();

    // open the first available device
    if (drdOpen () < 0) {
        printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr ());
        dhdSleep (2.0);
        return -1;
    }

    // print out device identifier
    if (!drdIsSupported ()) {
        printf ("unsupported device\n");
        printf ("exiting...\n");
        dhdSleep (2.0);
        drdClose ();
        return -1;
    }
    printf ("%s haptic device detected\n\n", dhdGetSystemName ());

    // perform auto-initialization
    if (!drdIsInitialized () && drdAutoInit () < 0) {
        printf ("error: auto-initialization failed (%s)\n", dhdErrorGetLastStr ());
        dhdSleep (2.0);
        return -1;
    }
    else if (drdStart () < 0) {
        printf ("error: regulation thread failed to start (%s)\n", dhdErrorGetLastStr ());
        dhdSleep (2.0);
        return -1;
    }

    // move to center
    drdMoveTo (nullPose);

    // stop regulation thread (but leaves forces on)
    drdStop (true);


    int count = 0;
    while (ros::ok())
    {

        /**
         * This is a message object. You stuff it with data, and then publish it.
         */

        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        // apply zero force
        if (dhdSetForceAndTorqueAndGripperForce (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR) {
            printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr ());
        }


        // write down position
        if (dhdGetPosition(&px, &py, &pz) < 0) {
            ROS_INFO("error: cannot read position (%s)\n", dhdErrorGetLastStr());
        }

        if (dhdGetForce(&fx, &fy, &fz) < 0) {
            ROS_INFO("error: cannot read force (%s)\n", dhdErrorGetLastStr());
        }

        ROS_INFO("p (%+0.03f %+0.03f %+0.03f) m  |  f (%+0.01f %+0.01f %+0.01f) N  \n", px,
               py, pz, fx, fy, fz);


        //ROS_INFO("%s\n", msg.data.c_str());

        t1 = dhdGetTime ();
        ROS_INFO("Time: %f", (t1-t0));
        t0 = t1;


        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();

        ++count;
    }


    return 0;
}