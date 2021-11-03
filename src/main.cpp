#include <iostream>
#include <math.h>
#include <cmath>

#include "../include/robot.h"
#include "../include/envir.h"
#include "../include/sensor.h"
#include "../include/sensor_range.h"
#include "../include/sensor_bearing.h"

using namespace std;
using namespace arpro;

int main(int argc, char **argv)
{

    // default environment with moving target
    Environment envir;

    // sensors gets measurements from this environment
    Sensor::setEnvironment(envir);


    // init robot at (0,0,0)
    Robot robot("R2D2", 0,0,0);
    robot.initWheel(0.07,0.3,10);
    envir.addRobot(robot);

    RangeSensor range_sensor(robot,0.1,0,0,0.1,0.1);

    Robot robot2("C3PO",0,-1,0);
    robot2.initWheel(0.05,0.3,10);
    envir.addRobot(robot2);

    BearingSensor bearing_sensor(robot2,0.1,0.0,0.0);
    
    // simulate 100 sec
    while(envir.time() < 105)
    {

        // update target position
        envir.updateTarget();

        // try to follow target
        robot.goTo(envir.target());


        //move forware at 0.4 m/s
        robot2.moveWithSensor(Twist(0.4,0,0));

    }

    // plot trajectory
    envir.plot();

}