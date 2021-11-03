#include "sensor.h"
#include "stdio.h"
#include "geom.h"
#include "robot.h"
#include "envir.h"
#include <math.h>


namespace arpro {
    double constrainAngle(double x) {
        x = fmod(x + M_PI,2*M_PI);
        if (x < 0)
            x += 2*M_PI;
        return x - M_PI;
    }
    class BearingSensor : public Sensor {
        public :
            BearingSensor(Robot &_robot, double _x, double _y, double _theta) : 
                        Sensor(_robot, _x,  _y,  _theta) {}
            void update(const Pose &_p) {
                //printf("This is the update method\n");
                for(auto other : envir_->robots_) {
                    if (other != robot_) {
                        s_ = atan2(other->pose().y - _p.y,other->pose().x - _p.x) - _p.theta;
                        break;
                    }
                }
                s_ = arpro::constrainAngle(s_); //set to [-pi,pi]

            }
            void correctTwist(Twist &_v) {
                //printf("This is the correctTwist method\n");
                _v.w -= gain*s_;
            }
        protected:
            double gain = 0.5;
    };

}