#include "sensor.h"
#include "stdio.h"
#include "geom.h"
#include "robot.h"
#include "envir.h"
#include <cmath>

namespace arpro {
    class RangeSensor : public Sensor {
        public:
            RangeSensor(Robot &_robot, double _x, double _y, double _theta,double gain, double sm) : 
                Sensor(_robot,_x,_y,_theta) {
                    // the RangeSensor constructor calls the base constructor and defines its unique 
                    // attributes in its own constructor
                    this->gain = gain;
                    this->sm = sm;
            }
            RangeSensor(Robot &_robot, double _x, double _y, double _theta) : 
                Sensor(_robot,_x,_y,_theta) {}
                
            void setSafetyLimits(double gain,double sm)    {
                this->gain = gain;
                this->sm = sm;
            }
            double getGain() {
                return this->gain;
            }
            double getSafetyMargin() {
                return this->sm;
            }
            void update(const Pose &p_) {
                Pose p1,p2;
                double c,s;
                double distance;

                c = cos(p_.theta);
                s = sin(p_.theta);

                s_ = 100; //initialize s_ with a very high value

                for(int i=0;i<envir_->walls.size();++i)
                {
                    p1 = envir_ ->walls[i];
                    p2 = envir_->walls[(i+1)%envir_->walls.size()];

                    // compute the distance between the sensor and the current wall
                    distance = (p1.x*p2.y - p1.x*p_.y - p2.x*p1.y + p2.x*p_.y + p_.x*p1.y-p_.x*p2.y)/(p1.x*s - p2.x*s - p1.y*c + p2.y*c);
                    if(distance > 0 && distance < s_)
                        s_ = distance;
                }
                
                //printf("Current Measurement : %f\n",s_);
                //printf("-------------------------------\n");
            }
            void correctTwist(Twist &_v) {
                if (s_ < 5) { // only correct the twist when robot is near the wall.
                    if (_v.vx > gain*(s_ - sm))
                        _v.vx = gain*(s_ - sm);
                }
            }
        protected:
            double gain,sm;

    };
}