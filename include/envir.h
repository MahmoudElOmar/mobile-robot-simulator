#ifndef ENVIR_H
#define ENVIR_H

#include <vector>
#include "./geom.h"


using namespace std;

namespace arpro {
    class Robot;

    class Environment {
        public:
            vector<Pose> walls;
            Pose target_;
            vector<double> x_hist,y_hist;
            vector<Robot*> robots_;

            double dt = 0.1;
            double t = 0;

            Environment();

            double time() const {
                return t;
            }
            // the target draws a cardoid curve

            void updateTarget();

            Pose target() const {
                return target_; //target is pose
            }
            void addRobot(Robot &_robot);
            // plots the trajectory in the given environment

            void plot();
    };
}

#endif // ENVIR_H