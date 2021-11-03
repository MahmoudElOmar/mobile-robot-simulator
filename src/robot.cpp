#include <iostream>
#include <math.h>
#include <robot.h>
#include <sensor.h>

using namespace arpro;
using namespace std;

Environment* Sensor::envir_ = nullptr;


//define two constuctors with different signatures 
Robot::Robot(string _name, double _x, double _y, double _theta,double r,double b)
{    
    pose_.x = _x;
    pose_.y = _y;
    pose_.theta = _theta;

    name_ = _name;

    //define wheel properties
    this->b = b;
    this->r = r;

    // init position history
    x_history_.push_back(_x);
    y_history_.push_back(_y);

    this->wheels_init = true;
}

Robot::Robot(string _name, double _x, double _y, double _theta)
{    
    pose_.x = _x;
    pose_.y = _y;
    pose_.theta = _theta;

    name_ = _name;

    // init position history
    x_history_.push_back(_x);
    y_history_.push_back(_y);
}

void Robot::initWheel(double r, double b, double w_limit) {
    this->r = r;
    this->b = b;
    this->w_limit = w_limit;
    this->wheels_init = true;

}
bool Robot::isWheelInit() {

    return this->wheels_init;
}

void Robot::moveXYT(double _vx, double _vy, double _omega)
{
    // update position
    pose_.x += _vx*dt_;
    pose_.y += _vy*dt_;
    pose_.theta += _omega*dt_;

    // store position history
    x_history_.push_back(pose_.x);
    y_history_.push_back(pose_.y);
}



void Robot::rotateWheels(double _left, double _right)
{
    if (!(abs(_left) < this->w_limit && abs(_right) < this->w_limit)) {
        double a = max(abs(_left),abs(_right));
        a = max(a/w_limit,1.0);
        _left /= a;
        _right /= a;
    }
    // to fill up after defining an initWheel method
    if (this->isWheelInit()) {

        double _vx,_vy,_theta;


        _theta = this->pose_.theta + (r*(_left - _right)/(2*b)) * this->dt_;

        _vx = cos(_theta) * (r*(_left + _right)/2);
        _vy = sin(_theta) * (r*(_left + _right)/2);
    
        Robot::moveXYT(_vx,_vy,(r*(_left - _right)/(2*b)));
    }
    else 
        throw "Initialize wheels before using them. By calling the appropriate constructor.\n Or calling initWheel()";
}


// move robot with linear and angular velocities
void Robot::moveVW(double _v, double _omega)
{
    double w_left,w_right;

    /*_theta = this->pose_.theta +_omega*this->dt_;

    _vx = cos(_theta) * _v;
    _vy = sin(_theta) * _v;
    
    Robot::moveXYT(_vx,_vy,_omega);*/
    w_left = (_v + this->b*_omega)/this->r;
    w_right = (_v - this->b*_omega)/this->r;
    rotateWheels(w_left,w_right);
}

// try to go to a given x-y position
void Robot::goTo(const Pose &_p)
{
    // error in robot frame
    Pose error = _p.transformInverse(pose_);

    // try to do a straight line with sensor constraints
    moveWithSensor(Twist(error.x, error.y, 0));
}

void Robot::moveWithSensor(Twist _twist)
{
    // to fill up, sensor measurement and twist checking
    for(auto &sensor: sensors_) {
        sensor->updateFromRobotPose(this->pose_);
        sensor->correctRobotTwist(_twist);
    }

    // uses X-Y motion (perfect but impossible in practice) 
    // _twist carries the desired motion to be followed
    //moveXYT(_twist.vx, _twist.vy,_twist.w);

    // to fill up, use V-W motion when defined

    double v,w;
    v = _twist.vx;
    w = 20*_twist.vy + _twist.w;
    
    //call rotate wheels after. And rotate wheels will call movXYT
    moveVW(v, w);
}

void Robot::printPosition()
{
    cout << "Current position: " << pose_.x << ", " << pose_.y << endl;
}