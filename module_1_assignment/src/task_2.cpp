#include <iostream>
 
using namespace std;

class Robot{
private:
    double temperature;
    double distance;

public:
    Robot(double RoboTemp = 20.0, double RoboDist = 100.0) :  
    temperature(RoboTemp), distance(RoboDist){
        cout<<"Robot is initialized!!"<<endl;
    }

    void get_temp(){
        cout<<"Temperature of the robot is "<<temperature<<"°C"<<endl;
    }
    void get_dist(){
        cout<<"Distance to the robot is "<<distance<<"m"<<endl;
    }
};

int main(){
    Robot myrobot;
    myrobot.get_temp();
    myrobot.get_dist();

    Robot anotherRobot(30, 400);
    anotherRobot.get_temp();
    anotherRobot.get_dist();

    return 0;

}