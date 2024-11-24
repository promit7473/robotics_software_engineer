#include <iostream>
#include <string>

using namespace std;

// Namespace for different robot types
namespace RobotTypes {
    // Namespace for Exploration Robots
    namespace Exploration {
        class Robot {
        private:
            string name;
            double speed;
            double weight;
            double size;
            int number_of_sensors;

        public:
            Robot(string robotName, double robotSpeed) 
                : name(robotName), speed(robotSpeed), weight(75.5), size(1.2), number_of_sensors(6) {
                cout << "Exploration Robot " << name << " created" << endl;
                cout << "Speed: " << speed << " m/s" << endl;
            }

            void moveForward() {
                cout << name << " (Exploration Robot) is moving forward at " << speed << " m/s" << endl;
            }

            void moveBackward() {
                cout << name << " (Exploration Robot) is moving backward at " << speed << " m/s" << endl;
            }

            void stop() {
                cout << name << " (Exploration Robot) has stopped moving" << endl;
            }
        };
    }

    // Namespace for Industrial Robots
    namespace Industrial {
        class Robot {
        private:
            string name;
            double speed;
            double weight;
            double size;
            int number_of_sensors;

        public:
            Robot(string robotName, double robotSpeed) 
                : name(robotName), speed(robotSpeed), weight(500.0), size(2.5), number_of_sensors(8) {
                cout << "Industrial Robot " << name << " created" << endl;
                cout << "Speed: " << speed << " m/s" << endl;
            }

            void moveForward() {
                cout << name << " (Industrial Robot) is moving forward at " << speed << " m/s" << endl;
            }

            void moveBackward() {
                cout << name << " (Industrial Robot) is moving backward at " << speed << " m/s" << endl;
            }

            void stop() {
                cout << name << " (Industrial Robot) has stopped moving" << endl;
            }

            void performTask() {
                cout << name << " is performing an industrial task" << endl;
            }
        };
    }

    // Namespace for Medical Robots
    namespace Medical {
        class Robot {
        private:
            string name;
            double speed;
            double weight;
            double size;
            int number_of_sensors;

        public:
            Robot(string robotName, double robotSpeed) 
                : name(robotName), speed(robotSpeed), weight(45.0), size(0.8), number_of_sensors(10) {
                cout << "Medical Robot " << name << " created" << endl;
                cout << "Speed: " << speed << " m/s" << endl;
            }

            void moveForward() {
                cout << name << " (Medical Robot) is moving forward at " << speed << " m/s" << endl;
            }

            void moveBackward() {
                cout << name << " (Medical Robot) is moving backward at " << speed << " m/s" << endl;
            }

            void stop() {
                cout << name << " (Medical Robot) has stopped moving" << endl;
            }

            void performSurgery() {
                cout << name << " is performing a surgical procedure" << endl;
            }
        };
    }
}


int main() {
    
    // Exploration Robot
    RobotTypes::Exploration::Robot explorationBot("ExploreBot", 5.5);
    explorationBot.moveForward();
    explorationBot.moveBackward();
    explorationBot.stop();

    cout << "\n";

    // Industrial Robot
    RobotTypes::Industrial::Robot industrialBot("FactoryBot", 3.0);
    industrialBot.moveForward();
    industrialBot.performTask();
    industrialBot.stop();

    cout << "\n";

    // Medical Robot
    RobotTypes::Medical::Robot medicalBot("SurgeonBot", 1.5);
    medicalBot.moveForward();
    medicalBot.performSurgery();
    medicalBot.stop();

    return 0;
}