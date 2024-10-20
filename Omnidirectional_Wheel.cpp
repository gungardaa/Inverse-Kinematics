#include <iostream>
#include <fstream>
#include <cmath>
#include <array>
#include <limits>

const double PI = 3.141592653589;

class OmniRobot {
private:
    double robotRadius;
    std::array<double, 4> wheelAngles;
    std::array<std::array<double, 3>, 4> kinematicsMatrix;

public:
    OmniRobot(double rRadius, std::array<double, 4> wAngles) 
             : robotRadius(rRadius), wheelAngles(wAngles) {
        
        for(int i = 0; i < 4; i++) {
            kinematicsMatrix[i] = {cos(wheelAngles[i]), sin(wheelAngles[i]), -robotRadius};
        }
    }

    std::array<double, 4> inverseKinematics(double desiredVx, double desiredVy, double desiredOmega) {
        std::array<double, 4> wheelSpeeds = {0, 0, 0, 0};
        std::array<double, 3> desiredVelocities = {desiredVx, desiredVy, desiredOmega};
        
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 3; j++) {
                wheelSpeeds[i] += kinematicsMatrix[i][j] * desiredVelocities[j];
            }
        }
        return wheelSpeeds;
    }

    void printWheelSpeeds(std::array<double, 4>& speeds) {
        for(int i = 0; i < 4; i++) {
            std::cout << "Wheel " << i + 1 << "'s speed: " << speeds[i] << std::endl;
        }
    }

    double getRobotRadius() { return robotRadius; }
};

class FileHandler {
public:
    static void saveData(std::string filename, OmniRobot& robot, 
                         double vx, double vy, double omega, const std::array<double, 4>& speeds) {
        std::ofstream file(filename, std::ios::app);
        if(file.is_open()) {
            file << "Robot Radius: " << robot.getRobotRadius() << "\n";
            file << "Vx: " << vx << ", Vy: " << vy << ", Omega: " << omega << "\n";
            
            file << "Wheel Speeds: ";
            for(int i = 0; i < 4; i++) {
                file << speeds[i];
                if(i != 3) file << ", ";
                else file << ".";
            }
            
            file << "\n-----------------------\n";
            file.close();
            std::cout << "Data saved to " << filename << std::endl;
        } 
        else std::cout << "Error opening file for writing!" << std::endl;
    }
};

double getValidatedInput(std::string prompt) {
    double value;
    while (true) {
        std::cout << prompt;
        std::cin >> value;

        if (std::cin.good()) {
            return value;
        }
        
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Invalid input. Please enter a valid number.\n\n";
    }
}

int main() {
    double desiredVx, desiredVy, desiredOmega, robotRadius;
    desiredVx = getValidatedInput("Input the desired velocity along the x-axis: ");
    desiredVy = getValidatedInput("Input the desired velocity along the y-axis: ");
    desiredOmega = getValidatedInput("Input the desired rotational velocity (omega): ");
    robotRadius = getValidatedInput("\nInput the robot radius: ");

    OmniRobot robot(robotRadius, {PI/4, 3*PI/4, 5*PI/4, 7*PI/4});

    std::array<double, 4> wheelSpeeds = robot.inverseKinematics(desiredVx, desiredVy, desiredOmega);
    robot.printWheelSpeeds(wheelSpeeds);
    FileHandler::saveData("wheel_speeds.txt", robot, desiredVx, desiredVy, desiredOmega, wheelSpeeds);

    return 0;
}
