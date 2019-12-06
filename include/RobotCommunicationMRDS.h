#pragma once

#include <curl/curl.h>
#include <iostream>
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include <future>
#include <external/glm/gtc/quaternion.hpp>

class RobotCommunicationMRDS {
public:
    struct laser_echos {
        std::vector<double> distance;
        std::vector<int> overflow;
        std::vector<int> reflector;
        int timestamp;
    };

    struct robot_properties {
        double AngleIncrement;
        double EndAngle;
        double StartAngle;
    };

    struct laser_localization {
        glm::quat orientation;
        glm::vec3 position;
        int timestamp;
    };

    RobotCommunicationMRDS() = default;
    RobotCommunicationMRDS(const std::string &host, int port);

    /** Read the properties of the robot (through host) */
    void ReadProperties();
    /** Read the laser sensors (through host) */
    void ReadSensors();


    /** Read the last fetched echoes by ReadSensors() */
    laser_echos GetEchoes() const { return laserEchoes; };
    /** Read the last fetched properties by ReadSensors() */
    robot_properties GetProperties() const { return robotProperties; };
    /** Read the last fetched localization by ReadSensors() */
    laser_localization GetLocalization() const { return laserLocalization; };


private:
    std::string host;
    int port;

    /** Fetch from host */
    laser_echos FechEchoes() const;
    robot_properties FetchProperties() const;
    laser_localization FetchLocalization() const;

    std::string GetPropertiesUrl() const;
    std::string GetEchoesUrl() const;
    std::string GetLocalizationUrl() const;
    std::string MakeFullUrl(const std::string &path) const;

    /** Stores the last feches when using Read*()*/
    laser_echos laserEchoes;
    robot_properties robotProperties;
    laser_localization laserLocalization;
};

