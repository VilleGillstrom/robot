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

    struct laser_properties {
        double AngleIncrement;
        double EndAngle;
        double StartAngle;
        glm::dvec3 LaserOffset;
    };

    struct robot_localization {
        glm::quat orientation;
        glm::dvec3 position;
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
    laser_properties GetProperties() const { return robotProperties; };
    /** Read the last fetched localization by ReadSensors() */
    robot_localization GetLocalization() const { return laserLocalization; };


private:
    std::string host;
    int port;

    /** Fetch from host */
    laser_echos FechEchoes() const;
    laser_properties FetchProperties() const;
    robot_localization FetchLocalization() const;

    std::string GetPropertiesUrl() const;
    std::string GetEchoesUrl() const;
    std::string GetLocalizationUrl() const;
    std::string MakeFullUrl(const std::string &path) const;

    /** Stores the last feches when using Read*()*/
    laser_echos laserEchoes;
    laser_properties robotProperties;
    robot_localization laserLocalization;
};

