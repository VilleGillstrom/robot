#pragma once

#include <curl/curl.h>
#include <iostream>
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include <future>
#include <sstream>
#include <iterator>
#include <external/glm/gtc/quaternion.hpp>
#include <external/json/json.hpp>

/**
 * Interface to MRDS
 */

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
    RobotCommunicationMRDS(const std::string &host, std::string port);

    /** Read the properties of the robot (through host) */
    void ReadProperties();
    /** Read the laser sensors (through host) */
    void ReadRobot();


    /** Read the last fetched echoes by ReadSensors() */
    laser_echos GetEchoes() const { return laserEchoes; };
    /** Read the last fetched properties by ReadSensors() */
    laser_properties GetProperties() const { return robotProperties; };
    /** Read the last fetched localization by ReadSensors() */
    robot_localization GetLocalization() const { return laserLocalization; };


    /** Set speed */
    void SetSpeed(float speed);
    /** Set speed and angular targets*/
    void SetSpeedAndAngular(float speed, float angular);

    bool SetURL(const std::string& url);

private:
    std::string host;
    std::string port;

    /** Fetch from host */
    laser_echos FechEchoes() const;
    laser_properties FetchProperties() const;
    robot_localization FetchLocalization() const;
    nlohmann::json FetchDifferentialDriveJson();

    std::string GetPropertiesUrl() const;
    std::string GetEchoesUrl() const;
    std::string GetLocalizationUrl() const;
    std::string GetDifferentialDriveUrl() const;

    std::string MakeFullUrl(const std::string &path) const;

    /** Stores the last feches when using Read*()*/
    laser_echos laserEchoes;
    laser_properties robotProperties;
    robot_localization laserLocalization;


    nlohmann::json differentialDriveJson;

};

