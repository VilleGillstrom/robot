#include <vector>
#include <external/cpr/include/cpr/response.h>
#include <external/cpr/include/cpr/cpr.h>
#include <external/cpr/include/cpr/cprtypes.h>
#include "../include/RobotCommunicationMRDS.h"

#include <external/json/json.hpp>
#include <stdexcept>

using namespace nlohmann;


RobotCommunicationMRDS::laser_echos RobotCommunicationMRDS::FechEchoes()  const {
    cpr::Response r = cpr::Get(cpr::Url{GetEchoesUrl()});
    if (r.status_code != cpr::status::HTTP_OK) {
        std::stringstream ss;
        ss << "Bad Echoes request: " << r.status_code;
        std::cerr << ss.str();
        throw std::runtime_error(ss.str());
    }

    json j = json::parse(r.text);
    return  {
            j["Echoes"].get<std::vector<double>>(),
            j["Overflow"].get<std::vector<int>>(),
            j["Reflector"].get<std::vector<int>>(),
            j["Timestamp"].get<int>()
    };

}

RobotCommunicationMRDS::robot_properties RobotCommunicationMRDS::FetchProperties()  const {
    cpr::Response r = cpr::Get(cpr::Url{GetPropertiesUrl()});
    if (r.status_code != cpr::status::HTTP_OK) {
        std::stringstream ss;
        ss << "Bad Properties request: " << r.status_code;
        std::cerr << ss.str();
        throw std::runtime_error(ss.str());
    }

    json j = json::parse(r.text);
    return {
            j["AngleIncrement"].get<double>(),
            j["EndAngle"].get<double>(),
            j["StartAngle"].get<double>()
    };
}

RobotCommunicationMRDS::laser_localization RobotCommunicationMRDS::FetchLocalization() const{
    cpr::Response r = cpr::Get(cpr::Url{GetLocalizationUrl()});
    if (r.status_code != cpr::status::HTTP_OK) {
        std::stringstream ss;
        ss << "Bad Localization request: " << r.status_code;
        throw std::runtime_error(ss.str());
    }

    json j = json::parse(r.text);
    glm::quat orientation;
    glm::vec3 position;

    orientation.x = j["Pose"]["Orientation"]["X"].get<double>();
    orientation.y = j["Pose"]["Orientation"]["Y"].get<double>();
    orientation.z = j["Pose"]["Orientation"]["Z"].get<double>();
    orientation.w = j["Pose"]["Orientation"]["W"].get<double>();

    position.x = j["Pose"]["Position"]["X"].get<double>();
    position.y = j["Pose"]["Position"]["Y"].get<double>();
    position.z = j["Pose"]["Position"]["Z"].get<double>();

    int timestamp = j["Timestamp"].get<int>();

    return {orientation, position, timestamp};
}


RobotCommunicationMRDS::RobotCommunicationMRDS(const std::string& host, int port) {
    this->host = host;
    this->port = port;
}

std::string RobotCommunicationMRDS::GetPropertiesUrl() const {
    return MakeFullUrl("/lokarria/laser/properties");
}

std::string RobotCommunicationMRDS::GetEchoesUrl() const {
    return MakeFullUrl("/lokarria/laser/echoes");
}

std::string RobotCommunicationMRDS::GetLocalizationUrl() const {
    return MakeFullUrl("/lokarria/localization");
}

std::string RobotCommunicationMRDS::MakeFullUrl(const std::string &path) const {
    std::stringstream ss;
    ss << host << ":" << port << path;
    std::string str = ss.str();
    return str;
}

void RobotCommunicationMRDS::ReadProperties() {
    robotProperties = FetchProperties();
}

void RobotCommunicationMRDS::ReadSensors() {
    auto future_echoes = std::async([this]() { return FechEchoes(); });
    auto future_localization = std::async([this]() { return FetchLocalization(); });
    laserEchoes = future_echoes.get();
    laserLocalization = future_localization.get();
}






