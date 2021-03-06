#include <vector>
#include <cpr/cpr.h>

#include "../include/RobotCommunicationMRDS.h"

#include <external/json/json.hpp>
#include <stdexcept>

using namespace nlohmann;

RobotCommunicationMRDS::RobotCommunicationMRDS(const std::string& host, std::string port) {
    this->host = host;
    this->port = port;
}


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

RobotCommunicationMRDS::laser_properties RobotCommunicationMRDS::FetchProperties()  const {
    cpr::Response r = cpr::Get(cpr::Url{GetPropertiesUrl()});
    if (r.status_code != cpr::status::HTTP_OK) {
        std::stringstream ss;
        ss << "Bad Properties request: " << r.status_code;
        std::cerr << ss.str();
        throw std::runtime_error(ss.str());
    }

    json j = json::parse(r.text);

    double x = j["Pose"]["Position"]["X"].get<double>();
    double y = j["Pose"]["Position"]["Y"].get<double>();
    double z = j["Pose"]["Position"]["Z"].get<double>();
    return {
            j["AngleIncrement"].get<double>(),
            j["EndAngle"].get<double>(),
            j["StartAngle"].get<double>(),
            {x,y,z}
    };
}

RobotCommunicationMRDS::robot_localization RobotCommunicationMRDS::FetchLocalization() const{
    cpr::Response r = cpr::Get(cpr::Url{GetLocalizationUrl()});
    if (r.status_code != cpr::status::HTTP_OK) {
        std::stringstream ss;
        ss << "Bad Localization request: " << r.status_code;
        throw std::runtime_error(ss.str());
    }

    json j = json::parse(r.text);
    glm::quat orientation;
    glm::dvec3 position;

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

json RobotCommunicationMRDS::FetchDifferentialDriveJson() {
    cpr::Response r = cpr::Get(cpr::Url{GetDifferentialDriveUrl()});
    if (r.status_code != cpr::status::HTTP_OK) {
        std::stringstream ss;
        ss << "Bad Localization request: " << r.status_code;
        throw std::runtime_error(ss.str());
    }
    return json::parse(r.text);
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

std::string RobotCommunicationMRDS::GetDifferentialDriveUrl() const {
    return MakeFullUrl("/lokarria/differentialdrive");
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

void RobotCommunicationMRDS::ReadRobot() {
    auto future_echoes = std::async([this]() { return FechEchoes(); });
    auto future_localization = std::async([this]() { return FetchLocalization(); });
    auto future_differential = std::async([this]() { return FetchDifferentialDriveJson(); });

    laserEchoes = future_echoes.get();
    laserLocalization = future_localization.get();
    differentialDriveJson = future_differential.get();
}

void RobotCommunicationMRDS::SetSpeed(float speed) {
    SetSpeedAndAngular(speed, differentialDriveJson["Command"]["TargetAngularSpeed"]);

}

void RobotCommunicationMRDS::SetSpeedAndAngular(float speed, float angular) {
    json j;
    j["TargetLinearSpeed"] = speed;
    j["TargetAngularSpeed"] = angular;
    const std::string jsonStr = j.dump();
    cpr::Response r = cpr::Post(cpr::Url{GetDifferentialDriveUrl()}, cpr::Header{{"Content-Type", "text/plain"}}, cpr::Body{jsonStr});
    if (!(r.status_code == cpr::status::HTTP_OK || r.status_code == cpr::status::HTTP_NO_CONTENT)) {
        throw std::runtime_error("SetSpeedAndAngular post failed");
    }
}

bool RobotCommunicationMRDS::SetURL(const std::string &url) {
    std::string cpy = url;


    //WHY IS SIMPLE STUFF LIKE SPLITTING STRING A HEADACHE !? /endrant
    std::vector<std::string> strings;
    std::stringstream f(url);
    std::string s;
    while (getline(f, s, ':')) {
        strings.push_back(s);
    }

    if(strings.size() !=3) {
        return false;
    }

    host = "http:";
    host.append(strings[1]);
    port = strings[2];
    std::cout << host << std::endl << port << std::endl;
    return true;
}

bool RobotCommunicationMRDS::TestUrlConnection() {
    cpr::Response r = cpr::Get(cpr::Url{MakeFullUrl("")}, cpr::Timeout{1000});
    return r.status_code == cpr::status::HTTP_OK;
}






