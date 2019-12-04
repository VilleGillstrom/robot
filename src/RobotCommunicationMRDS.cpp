#include "../include/RobotCommunicationMRDS.h"

using namespace nlohmann;


struct robot_echos {
    std::vector<double> values;
    int timestamp;
};

int RobotCommunicationMRDS::GetRequest() const {
//    cpr::Response r;
//    r = cpr::Get(cpr::Url{"http://localhost:50004/lokarria/laser/echoes"});
//    //r.status_code;                  // 200
//    //r.header["content-type"];       // application/json; charset=utf-8
//    //r.text;                         // JSON text string
//
//    if(r.status_code == cpr::status::HTTP_OK) {
//        std::cout << r.text;
//
//
//        nlohmann::json j;
//        j = json::parse(r.text);
//
//        robot_echos e {
//            j["Echoes"].get<std::vector<double>>(),
//            j["Timestamp"].get<int>()
//        };
//
//        std::cout << e.values[0];
//        std::cout << std::endl;
//
//
//    } else {
//        std::cerr << "Bad request!\n";
//    }
    return 0;

}
