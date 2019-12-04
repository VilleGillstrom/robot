#pragma once

#include <curl/curl.h>
#include <iostream>

class RobotCommunicationMRDS {

public:
    RobotCommunicationMRDS() = default;
    RobotCommunicationMRDS(std::string host, std::string port) {
        this->host = host;
        this->port = port;
    }



    int GetRequest() const;


private:
    std::string host;
    std::string port;




};

