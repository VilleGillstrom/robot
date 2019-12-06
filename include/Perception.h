#pragma once

#include <memory>
#include "RobotCommunicationMRDS.h"
#include <glm/glm.hpp>

class Perception {

public:
    Perception(const std::shared_ptr<RobotCommunicationMRDS>& Communicator ) : communicator(Communicator){

    }

    void ReadSensors() {
        communicator->ReadSensors();
        echoes = communicator->GetEchoes();
        localization = communicator->GetLocalization();
        std::cout << "echoes stamp: " << echoes.timestamp  << ", loc stamp: " << localization.timestamp << std::endl;
    }

    glm::vec3 GetPosition() const {
        return localization.position;
    }

    glm::quat GetOrientation() const {
        return localization.orientation;
    }

    const std::vector<double>& GetEchoDistances() const {
        return echoes.distance;
    }
private:
    std::shared_ptr<RobotCommunicationMRDS> communicator;

    /* Last Echoes*/
    RobotCommunicationMRDS::laser_echos echoes;
    /* Last localization */
    RobotCommunicationMRDS::laser_localization localization;
};
