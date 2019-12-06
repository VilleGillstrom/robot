
#include "include/MapVisualizer.h"

unsigned int MapVisualizer::GetHeight() const {
    return Robot->GetCartographer().GetProbablityGrid().at(0).size();
}

unsigned int MapVisualizer::GetWidth() const {
    return Robot->GetCartographer().GetProbablityGrid().size();
}
