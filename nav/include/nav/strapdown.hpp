#pragma once
#include "nav/earth.hpp"
namespace nav {
bool propagateStrapdown(NominalState &, const ImuSample &, double dt);
}
