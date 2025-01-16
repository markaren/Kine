
#include "kine/math/MathUtils.hpp"


using namespace kine;


float kine::mapLinear(float x, float a1, float a2, float b1, float b2) {

    return b1 + (x - a1) * (b2 - b1) / (a2 - a1);
}

float kine::degToRad(float degrees) {

    return degrees * DEG2RAD;
}

float kine::radToDeg(float radians) {

    return radians * RAD2DEG;
}
