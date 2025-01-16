
#ifndef KINE_MATHUTILS_HPP
#define KINE_MATHUTILS_HPP

#include <numbers>

namespace kine {

    constexpr float PI = std::numbers::pi_v<float>;

    constexpr float DEG2RAD = PI / 180.f;
    constexpr float RAD2DEG = 180.f / PI;

    // Linear mapping from range <a1, a2> to range <b1, b2>
    float mapLinear(float x, float a1, float a2, float b1, float b2);

    // Converts degrees to radians.
    float degToRad(float degrees);

    // Converts radians to degrees.
    float radToDeg(float radians);

}

#endif //KINE_MATHUTILS_HPP
