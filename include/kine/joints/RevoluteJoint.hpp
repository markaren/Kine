
#ifndef THREEPP_REVOLUTEJOINT_HPP
#define THREEPP_REVOLUTEJOINT_HPP

#include "kine/joints/KineJoint.hpp"

#include "kine/math/MathUtils.hpp"

namespace kine {

    class RevoluteJoint: public KineJoint {

    public:
        RevoluteJoint(const Vector3& axis, KineLimit limit): KineJoint(axis, limit) {}

        [[nodiscard]] Matrix4 getTransformation(float value) const override {
            return Matrix4().makeRotationAxis(axis, value * DEG2RAD);
        }
    };

}// namespace kine

#endif//THREEPP_REVOLUTEJOINT_HPP
