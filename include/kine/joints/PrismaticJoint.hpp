
#ifndef THREEPP_PRISMATICJOINT_HPP
#define THREEPP_PRISMATICJOINT_HPP

#include "kine/joints/KineJoint.hpp"

namespace kine {

    class PrismaticJoint: public KineJoint {

    public:
        PrismaticJoint(const Vector3& axis, KineLimit limit): KineJoint(axis, limit) {}

        [[nodiscard]] Matrix4 getTransformation(float value) const override {
            return Matrix4().makeTranslation(tmp_.copy(axis_).multiplyScalar(value));
        }


    private:
        mutable Vector3 tmp_;
    };

}// namespace kine

#endif//THREEPP_PRISMATICJOINT_HPP
