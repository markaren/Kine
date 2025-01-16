
#ifndef KINE_JOINT_HPP
#define KINE_JOINT_HPP

#include "kine/math/Matrix4.hpp"
#include "kine/math/Vector3.hpp"

#include "kine/KineComponent.hpp"
#include "kine/KineLimit.hpp"

namespace kine {

    class KineJoint: public KineComponent {

    public:
        Vector3 axis;
        KineLimit limit;

        KineJoint(const Vector3& axis, const KineLimit& limit)
            : axis(axis),
              limit(limit){};

        [[nodiscard]] float getJointValue() const {
            return value_;
        }

        void setJointValue(float value) {
            value_ = value;
            limit.clampWithinLimit(value_);
        }

        [[nodiscard]] Matrix4 getTransformation() const override {
            return getTransformation(value_);
        }

        [[nodiscard]] virtual Matrix4 getTransformation(float value) const = 0;

    private:
        float value_{};
    };

}// namespace kine


#endif//KINE_JOINT_HPP
