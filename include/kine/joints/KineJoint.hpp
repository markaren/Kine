
#ifndef KINE_JOINT_HPP
#define KINE_JOINT_HPP

#include "kine/math/Matrix4.hpp"
#include "kine/math/Vector3.hpp"

#include "kine/KineComponent.hpp"
#include "kine/KineLimit.hpp"

namespace kine {

    class KineJoint: public KineComponent {

    public:
        KineJoint(const Vector3& axis, const KineLimit& limit)
            : axis_(axis),
              limit_(limit){};

        [[nodiscard]] float getJointValue() const {
            return value_;
        }

        [[nodiscard]] const Vector3& axis() const { return axis_; }

        [[nodiscard]] const KineLimit& limit() const { return limit_; }

        void setJointValue(float value) {
            value_ = value;
            limit_.clampWithinLimit(value_);
        }

        [[nodiscard]] Matrix4 getTransformation() const override {
            return getTransformation(value_);
        }

        [[nodiscard]] virtual Matrix4 getTransformation(float value) const = 0;

    protected:
        Vector3 axis_;
        KineLimit limit_;

    private:
        float value_{};
    };

}// namespace kine


#endif//KINE_JOINT_HPP
