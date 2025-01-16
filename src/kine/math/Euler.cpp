
#include "kine/math/Euler.hpp"

#include "kine/math/Matrix4.hpp"
#include "kine/math/Quaternion.hpp"
#include "kine/math/Vector3.hpp"

#include <algorithm>
#include <cmath>

using namespace kine;

Euler::Euler(float x, float y, float z, RotationOrders order)
    : x(x), y(y), z(z), order_(order) {}


Euler::RotationOrders Euler::getOrder() const {

    return order_;
}
void Euler::setOrder(RotationOrders value) {

    this->order_ = value;
}

Euler& Euler::set(float x, float y, float z, const std::optional<RotationOrders>& order) {

    this->x = x;
    this->y = y;
    this->z = z;
    this->order_ = order.value_or(this->order_);
    
    return *this;
}

Euler& Euler::copy(const Euler& euler) {
    this->x = euler.x;
    this->y = euler.y;
    this->z = euler.z;
    this->order_ = euler.order_;

    return *this;
}

Euler& Euler::setFromRotationMatrix(const Matrix4& m, std::optional<RotationOrders> order) {

    // assumes the upper 3x3 of m is a pure rotation matrix (i.e, unscaled)

    const auto& te = m.elements;
    const auto m11 = te[0], m12 = te[4], m13 = te[8];
    const auto m21 = te[1], m22 = te[5], m23 = te[9];
    const auto m31 = te[2], m32 = te[6], m33 = te[10];

    const float EPS = 0.9999999f;

    switch (order.value_or(this->order_)) {

        case XYZ:

            this->y = std::asin(std::clamp(m13, -1.0f, 1.0f));

            if (std::abs(m13) < EPS) {

                this->x = std::atan2(-m23, m33);
                this->z = std::atan2(-m12, m11);

            } else {

                this->x = std::atan2(m32, m22);
                this->z = 0;
            }

            break;

        case YXZ:

            this->x = std::asin(-std::clamp(m23, -1.0f, 1.0f));

            if (std::abs(m23) < EPS) {

                this->y = std::atan2(m13, m33);
                this->z = std::atan2(m21, m22);

            } else {

                this->y = std::atan2(-m31, m11);
                this->z = 0;
            }

            break;

        case ZXY:

            this->x = std::asin(std::clamp(m32, -1.0f, 1.0f));

            if (std::abs(m32) < EPS) {

                this->y = std::atan2(-m31, m33);
                this->z = std::atan2(-m12, m22);

            } else {

                this->y = 0;
                this->z = std::atan2(m21, m11);
            }

            break;

        case ZYX:

            this->y = std::asin(-std::clamp(m31, -1.0f, 1.0f));

            if (std::abs(m31) < EPS) {

                this->x = std::atan2(m32, m33);
                this->z = std::atan2(m21, m11);

            } else {

                this->x = 0;
                this->z = std::atan2(-m12, m22);
            }

            break;

        case YZX:

            this->z = std::asin(std::clamp(m21, -1.0f, 1.0f));

            if (std::abs(m21) < EPS) {

                this->x = std::atan2(-m23, m22);
                this->y = std::atan2(-m31, m11);

            } else {

                this->x = 0;
                this->y = std::atan2(m13, m33);
            }

            break;

        case XZY:

            this->z = std::asin(-std::clamp(m12, -1.0f, 1.0f));

            if (std::abs(m12) < EPS) {

                this->x = std::atan2(m32, m22);
                this->y = std::atan2(m13, m11);

            } else {

                this->x = std::atan2(-m23, m33);
                this->y = 0;
            }

            break;
    }

    return *this;
}

Euler& Euler::setFromQuaternion(const Quaternion& q, std::optional<RotationOrders> order) {

    Matrix4 _matrix{};
    _matrix.makeRotationFromQuaternion(q);

    return this->setFromRotationMatrix(_matrix, order);
}

Euler& Euler::setFromVector3(const Vector3& v, std::optional<RotationOrders> order) {

    return this->set(v.x, v.y, v.z, order);
}

bool Euler::equals(const Euler& euler) const {

    return (euler.x == this->x) && (euler.y == this->y) && (euler.z == this->z) && (euler.order_ == this->order_);
}
