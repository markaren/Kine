
#include "kine/math/Quaternion.hpp"

#include "kine/math/Euler.hpp"
#include "kine/math/Matrix4.hpp"
#include "kine/math/Vector3.hpp"

#include <algorithm>
#include <cmath>
#include <string>

using namespace kine;


Quaternion::Quaternion(float x, float y, float z, float w)
    : x(x), y(y), z(z), w(w) {}

float Quaternion::operator[](unsigned int index) const {
    switch (index) {
        case 0:
            return x;
        case 1:
            return y;
        case 2:
            return z;
        case 3:
            return w;
        default:
            throw std::runtime_error("index out of bound: " + std::to_string(index));
    }
}

Quaternion& Quaternion::set(float x, float y, float z, float w) {

    this->x = x;
    this->y = y;
    this->z = z;
    this->w = w;

    this->onChangeCallback_();

    return *this;
}

Quaternion& Quaternion::copy(const Quaternion& quaternion) {

    this->x = quaternion.x;
    this->y = quaternion.y;
    this->z = quaternion.z;
    this->w = quaternion.w;

    this->onChangeCallback_();

    return *this;
}

Quaternion& Quaternion::setFromEuler(const Euler& euler, bool update) {

    const auto x = euler.x, y = euler.y, z = euler.z;
    const auto order = euler.order_;

    // http://www.mathworks.com/matlabcentral/fileexchange/
    // 	20696-function-to-convert-between-dcm-euler-angles-quaternions-and-euler-vectors/
    //	content/SpinCalc.m

    const float c1 = std::cos(x / 2.f);
    const float c2 = std::cos(y / 2.f);
    const float c3 = std::cos(z / 2.f);

    const float s1 = std::sin(x / 2.f);
    const float s2 = std::sin(y / 2.f);
    const float s3 = std::sin(z / 2.f);

    switch (order) {

        case Euler::RotationOrders::XYZ:
            this->x = s1 * c2 * c3 + c1 * s2 * s3;
            this->y = c1 * s2 * c3 - s1 * c2 * s3;
            this->z = c1 * c2 * s3 + s1 * s2 * c3;
            this->w = c1 * c2 * c3 - s1 * s2 * s3;
            break;

        case Euler::RotationOrders::YXZ:
            this->x = s1 * c2 * c3 + c1 * s2 * s3;
            this->y = c1 * s2 * c3 - s1 * c2 * s3;
            this->z = c1 * c2 * s3 - s1 * s2 * c3;
            this->w = c1 * c2 * c3 + s1 * s2 * s3;
            break;

        case Euler::RotationOrders::ZXY:
            this->x = s1 * c2 * c3 - c1 * s2 * s3;
            this->y = c1 * s2 * c3 + s1 * c2 * s3;
            this->z = c1 * c2 * s3 + s1 * s2 * c3;
            this->w = c1 * c2 * c3 - s1 * s2 * s3;
            break;

        case Euler::RotationOrders::ZYX:
            this->x = s1 * c2 * c3 - c1 * s2 * s3;
            this->y = c1 * s2 * c3 + s1 * c2 * s3;
            this->z = c1 * c2 * s3 - s1 * s2 * c3;
            this->w = c1 * c2 * c3 + s1 * s2 * s3;
            break;

        case Euler::RotationOrders::YZX:
            this->x = s1 * c2 * c3 + c1 * s2 * s3;
            this->y = c1 * s2 * c3 + s1 * c2 * s3;
            this->z = c1 * c2 * s3 - s1 * s2 * c3;
            this->w = c1 * c2 * c3 - s1 * s2 * s3;
            break;

        case Euler::RotationOrders::XZY:
            this->x = s1 * c2 * c3 - c1 * s2 * s3;
            this->y = c1 * s2 * c3 - s1 * c2 * s3;
            this->z = c1 * c2 * s3 + s1 * s2 * c3;
            this->w = c1 * c2 * c3 + s1 * s2 * s3;
            break;
    }

    if (update) {
        this->onChangeCallback_();
    }

    return *this;
}

Quaternion& Quaternion::setFromAxisAngle(const Vector3& axis, float angle) {

    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.htm

    // assumes axis is normalized

    const float halfAngle = angle / 2.f, s = std::sin(halfAngle);

    this->x = axis.x * s;
    this->y = axis.y * s;
    this->z = axis.z * s;
    this->w = std::cos(halfAngle);

    this->onChangeCallback_();

    return *this;
}

Quaternion& Quaternion::setFromRotationMatrix(const Matrix4& m) {

    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm

    // assumes the upper 3x3 of m is a pure rotation matrix (i.e, unscaled)

    const auto& te = m.elements;

    const auto m11 = te[0], m12 = te[4], m13 = te[8],
               m21 = te[1], m22 = te[5], m23 = te[9],
               m31 = te[2], m32 = te[6], m33 = te[10],

               trace = m11 + m22 + m33;

    if (trace > 0) {

        const auto s = 0.5f / std::sqrt(trace + 1.0f);

        this->w = 0.25f / s;
        this->x = (m32 - m23) * s;
        this->y = (m13 - m31) * s;
        this->z = (m21 - m12) * s;

    } else if (m11 > m22 && m11 > m33) {

        const auto s = 2.0f * std::sqrt(1.0f + m11 - m22 - m33);

        this->w = (m32 - m23) / s;
        this->x = 0.25f * s;
        this->y = (m12 + m21) / s;
        this->z = (m13 + m31) / s;

    } else if (m22 > m33) {

        const auto s = 2.0f * std::sqrt(1.0f + m22 - m11 - m33);

        this->w = (m13 - m31) / s;
        this->x = (m12 + m21) / s;
        this->y = 0.25f * s;
        this->z = (m23 + m32) / s;

    } else {

        const auto s = 2.f * std::sqrt(1.0f + m33 - m11 - m22);

        this->w = (m21 - m12) / s;
        this->x = (m13 + m31) / s;
        this->y = (m23 + m32) / s;
        this->z = 0.25f * s;
    }

    this->onChangeCallback_();

    return *this;
}

Quaternion& Quaternion::setFromUnitVectors(const Vector3& vFrom, const Vector3& vTo) {
    // assumes direction vectors vFrom and vTo are normalized

    const auto EPS = 0.000001f;

    auto r = vFrom.dot(vTo) + 1;

    if (r < EPS) {

        // vFrom and vTo point in opposite directions

        r = 0;

        if (std::abs(vFrom.x) > std::abs(vFrom.z)) {

            this->x = -vFrom.y;
            this->y = vFrom.x;
            this->z = 0;
            this->w = r;

        } else {

            this->x = 0;
            this->y = -vFrom.z;
            this->z = vFrom.y;
            this->w = r;
        }

    } else {

        // crossVectors( vFrom, vTo ); // inlined to avoid cyclic dependency on Vector3

        this->x = vFrom.y * vTo.z - vFrom.z * vTo.y;
        this->y = vFrom.z * vTo.x - vFrom.x * vTo.z;
        this->z = vFrom.x * vTo.y - vFrom.y * vTo.x;
        this->w = r;
    }

    return this->normalize();
}


float Quaternion::angleTo(const Quaternion& q) const {

    return 2 * std::acos(std::abs(std::clamp(this->dot(q), -1.0f, 1.0f)));
}

Quaternion& Quaternion::rotateTowards(const Quaternion& q, float step) {

    const float angle = this->angleTo(q);

    if (angle == 0) return *this;

    auto t = std::min(1.f, step / angle);

    this->slerp(q, t);

    return *this;
}

Quaternion& Quaternion::slerp(const Quaternion& qb, float t) {

    if (t == 0) return *this;
    if (t == 1) return this->copy(qb);

    const float x = this->x, y = this->y, z = this->z, w = this->w;

    // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/

    float cosHalfTheta = w * qb.w + x * qb.x + y * qb.y + z * qb.z;

    if (cosHalfTheta < 0) {

        this->w = -qb.w;
        this->x = -qb.x;
        this->y = -qb.y;
        this->z = -qb.z;

        cosHalfTheta = -cosHalfTheta;

    } else {

        this->copy(qb);
    }

    if (cosHalfTheta >= 1.0) {

        this->w = w;
        this->x = x;
        this->y = y;
        this->z = z;

        return *this;
    }

    const float sqrSinHalfTheta = 1.f - cosHalfTheta * cosHalfTheta;

    if (sqrSinHalfTheta <= std::numeric_limits<float>::epsilon()) {

        const float s = 1 - t;
        this->w = s * w + t * this->w;
        this->x = s * x + t * this->x;
        this->y = s * y + t * this->y;
        this->z = s * z + t * this->z;

        this->normalize();
        this->onChangeCallback_();

        return *this;
    }

    const float sinHalfTheta = std::sqrt(sqrSinHalfTheta);
    const float halfTheta = std::atan2(sinHalfTheta, cosHalfTheta);
    const float ratioA = std::sin((1 - t) * halfTheta) / sinHalfTheta,
                ratioB = std::sin(t * halfTheta) / sinHalfTheta;

    this->w = (w * ratioA + this->w * ratioB);
    this->x = (x * ratioA + this->x * ratioB);
    this->y = (y * ratioA + this->y * ratioB);
    this->z = (z * ratioA + this->z * ratioB);

    this->onChangeCallback_();

    return *this;
}

void Quaternion::slerpQuaternions(const Quaternion& qa, const Quaternion& qb, float t) {

    copy(qa).slerp(qb, t);
}

Quaternion& Quaternion::identity() {

    return this->set(0, 0, 0, 1);
}

Quaternion& Quaternion::invert() {

    // Quaternion is assumed to have unit length

    return this->conjugate();
}

Quaternion& Quaternion::conjugate() {

    this->x *= -1;
    this->y *= -1;
    this->z *= -1;

    this->onChangeCallback_();

    return *this;
}

float Quaternion::dot(const Quaternion& v) const {

    return this->x * v.x + this->y * v.y + this->z * v.z + this->w * v.w;
}

float Quaternion::lengthSq() const {

    return this->x * this->x + this->y * this->y + this->z * this->z + this->w * this->w;
}

float Quaternion::length() const {

    return std::sqrt(this->x * this->x + this->y * this->y + this->z * this->z + this->w * this->w);
}

Quaternion& Quaternion::normalize() {

    auto l = length();

    if (l == 0) {

        this->x = 0;
        this->y = 0;
        this->z = 0;
        this->w = 1;

    } else {

        l = 1.0f / l;

        this->x = this->x * l;
        this->y = this->y * l;
        this->z = this->z * l;
        this->w = this->w * l;
    }

    this->onChangeCallback_();

    return *this;
}

Quaternion& Quaternion::multiply(const Quaternion& q) {

    return this->multiplyQuaternions(*this, q);
}

Quaternion& Quaternion::premultiply(const Quaternion& q) {

    return this->multiplyQuaternions(q, *this);
}

Quaternion& Quaternion::multiplyQuaternions(const Quaternion& a, const Quaternion& b) {

    // from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/code/index.htm

    const auto qax = a.x, qay = a.y, qaz = a.z, qaw = a.w;
    const auto qbx = b.x, qby = b.y, qbz = b.z, qbw = b.w;

    this->x = qax * qbw + qaw * qbx + qay * qbz - qaz * qby;
    this->y = qay * qbw + qaw * qby + qaz * qbx - qax * qbz;
    this->z = qaz * qbw + qaw * qbz + qax * qby - qay * qbx;
    this->w = qaw * qbw - qax * qbx - qay * qby - qaz * qbz;

    this->onChangeCallback_();

    return *this;
}

Quaternion Quaternion::clone() const {

    return Quaternion(x, y, z, w);
}

bool Quaternion::equals(const Quaternion& v) const {

    return ((v.x == this->x) && (v.y == this->y) && (v.z == this->z) && (v.w == this->w));
}

bool Quaternion::operator==(const Quaternion& other) const {

    return equals(other);
}

bool Quaternion::operator!=(const Quaternion& other) const {

    return !equals(other);
}
