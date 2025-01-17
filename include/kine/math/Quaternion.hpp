// https://github.com/mrdoob/three.js/blob/r129/src/math/Quaternion.js

#ifndef KINE_QUATERNION_HPP
#define KINE_QUATERNION_HPP

#include <functional>
#include <ostream>

namespace kine {

    class Vector3;
    class Matrix4;
    class Euler;

    class Quaternion {

    public:
        float x;
        float y;
        float z;
        float w;

        explicit Quaternion(float x = 0, float y = 0, float z = 0, float w = 1);

        float operator[](unsigned int index) const;

        Quaternion& set(float x, float y, float z, float w);

        Quaternion& copy(const Quaternion& quaternion);

        Quaternion& setFromEuler(const Euler& euler, bool update = true);

        Quaternion& setFromAxisAngle(const Vector3& axis, float angle);

        Quaternion& setFromRotationMatrix(const Matrix4& m);

        Quaternion& setFromUnitVectors(const Vector3& vFrom, const Vector3& vTo);

        [[nodiscard]] float angleTo(const Quaternion& q) const;

        Quaternion& rotateTowards(const Quaternion& q, float step);

        Quaternion& identity();

        Quaternion& invert();

        Quaternion& conjugate();

        [[nodiscard]] float dot(const Quaternion& v) const;

        [[nodiscard]] float lengthSq() const;

        [[nodiscard]] float length() const;

        Quaternion& normalize();

        Quaternion& multiply(const Quaternion& q);

        Quaternion& premultiply(const Quaternion& q);

        Quaternion& multiplyQuaternions(const Quaternion& a, const Quaternion& b);

        Quaternion& slerp(const Quaternion& qb, float t);

        void slerpQuaternions(const Quaternion& qa, const Quaternion& qb, float t);

        [[nodiscard]] Quaternion clone() const;

        [[nodiscard]] bool equals(const Quaternion& v) const;

        bool operator==(const Quaternion& other) const;

        bool operator!=(const Quaternion& other) const;

        Quaternion& _onChange(std::function<void()> callback);

        template<class ArrayLike>
        Quaternion& fromArray(const ArrayLike& array, unsigned int offset = 0) {

            this->x = array[offset];
            this->y = array[offset + 1];
            this->z = array[offset + 2];
            this->w = array[offset + 3];

            this->onChangeCallback_();

            return *this;
        }

        template<class ArrayLike>
        void toArray(ArrayLike& array, unsigned int offset = 0) const {

            array[offset] = this->x;
            array[offset + 1] = this->y;
            array[offset + 2] = this->z;
            array[offset + 3] = this->w;
        }

        friend std::ostream& operator<<(std::ostream& os, const Quaternion& v) {
            os << "Quaternion(x=" << v.x << ", y=" << v.y << ", z=" << v.z << ", w=" << v.w << ")";
            return os;
        }

    private:
        std::function<void()> onChangeCallback_ = [] {};
    };

}// namespace threepp

#endif//KINE_QUATERNION_HPP
