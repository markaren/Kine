// https://github.com/mrdoob/three.js/blob/r129/src/math/Euler.js

#ifndef KINE_EULER_HPP
#define KINE_EULER_HPP

#include <optional>

namespace kine {

    class Vector3;
    class Matrix3;
    class Matrix4;
    class Quaternion;

    /**
     * A class representing Euler Angles.
     *
     * Euler angles describe a rotational transformation by rotating an object
     * on its various axes in specified amounts per axis, and a specified axis order.
     *
     * Iterating through a Euler instance will yield its components (x, y, z, order) in the corresponding order.
     */
    class Euler {

    public:
        enum RotationOrders {
            XYZ,
            YZX,
            ZXY,
            XZY,
            YXZ,
            ZYX
        };

        const static RotationOrders default_order = XYZ;

        float x;
        float y;
        float z;

        explicit Euler(float x = 0, float y = 0, float z = 0, RotationOrders order = default_order);

        [[nodiscard]] RotationOrders getOrder() const;

        void setOrder(RotationOrders value);

        Euler& set(float x, float y, float z, const std::optional<RotationOrders>& order = std::nullopt);

        Euler& copy(const Euler& e);

        Euler& setFromRotationMatrix(const Matrix4& m, std::optional<RotationOrders> order = std::nullopt);

        Euler& setFromQuaternion(const Quaternion& q, std::optional<RotationOrders> order = std::nullopt);

        Euler& setFromVector3(const Vector3& v, std::optional<RotationOrders> order = std::nullopt);

        [[nodiscard]] bool equals(const Euler& euler) const;

        template<class ArrayLike>
        Euler& fromArray(const ArrayLike& array, unsigned int offset = 0) {

            this->x = array[offset];
            this->y = array[offset + 1];
            this->z= array[offset + 2];

            return *this;
        }

        template<class ArrayLike>
        void toArray(ArrayLike& array, unsigned int offset = 0) const {

            array[offset] = this->x;
            array[offset + 1] = this->y;
            array[offset + 2] = this->z;
        }


    private:
        RotationOrders order_ = default_order;

        friend class Quaternion;
    };


}// namespace kine

#endif//KINE_EULER_HPP
