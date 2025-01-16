
#ifndef KINE_KINECOMPONENT_HPP
#define KINE_KINECOMPONENT_HPP

#include "kine/math/Matrix4.hpp"

namespace kine {

    class KineComponent {

    public:
        [[nodiscard]] virtual Matrix4 getTransformation() const = 0;

        virtual ~KineComponent() = default;
    };

}// namespace kine

#endif//KINE_KINECOMPONENT_HPP
