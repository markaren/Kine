
#ifndef KINE_LINK_HPP
#define KINE_LINK_HPP

#include "kine/KineComponent.hpp"

namespace kine {

    class KineLink: public KineComponent {

    public:
        KineLink(const Vector3& link)
            : transformation_(Matrix4().setPosition(link)) {}

        [[nodiscard]] Matrix4 getTransformation() const override {
            return transformation_;
        }

    private:
        Matrix4 transformation_;
    };

}// namespace kine

#endif//KINE_LINK_HPP
