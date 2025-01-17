
#ifndef KINE_KINE_HPP
#define KINE_KINE_HPP

#include <vector>

#include "KineComponent.hpp"
#include "KineLink.hpp"

#include "joints/KineJoint.hpp"
#include "joints/PrismaticJoint.hpp"
#include "joints/RevoluteJoint.hpp"

namespace kine {

    class Kine {

    public:
        explicit Kine(std::vector<std::unique_ptr<KineComponent>> components)
            : components_(std::move(components)) {

            for (const auto& c : components_) {

                if (const auto joint = dynamic_cast<KineJoint*>(c.get())) {
                    joints_.emplace_back(joint);
                }
            }
        }

        [[nodiscard]] size_t numDof() const {

            return joints_.size();
        }

        [[nodiscard]] Matrix4 calculateEndEffectorTransformation(const std::vector<float>& values) const {

            Matrix4 result;
            for (unsigned i = 0, j = 0; i < components_.size(); ++i) {
                const auto& c = components_[i];
                if (const auto joint = dynamic_cast<KineJoint*>(c.get())) {
                    result.multiply(joint->getTransformation(values.at(j++)));
                } else {
                    result.multiply(c->getTransformation());
                }
            }
            return result;
        }

        [[nodiscard]] const std::vector<KineJoint*>& joints() const {
            return joints_;
        }

        [[nodiscard]] std::vector<KineLimit> limits() const {
            std::vector<KineLimit> limits;
            for (unsigned i = 0; i < numDof(); i++) {
                limits.emplace_back(joints_[i]->limit());
            }
            return limits;
        }

        [[nodiscard]] std::vector<float> meanAngles() const {
            const auto lim = limits();
            std::vector<float> res(numDof());
            for (unsigned i = 0; i < numDof(); ++i) {
                res[i] = lim[i].mean();
            }
            return res;
        }

        [[nodiscard]] std::vector<float> normalizeValues(const std::vector<float>& values) const {
            std::vector<float> res(numDof());
            for (unsigned i = 0; i < numDof(); ++i) {
                res[i] = joints_[i]->limit().normalize(values[i]);
            }
            return res;
        }

        [[nodiscard]] std::vector<float> denormalizeValues(const std::vector<float>& values) const {
            std::vector<float> res(numDof());
            for (unsigned i = 0; i < numDof(); ++i) {
                res[i] = joints_[i]->limit().denormalize(values[i]);
            }
            return res;
        }

        // [[nodiscard]] std::vector<std::vector<float>> computeJacobian(const std::vector<float>& values) const {
        //
        //     constexpr float h = 0.0001;// some low value
        //
        //     std::vector<std::vector<float>> jacobian(3);
        //     for (auto & row : jacobian)
        //     {
        //         row = std::vector<float>(numDof());
        //     }
        //     auto d1 = calculateEndEffectorTransformation(values);
        //
        //     for (int i = 0; i < 3; ++i) {
        //         auto vals = values;// copy
        //         vals[i] += h;
        //         auto d2 = calculateEndEffectorTransformation(vals);
        //
        //         jacobian[0][i] = (d2[12] - d1[12]) / h;
        //         jacobian[1][i] = (d2[13] - d1[13]) / h;
        //         jacobian[2][i] = (d2[14] - d1[14]) / h;
        //     }
        //
        //     return jacobian;
        // }

    private:
        std::vector<KineJoint*> joints_;
        std::vector<std::unique_ptr<KineComponent>> components_;
    };

    class KineBuilder {

    public:
        KineBuilder& addRevoluteJoint(const Vector3& axis, const KineLimit& limit) {
            components_.emplace_back(std::make_unique<RevoluteJoint>(axis, limit));

            return *this;
        }

        KineBuilder& addPrismaticJoint(const Vector3& axis, const KineLimit& limit) {
            components_.emplace_back(std::make_unique<PrismaticJoint>(axis, limit));

            return *this;
        }

        KineBuilder& addLink(const Vector3& axis) {
            components_.emplace_back(std::make_unique<KineLink>(axis));

            return *this;
        }

        Kine build() {

            return Kine{std::move(components_)};
        }

    private:
        std::vector<std::unique_ptr<KineComponent>> components_;
    };

}// namespace kine

#endif//KINE_KINE_HPP
