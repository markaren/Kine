
#ifndef KINE_DNNSOLVER_HPP
#define KINE_DNNSOLVER_HPP

#include "kine/ik/IKSolver.hpp"


#include <filesystem>
#include <memory>

namespace kine {

class DNNSolver : public IKSolver {

public:

    DNNSolver(const std::filesystem::path& model);

    std::vector<float> solveIK(const Kine& kine, const Vector3& target, const std::vector<float>& startValues) override;

    ~DNNSolver() override;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

}

#endif //KINE_DNNSOLVER_HPP
