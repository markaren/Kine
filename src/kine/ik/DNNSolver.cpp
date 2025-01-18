
#include "kine/ik/DNNSolver.hpp"

#include <onnxruntime_cxx_api.h>


using namespace kine;


struct DNNSolver::Impl {

    Impl(const std::filesystem::path& model)
        : env_(ORT_LOGGING_LEVEL_WARNING, "ONNX IK"),
          input_data_(3),
          model_(model) {}

    std::vector<float> solveIK(const Kine& kine, const Vector3& target, const std::vector<float>&) {

        if (!session_) {
            Ort::SessionOptions session_options;
            session_options.SetIntraOpNumThreads(1);
            session_options.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);
            session_ = std::make_unique<Ort::Session>(env_, model_.c_str(), session_options);
        }

        const char* input_names[] = {"input"};
        const char* output_names[] = {"output"};

        input_data_[0] = target.x;
        input_data_[1] = target.y;
        input_data_[2] = target.z;

        // Input tensor
        std::vector<int64_t> input_dims = {1, 3};// Batch size 1, 3 features
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
                Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault),
                input_data_.data(), input_data_.size(), input_dims.data(), input_dims.size());

        // Run inference
        auto output_tensors = session_->Run(
                Ort::RunOptions{nullptr},// Run options
                input_names,             // Input names
                &input_tensor,           // Input tensors
                1,                       // Input count
                output_names,            // Output names
                1                        // Output count
        );

        // Process output
        const auto* output_data = output_tensors[0].GetTensorMutableData<float>();

        // Convert the data to a vector
        std::vector<float> output_vector(kine.numDof());
        for (size_t i = 0; i < kine.numDof(); i++) {
            output_vector[i] = output_data[i];
        }

        return output_vector;
    }

private:
    Ort::Env env_;
    std::vector<float> input_data_;
    std::unique_ptr<Ort::Session> session_;

    std::filesystem::path model_;
};

DNNSolver::DNNSolver(const std::filesystem::path& model)
    : pimpl_(std::make_unique<Impl>(model)) {
}

std::vector<float> DNNSolver::solveIK(const Kine& kine, const Vector3& target, const std::vector<float>& startValues) {

    return pimpl_->solveIK(kine, target, startValues);
}

DNNSolver::~DNNSolver() = default;
