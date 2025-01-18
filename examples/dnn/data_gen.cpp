
#include "kine/Kine.hpp"

#include <fstream>
#include <iostream>
#include <random>

#include <filesystem>
#include <source_location>

std::vector<float> generateRandomVector(size_t len) {

    // Random number generator
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> dist(0.f, 1.0f);

    std::vector<float> result(len);
    for (size_t i = 0; i < len; ++i) {
        result[i] = dist(gen);
    }

    return result;
}

void generateTrainingData(const kine::Kine& kine, const std::filesystem::path& positionsFile, const std::filesystem::path& anglesFile, int numSamples) {
    std::ofstream posFile(positionsFile);
    std::ofstream angFile(anglesFile);

    if (!posFile.is_open() || !angFile.is_open()) {
        std::cerr << "Error opening output files!" << std::endl;
        return;
    }

    kine::Vector3 pos;
    for (int i = 0; i < numSamples; ++i) {
        // Generate random joint angles
        std::vector<float> normalizedAngles = generateRandomVector(kine.numDof());
        const auto denormalizedAngles = kine.denormalizeValues(normalizedAngles);
        angFile << denormalizedAngles[0] << "," << denormalizedAngles[1] << "," << denormalizedAngles[2] << "\n";

        // Compute end-effector position
        auto transformation = kine.calculateEndEffectorTransformation(denormalizedAngles, false);
        pos.setFromMatrixPosition(transformation);
        posFile << pos.x << "," << pos.y << "," << pos.z << "\n";
    }

    posFile.close();
    angFile.close();
    std::cout << "Training data generated successfully!" << std::endl;
}

int main() {

    // Crane3R
    const auto kine = kine::KineBuilder()
                              .addRevoluteJoint(kine::Vector3::Y(), {-90.f, 90.f})
                              .addLink(kine::Vector3::Y() * 4.2)
                              .addRevoluteJoint(kine::Vector3::X(), {-80.f, 0.f})
                              .addLink(kine::Vector3::Z() * 7)
                              .addRevoluteJoint(kine::Vector3::X(), {40.f, 140.f})
                              .addLink(kine::Vector3::Z() * 5.2)
                              .build();

    const auto loc = std::source_location::current();
    std::filesystem::path currentFolder = std::filesystem::path(loc.file_name()).parent_path();
    std::filesystem::path trainingFolder = currentFolder / "training/crane3r";
    create_directories(trainingFolder);

    generateTrainingData(kine, trainingFolder / "positions.csv", trainingFolder / "values.csv", 100000);
}
