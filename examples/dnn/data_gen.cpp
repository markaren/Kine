
#include "kine/Kine.hpp"

#include <fstream>
#include <iostream>
#include <random>

#include <filesystem>
#include <source_location>

std::vector<float> generateRandomVector(size_t len) {
    std::vector<float> result(len);

    // Random number generator
    static std::random_device rd;                                               // Seed generator
    static std::mt19937 gen(rd());                                           // Mersenne Twister RNG
    std::uniform_real_distribution<float> dist(0.f, 1.0f);   // Range [0, 1]

    // Fill the vector with random values
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

    kine::Kine kine = kine::KineBuilder()
                              .addRevoluteJoint(kine::Vector3::Y(), {-90.f, 90.f})
                              .addLink(kine::Vector3::Y() * 4.2)
                              .addRevoluteJoint(kine::Vector3::X(), {-80.f, 0.f})
                              .addLink(kine::Vector3::Z() * 7)
                              .addRevoluteJoint(kine::Vector3::X(), {40.f, 140.f})
                              .addLink(kine::Vector3::Z() * 5.2)
                              .build();

    const auto loc = std::source_location::current();
    std::filesystem::path currentFolder = std::filesystem::path(loc.file_name()).parent_path();
    std::filesystem::path trainingFolder = currentFolder / "training";
    create_directories(trainingFolder);

    generateTrainingData(kine, trainingFolder / "positions.csv", trainingFolder / "angles.csv", 10000);
}
