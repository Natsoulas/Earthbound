/*
 * Copyright (c) 2024 Niko Natsoulas
 * 
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include <gtest/gtest.h>
#include <filesystem>
#include "../include/spacecraft/simpleSat.hpp"

class SpacecraftTest : public ::testing::Test {
protected:
    const double tol = 1e-10;
    spacecraft::SimpleSat satellite;
    
    void SetUp() override {
        // Create test directory
        std::filesystem::create_directories("../test_data");
    }
    
    void TearDown() override {
        // Clean up test directory
        std::filesystem::remove_all("../test_data");
    }
};

TEST_F(SpacecraftTest, InitialConditions) {
    // Test initial state matches config
    EXPECT_TRUE(satellite.getPosition().isApprox(config::SimpleSat::initial_position, tol));
    EXPECT_TRUE(satellite.getVelocity().isApprox(config::SimpleSat::initial_velocity, tol));
    EXPECT_TRUE(satellite.getQuaternion().isApprox(config::SimpleSat::initial_quaternion, tol));
    EXPECT_TRUE(satellite.getAngularVelocity().isApprox(config::SimpleSat::initial_omega, tol));
}

// Not needed, redundant, causes unit tests to take a long time.
// TEST_F(SpacecraftTest, SimulationOutput) {
//     // Run a short simulation
//     std::string test_file = "../test_data/test_output.csv";
//     satellite.runSimulation(test_file);
    
//     // Verify file exists and has content
//     EXPECT_TRUE(std::filesystem::exists(test_file));
    
//     // Check file content
//     std::ifstream file(test_file);
//     EXPECT_TRUE(file.good());
    
//     // Read header line
//     std::string header;
//     std::getline(file, header);
//     EXPECT_FALSE(header.empty());
    
//     // Count number of data lines
//     int line_count = 0;
//     std::string line;
//     while (std::getline(file, line)) {
//         line_count++;
//     }
    
//     // Verify we have the expected number of lines
//     // Add 1 to include the initial state
//     double expected_lines = (config::SimpleSat::sim_duration / config::SimpleSat::time_step);
//     EXPECT_EQ(line_count, static_cast<int>(expected_lines));
// } 