# Create test executable
add_executable(unit_tests
    attitudeController_test.cpp
    test_main.cpp
    frames_test.cpp
    orbital_mechanics_test.cpp
    gravity_test.cpp
    eom_test.cpp
    spacecraft_test.cpp
)

# Set the output directory
set_target_properties(unit_tests
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# Include directories
target_include_directories(unit_tests
    PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${ERFA_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/lib/eigen
    ${GTEST_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(unit_tests
    PRIVATE
    GTest::GTest
    GTest::Main
    Eigen3::Eigen
    erfa
)

# Add tests to CTest
add_test(NAME unit_tests 
         COMMAND ${CMAKE_BINARY_DIR}/bin/unit_tests)

# Optional: Add test labels for organization
set_tests_properties(unit_tests PROPERTIES LABELS "unit")
