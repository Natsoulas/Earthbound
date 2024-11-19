# Earthbound
This repo will serve as a test bed for simulating interesting Near-Earth satellite and space station flight concepts from a guidance and controls perspective.

## Overview
At the moment, this simulation implements a basic satellite dynamics model in Low Earth Orbit (LEO), including:

- Full 6-DOF rigid body dynamics
- Quaternion-based attitude representation (inertial to body frame quaternion that evolves with inertial trajectory)
- Coordinate frame transformations between:
  - GCRS (Geocentric Celestial Reference System) - An inertial frame aligned with the Earth's equator, ideal for LEO satellite navigation
  - ITRS (International Terrestrial Reference System) - Earth-fixed frame, also known as ECEF
  - LLA (Latitude, Longitude, Altitude) coordinates

The GCRS frame serves as our primary inertial reference frame (equivalent to ECI for most applications), with its xy-plane aligned with the Earth's equator. This makes it particularly well-suited for LEO satellite applications since most orbital perturbations are symmetric about the equator.

## Dependencies
- C++17 or later
- CMake 3.10 or later
- Eigen3 (header-only linear algebra library)
- ERFA (Essential Routines for Fundamental Astronomy) - automatically built from source

## Visualization
The project includes a Python script for visualizing simulation results. To run the visualization:

### Python Dependencies
- Python 3.7+
- NumPy
- Pandas
- Matplotlib
- Cartopy (for geographic plotting)

You can install the required Python packages using pip:

## Building and Running

1. Clone the repository:

```bash
git clone https://github.com/Natsoulas/Earthbound.git
cd Earthbound
```

2. Create and enter build directory:

```bash
mkdir build && cd build
```

3. Configure and build the project:

```bash
cmake ..
make -j$(nproc)
```

4. Run the simulation:

```bash
./Earthbound
```

This will generate output files in the `output` directory containing the simulation results.

## Running Tests

The project uses Google Test for unit testing. To run the tests:

1. Build the tests (they are built automatically with the main project):

```bash
cmake .. -DBUILD_TESTING=ON
make -j$(nproc)
```

2. Run all tests:

```bash
ctest --output-on-failure
```

Or run tests directly for more detailed output:

```bash
./tests/unit_tests
```

Individual test suites can be run using Google Test filters:

```bash
./tests/unit_tests --gtest_filter=OrbitDynamics.*
```

## Visualization

To visualize the simulation results:

1. Install the required Python packages:

```bash
pip install numpy pandas matplotlib cartopy
```

2. Run the visualization script:

```bash
python viz/plotMissionDashboard.py
```

This will generate several plots:
- Orbital trajectory in 3D
- Ground track overlaid on a map
- Time history of key state variables
- Attitude quaternion evolution

The plots will be saved in the `plots` directory by default.

### Example Dashboard
![Mission Dashboard](docs/images/mission_dashboard.png)

## Configuration

Simulation parameters can be modified in the `include/config/simpleSat.hpp` file, including:
- Initial orbital elements
- Simulation duration and time step
- Spacecraft physical properties
- Output file settings
