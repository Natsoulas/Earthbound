/*
 * Copyright (c) 2024 Niko Natsoulas
 * 
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include <iostream>
#include <filesystem>
#include <chrono>
#include <thread>
#include "../include/spacecraft/simpleSat.hpp"
#include "../include/machine/states/state.hpp"
#include "../include/machine/states/safe.hpp"
#include "../include/machine/states/maneuver.hpp"
#include "../include/machine/transitions/transition.hpp"
#include "../include/machine/transitions/condition.hpp"

void printEarthAscii() {
    // Clear screen first
    std::cout << "\033[2J\033[H";
    
    const int FRAMES = 10;  // 10 seconds
    const std::chrono::milliseconds DELAY(100);
    
    const std::string title[] = {
        R"(
    ███████╗ █████╗ ██████╗ ████████╗██╗  ██╗██████╗  ██████╗ ██╗   ██╗███╗   ██╗██████╗ 
    ██╔════╝██╔══██╗██╔══██╗╚══██╔══╝██║  ██║██╔══██╗██╔═══██╗██║   ██║████╗  ██║██╔══██╗
    █████╗  ███████║██████╔╝   ██║   ███████║██████╔╝██║   ██║██║   ██║██╔██╗ ██║██║  ██║
    ██╔══╝  ██╔══██║██╔══██╗   ██║   ██╔══██║██╔══██╗██║   ██║██║   ██║██║╚██╗██║██║  ██║
    ███████╗██║  ██║██║  ██║   ██║   ██║  ██║██████╔╝╚██████╔╝╚██████╔╝██║ ╚████║██████╔╝
    ╚══════╝╚═╝  ╚═╝╚═╝  ╚═╝   ╚═╝   ╚═╝  ╚═╝╚═════╝  ╚═════╝  ╚═════╝ ╚═╝  ╚═══╝╚═════╝ )",
        R"(
    ▓▓▓▓▓▓▓╬▓▓▓▓▓╬▓▓▓▓▓▓╬▓▓▓▓▓▓▓▓╬▓▓╬  ▓▓╬▓▓▓▓▓▓╬  ▓▓▓▓▓▓╬▓▓╬   ▓▓╬▓▓▓╬   ▓▓╬▓▓▓▓▓▓╬
    ▓▓╬════╬▓▓╬══▓▓╬▓▓╬══▓▓╬╚══▓▓╬══╬▓▓╬  ▓▓╬▓▓╬══▓▓╬▓▓╬═══▓▓╬▓▓╬   ▓▓╬▓▓▓▓╬  ▓▓╬▓▓╬══▓▓╬
    ▓▓▓▓▓╬  ▓▓▓▓▓▓▓╬▓▓▓▓▓▓╬   ▓▓╬   ▓▓▓▓▓▓▓╬▓▓▓▓▓▓╬▓▓╬   ▓▓╬▓▓╬   ▓▓╬▓▓╬▓▓╬ ▓▓╬▓▓╬  ▓▓╬
    ▓▓╬══╬  ▓▓╬══▓▓╬▓▓╬══▓▓╬   ▓▓╬   ▓▓╬══▓▓╬▓▓╬══▓▓╬▓▓╬   ▓▓╬▓▓╬   ▓▓╬▓▓╬╚▓▓╬▓▓╬▓▓╬  ▓▓╬
    ▓▓▓▓▓▓▓╬▓▓╬  ▓▓╬▓▓╬  ▓▓╬   ▓▓╬   ▓▓╬  ▓▓╬▓▓▓▓▓▓╬╚▓▓▓▓▓▓╬╚▓▓▓▓▓▓╬▓▓╬ ╚▓▓▓▓╬▓▓▓▓▓▓╬
    ╚══════╬╚═╬  ╚═╬╚═╬  ╚═╬   ╚═╬   ╚═╬  ╚═╬╚═════╬ ╚═════╬ ╚═════╬╚═╬  ╚═══╬╚═════╬)",
        R"(
    ░░░░░░░│░░░░░│░░░░░░│░░░░░░░░│░░│  ░░│░░░░░░│  ░░░░░░│░░│   ░░│░░░│   ░░│░░░░░░│
    ░░│════│░░│══░░│░░│══░░│╚══░░│══│░░│  ░░│░░│══░░│░░│═══░░│░░│   ░░│░░░░│  ░░│░░│══░░│
    ░░░░░│  ░░░░░░░│░░░░░░│   ░░│   ░░░░░░░│░░░░░░│░░│   ░░│░░│   ░░│░░│░░│ ░░│░░│  ░░│
    ░░│══│  ░░│══░░│░░│══░░│   ░░│   ░░│══░░│░░│══░░│░░│   ░░│░░│   ░░│░░│╚░░│░░│░░│  ░░│
    ░░░░░░░│░░│  ░░│░░│  ░░│   ░░│   ░░│  ░░│░░░░░░│╚░░░░░░│╚░░░░░░│░░│ ╚░░░░│░░░░░░│
    ╚══════│╚═│  ╚═│╚═│  ╚═│   ╚═│   ╚═│  ╚═│╚═════│ ╚═════│ ╚═════│╚═│  ╚═══│╚═════│)"
    };

    // Subtitle
    const std::string subtitle = "\n        6-DOF Near-Earth Spacecraft Flight Simulator";
    const std::string credit = "\n\n              Brought to you by Niko Natsoulas\n";

    // Colors
    const std::string cyan = "\033[36m";
    const std::string bright_cyan = "\033[96m";
    const std::string blue = "\033[34m";
    const std::string gray = "\033[90m";
    const std::string reset = "\033[0m";

    // Animate for 10 seconds
    for (int i = 0; i < FRAMES; i++) {
        // Clear screen on first frame
        if (i == 0) {
            std::cout << "\033[2J\033[H";
        } else {
            // Move cursor up
            std::cout << "\033[7A";
        }

        // Alternate between colors and patterns
        std::string color = (i % 2 == 0) ? bright_cyan : cyan;
        std::cout << color << title[i % 3] 
                  << blue << subtitle 
                  << gray << credit 
                  << reset << std::flush;

        std::this_thread::sleep_for(DELAY);
    }
    
    std::cout << std::endl;
}



int main() {
    printEarthAscii();
    
    std::filesystem::create_directories("../data");
    
    spacecraft::SimpleSat satellite;
    
    // Create states
    auto safeState = std::make_unique<machine::states::SafeState>();
    auto raiseApogeeState = std::make_unique<machine::states::ManeuverState>(
        machine::states::ManeuverState::ManeuverType::RAISE_APOGEE, 800.0);
    auto raisePerigeeState = std::make_unique<machine::states::ManeuverState>(
        machine::states::ManeuverState::ManeuverType::RAISE_PERIGEE, 800.0);
    
    // Create transitions
    auto toRaiseApogee = std::make_unique<machine::transitions::ConditionTransition>(
        safeState.get(), raiseApogeeState.get(),
        [](const spacecraft::SimpleSat& sat) { return sat.getCurrentTime() >= 5400.0; }
    );
    
    auto toRaisePerigee = std::make_unique<machine::transitions::ConditionTransition>(
        raiseApogeeState.get(), raisePerigeeState.get(),
        [](const spacecraft::SimpleSat& sat) { 
            return std::abs(sat.getApogeeAltitude() - 800.0) < 1.0;
        }
    );
    
    // Current state
    machine::states::State* currentState = safeState.get();
    currentState->enter();
    
    // Run simulation with state machine
    satellite.runSimulation("../data/satellite_state.csv", [&](double t) {
        // Check transitions
        for (const auto& transition : {toRaiseApogee.get(), toRaisePerigee.get()}) {
            if (transition->getSourceState() == currentState && 
                transition->shouldTransition(satellite)) {
                currentState->exit();
                currentState = transition->getTargetState();
                currentState->enter();
                break;
            }
        }
        
        // Execute current state
        currentState->execute(satellite);
    });
    
    return 0;
}
