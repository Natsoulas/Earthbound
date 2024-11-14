#ifndef SAFE_STATE_HPP
#define SAFE_STATE_HPP

#include "state.hpp"
#include <iostream>

namespace machine {
namespace states {

class SafeState : public State {
public:
    void enter() override {
        std::cout << "Entering Safe mode" << std::endl;
    }
    
    void execute(spacecraft::SimpleSat& satellite) override {
        // Minimal operations, ensure power positive attitude
    }
    
    void exit() override {
        std::cout << "Exiting Safe mode" << std::endl;
    }
    
    const char* getName() const override { return "Safe"; }
};

} // namespace states
} // namespace machine

#endif // SAFE_STATE_HPP
