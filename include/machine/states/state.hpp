#ifndef STATE_HPP
#define STATE_HPP

#include "../../spacecraft/simpleSat.hpp"

namespace machine {
namespace states {

class State {
public:
    virtual ~State() = default;
    
    // Pure virtual functions that must be implemented by concrete states
    virtual void enter() = 0;
    virtual void execute(spacecraft::SimpleSat& satellite) = 0;
    virtual void exit() = 0;
    
    // Virtual function for state name
    virtual const char* getName() const = 0;
};

} // namespace states
} // namespace machine

#endif // STATE_HPP 