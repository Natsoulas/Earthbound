#ifndef TRANSITION_HPP
#define TRANSITION_HPP

#include "../states/state.hpp"
#include "../../spacecraft/simpleSat.hpp"

namespace machine {
namespace transitions {

class Transition {
public:
    virtual ~Transition() = default;
    
    virtual bool shouldTransition(const spacecraft::SimpleSat& satellite) const = 0;
    
    states::State* getSourceState() const { return sourceState_; }
    states::State* getTargetState() const { return targetState_; }

protected:
    Transition(states::State* source, states::State* target)
        : sourceState_(source), targetState_(target) {}

private:
    states::State* sourceState_;
    states::State* targetState_;
};

} // namespace transitions
} // namespace machine

#endif // TRANSITION_HPP 