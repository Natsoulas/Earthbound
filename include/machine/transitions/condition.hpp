#ifndef CONDITION_TRANSITION_HPP
#define CONDITION_TRANSITION_HPP

#include "transition.hpp"
#include <functional>

namespace machine {
namespace transitions {

class ConditionTransition : public Transition {
public:
    ConditionTransition(states::State* source, states::State* target, 
                       std::function<bool(const spacecraft::SimpleSat&)> condition)
        : Transition(source, target), condition_(condition) {}
    
    bool shouldTransition(const spacecraft::SimpleSat& satellite) const override {
        return condition_(satellite);
    }

private:
    std::function<bool(const spacecraft::SimpleSat&)> condition_;
};

} // namespace transitions
} // namespace machine

#endif // CONDITION_TRANSITION_HPP
