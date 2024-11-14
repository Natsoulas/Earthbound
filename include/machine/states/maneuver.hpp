#ifndef MANEUVER_STATE_HPP
#define MANEUVER_STATE_HPP

#include "state.hpp"
#include <iostream>

namespace machine {
namespace states {

class ManeuverState : public State {
public:
    enum class ManeuverType {
        RAISE_APOGEE,
        RAISE_PERIGEE,
        LOWER_APOGEE,
        LOWER_PERIGEE
    };

    ManeuverState(ManeuverType type, double target_altitude_km)
        : type_(type), targetAltitude_(target_altitude_km) {}

    void enter() override {
        std::cout << "Entering Maneuver mode: " 
                  << (type_ == ManeuverType::RAISE_APOGEE ? "Raise Apogee" : "Raise Perigee")
                  << std::endl;
    }
    
    void execute(spacecraft::SimpleSat& satellite) override {
        switch(type_) {
            case ManeuverType::RAISE_APOGEE:
                satellite.raiseApogee(targetAltitude_);
                break;
            case ManeuverType::RAISE_PERIGEE:
                satellite.raisePerigee(targetAltitude_);
                break;
            default:
                break;
        }
    }
    
    void exit() override {
        std::cout << "Exiting Maneuver mode" << std::endl;
    }
    
    const char* getName() const override { return "Maneuver"; }

private:
    ManeuverType type_;
    double targetAltitude_;
};

} // namespace states
} // namespace machine

#endif // MANEUVER_STATE_HPP
