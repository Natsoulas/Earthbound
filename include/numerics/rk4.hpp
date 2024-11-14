/*
 * Copyright (c) 2024 Niko Natsoulas
 * 
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

#ifndef RK4_HPP
#define RK4_HPP

#include <Eigen/Dense>
#include <functional>

namespace numerics {

class RK4 {
public:
    using StateVector = Eigen::VectorXd;
    using DerivativeFunction = std::function<StateVector(const StateVector&)>;

    /**
     * Performs a single RK4 integration step
     * @param f Function that computes state derivative
     * @param state Current state vector
     * @param dt Time step
     * @return Updated state vector
     */
    static StateVector step(
        const DerivativeFunction& f,
        const StateVector& state,
        double dt
    ) {
        StateVector k1 = f(state);
        StateVector k2 = f(state + (dt/2.0) * k1);
        StateVector k3 = f(state + (dt/2.0) * k2);
        StateVector k4 = f(state + dt * k3);

        return state + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4);
    }

    /**
     * Integrates the system for a specified duration
     * @param f Function that computes state derivative
     * @param state Initial state vector
     * @param t0 Initial time
     * @param tf Final time
     * @param dt Time step
     * @return Final state vector
     */
    static StateVector integrate(
        const DerivativeFunction& f,
        const StateVector& state,
        double t0,
        double tf,
        double dt
    ) {
        StateVector current_state = state;
        double current_time = t0;

        while(current_time < tf) {
            // Adjust final step if needed
            if(current_time + dt > tf) {
                dt = tf - current_time;
            }
            
            current_state = step(f, current_state, dt);
            current_time += dt;
        }

        return current_state;
    }
};

} // namespace numerics

#endif // RK4_HPP
