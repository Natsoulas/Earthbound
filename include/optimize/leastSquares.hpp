/*
 * Copyright (c) 2024 Niko Natsoulas
 * 
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */
#pragma once

#include <Eigen/Dense>
#include <Eigen/QR>
#include <limits>

namespace optimize {

/**
 * @brief Control allocation solver using least squares optimization
 * 
 * Solves the control allocation problem of the form:
 * minimize ||Bu - v||^2 + gamma||u||^2
 * subject to u_min <= u <= u_max
 * 
 * where:
 * B = control effectiveness matrix
 * u = control inputs (to be solved for)
 * v = desired forces/torques
 * gamma = weight for control minimization (optional)
 */
class ControlAllocator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Constructor
     * @param B Control effectiveness matrix mapping controls to forces/torques
     * @param u_min Minimum control limits
     * @param u_max Maximum control limits 
     * @param gamma Optional weight for control minimization (default = 0)
     */
    ControlAllocator(const Eigen::MatrixXd& B,
                    const Eigen::VectorXd& u_min,
                    const Eigen::VectorXd& u_max,
                    double gamma = 0.0)
        : B_(B),
          u_min_(u_min),
          u_max_(u_max),
          gamma_(gamma),
          m_(B.rows()),
          n_(B.cols()) {
        
        // Pre-compute matrices used in optimization
        if (gamma_ > 0) {
            // Augment B matrix with weighted identity for control minimization
            Eigen::MatrixXd B_aug(m_ + n_, n_);
            B_aug << B, 
                    sqrt(gamma_) * Eigen::MatrixXd::Identity(n_, n_);
            qr_.compute(B_aug);
        } else {
            qr_.compute(B);
        }
    }

    /**
     * @brief Solve the control allocation problem
     * @param v Desired forces/torques
     * @param u Output control solution
     * @return True if successful
     */
    bool solve(const Eigen::VectorXd& v, Eigen::VectorXd& u) {
        if (v.size() != m_) {
            return false;
        }

        // Solve unconstrained least squares
        if (gamma_ > 0) {
            // Augment desired vector with zeros for control minimization
            Eigen::VectorXd v_aug(m_ + n_);
            v_aug << v, Eigen::VectorXd::Zero(n_);
            u = qr_.solve(v_aug);
        } else {
            u = qr_.solve(v);
        }

        // Project solution onto constraints
        for (int i = 0; i < n_; i++) {
            u(i) = std::max(u_min_(i), std::min(u_max_(i), u(i)));
        }

        return true;
    }

    /**
     * @brief Get the last solution error norm
     * @return Error norm ||Bu - v||
     */
    double getError() const {
        return qr_.info() == Eigen::Success ? 
               (B_ * qr_.solution()).norm() : 
               std::numeric_limits<double>::infinity();
    }

private:
    Eigen::MatrixXd B_;                  // Control effectiveness matrix
    Eigen::VectorXd u_min_;              // Min control limits
    Eigen::VectorXd u_max_;              // Max control limits
    double gamma_;                       // Control minimization weight
    int m_;                             // Number of objectives (forces/torques)
    int n_;                             // Number of controls
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr_; // QR solver
};

} // namespace optimize
