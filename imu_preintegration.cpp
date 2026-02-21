#include<iostream>

#include"imu.hpp"
#include"nav_state.hpp"
#include"imu_preintegration.hpp"
#include"SophusUtil.hpp"

IMUPreintegration::IMUPreintegration(PreIntOptions options){
    ba_ = options.init_ba_;
    bg_ = options.init_bg_;
    const float na2 = options.noise_acce_ * options.noise_acce_;
    const float ng2 = options.noise_gyro_ * options.noise_gyro_;
    noise_gyro_acce_.diagonal() << ng2, ng2, ng2, na2, na2, na2;  //Cov(η d,k )  eq(4.26)
}

void IMUPreintegration::Integrate(const IMU& imu, double dt){

    // Basic assumption, fix bias during pre-integration process 
    Eigen::Vector3d gyro = imu.gyro_ - bg_;
    Eigen::Vector3d acce = imu.acce_ - ba_; 

    // pre-integration 
    // // Rotation  
    // ∆R˜ij = ∆R˜ij-1 * Exp ((ω˜j-1 − bg,i)∆t)  => eq(4.9)
    // // Velocity 
    // ∆ṽij = ∆ṽ ij-1 + ∆R̃i,j-1 (ãj-1 − ba,i )* ∆t => eq(4.13) 
    // // Translation 
    // ∆p̃ij = ∆p̃ij-1 + ∆ṽi,j-1 * ∆t + 0.5 * ∆R̃ij-1 (ãj-1 − ba,i ) * ∆t *  ∆t => eq(4.16)

    // First translation => depends on  ∆ṽi,j-1 and ∆R̃ij-1 
    dp_ = dp_ +  dv_ * dt + 0.5f * dR_.matrix() * acce * dt * dt ; 

    // Then Velocity => depends on ∆R̃ij-1
    dv_ = dv_ + dR_ * acce * dt; 

    // UPDATE dR_ afterwards as A, and B matrix depnds on ∆R˜ij-1
    // A Matrix 
    Eigen::Matrix<double, 9 , 9> A;
    A.setIdentity();

    Eigen::Matrix<double, 3, 3> acc_hat = Sophus::SO3d::hat(acce);
    double dt2 = dt*dt; 

    // Velocity
    A.block<3, 3>(3, 0) = -dR_.matrix() * dt * acc_hat;

    // Translation 
    A.block<3, 3>(6, 0) = -0.5f * dR_.matrix() * acc_hat * dt2;
    A.block<3, 3>(6, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity() ;

    // B Matrix
    Eigen::Matrix<double, 9 ,6> B;
    B.setZero();
    B.block<3, 3>(3, 3) = dR_.matrix() * dt; 
    B.block<3, 3>(6, 3) = 0.5f * dR_.matrix() * dt2;

    // Jacobian with respect to bias 
    // Translation 
    dP_dba_ = dP_dba_ + dV_dba_ * dt - 0.5f * dR_.matrix() * dt2;  
    dP_dbg_ = dP_dbg_ + dV_dbg_ * dt - 0.5f * dR_.matrix() * dt2 * acc_hat * dR_dbg_; 

    // Vel
    dV_dba_ = dV_dba_ - dR_.matrix() * dt;
    dV_dbg_ = dV_dbg_ - dR_.matrix() * dt * acc_hat * dR_dbg_;
    
    Eigen::Vector3d omega = gyro * dt;
    Eigen::Matrix<double, 3, 3> rightJ = Sophus::SO3d::jr(omega);
    Sophus::SO3d delta_R = Sophus::SO3d::exp(omega);

    // Eigen::Matrix<double, 3, 3> rightJ = Sophus::SO3d::Dx_exp_x(omega);
    // Eigen::Matrix<double, 3, 3> rightJ = SophusUtil::jr(omega);

    // Update Pre-Integration Rotation
    dR_ = dR_ * delta_R; // eq(4.9)  Add rotation to GLOBAL ROTATION 
    
    // Update Rotation Matrix 
    A.block<3, 3>(0, 0) = delta_R.matrix().transpose(); // ∆R̃⊤[j-1, j] = delta_R

    // Update B matrix 
    B.block<3, 3>(0, 0) = rightJ * dt;

    // Update Bias for Rotation 
    dR_dbg_ = delta_R.matrix().transpose() * dR_dbg_ - rightJ * dt; // ∆R̃⊤[j-1, j] = delta_R

    // UPDATE COV_
    cov_ = A * cov_ * A.transpose() + B * noise_gyro_acce_ * B.transpose();

    // ACCUMLATE  TIME 
    dt_ += dt; 

    // Optional Store IMU 
    imu_msgs_.push_back(imu); 
}

Sophus::SO3d IMUPreintegration::GetDeltaRotation(const Eigen::Vector3d& bg){
    return dR_* Sophus::SO3d::exp( dR_dbg_* (bg - bg_)) ; // RETURNS ROT IN LIE-ALGEBRA 
}

Eigen::Vector3d IMUPreintegration::GetDeltaVelocity(const Eigen::Vector3d& bg, const Eigen::Vector3d& ba){
    return dv_ + dV_dba_ * (ba - ba_) + dV_dbg_ *(bg - bg_);
}

Eigen::Vector3d IMUPreintegration::GetDeltaPosition(const Eigen::Vector3d& bg, const Eigen::Vector3d& ba){
    return dp_+ dP_dbg_ * (bg - bg_) + dP_dba_ * (ba - ba_); 
}

NavStated IMUPreintegration::Predict(const NavStated& start, const Eigen::Vector3d& grav) const{ // eq(4.4)
    // R(t + ∆t) = R(t) Exp (( ω̃ − bg(t) − ηgd(t))∆t)
    Sophus::SO3d Rj = start.R_ * dR_;  // dR_ = Exp (( ω̃ − bg(t) − ηgd(t))∆t) 
    
    // v(t + ∆t) = v(t) + g∆t + R(t)(ã − b a (t) − η ad (t))∆t
    Eigen::Vector3d Vj = start.v_ + start.R_ * dv_ + grav * dt_; // start.R_ * dv_ = R(t)(ã − ba(t) − ηad(t))∆t

    // p(t + ∆t) = p(t) + v(t)∆t + 0.5 g ∆t2 + R(t)(ã − ba(t) − ηad (t))∆t 2
    Eigen::Vector3d Pj = start.p_ + start.v_*dt_ + 0.5f * grav * dt_ * dt_ + start.R_ * dp_ ; 
    
    NavStated state = NavStated(start.timestamp_+ dt_, Rj, Pj, Vj);

    state.ba_ = ba_; 
    state.bg_ = bg_; 

    return state;
}

// int main(int argc, char* argv[]){
//     NavStated start_status(0), end_status(1.0);
//     std::cout << " START : " << start_status << std::endl;
//     std::cout << " END : " << end_status << std::endl;
//     return 0; 
// }