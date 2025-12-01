/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      autonomous_driving_config.hpp
 * @brief     autonomous driving configuration
 * 
 * @date      2023-08-07 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 */

#ifndef __AUTONOMOUS_DRIVING_CONFIG_HPP__
#define __AUTONOMOUS_DRIVING_CONFIG_HPP__
#pragma once

// STD Header
#include <string>
#include <cmath>

typedef struct {
    std::string vehicle_namespace{""};
    double loop_rate_hz{100.0};
    bool use_manual_inputs{false};

    ////////////////////// TODO //////////////////////
    // [다훈 수정 5]TODO: Add more parameters (ex: kd, ki, kv, ...) from autonomous_driving.hpp
    //1. 차량 물리 파라미터
    double param_wheel_base{1.302 + 1.398}; // L_f + L_r
    double param_max_lateral_accel{6200.0 / 1319.91}; // Fyf_max / Mass

    //2. Pure Pursuit 제어 파라미터
    double param_pp_kd{3.0};
    double param_pp_kv{0.0};
    double param_pp_kc{0.0};

    //3. PID 제어 파라미터
    double param_pid_kp{5.0};
    double param_pid_ki{0.002};
    double param_pid_kd{0.0};
    double param_brake_ratio{1.2};

    //4. ROI 파라미터
    double param_m_ROIFront_param{20.0};
    double param_m_ROIRear_param{10.0};
    double param_m_ROILeft_param{3.0};
    double param_m_ROIRight_param{3.0};
    std::string ref_csv_path{""};

    // [다훈 수정9] Control Parameters (for longitudinal control)
    const double dt{1.0/100.0};
    
    //Algorithm Parameters
    double speed_error_integral{0.0};
    double speed_error_prev{0.0};

    // [수빈 수정 12.1] RANSAC Parameters for Lane Detection
    int ransac_max_iterations{50};           // RANSAC 최대 반복 횟수
    double ransac_inlier_threshold{0.15};    // inlier 판정 거리 (m)
    double ransac_min_inlier_ratio{0.9};     // 조기 종료를 위한 최소 inlier 비율

    //////////////////////////////////////////////////
} AutonomousDrivingConfig;

#endif // __AUTONOMOUS_DRIVING_CONFIG_HPP__