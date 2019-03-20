#pragma once
#include <Eigen/Dense>

static const double b = 0.000001;
static const double rho = 1.225000;
static const double g = 9.806550;
static const double pitch = 0.230000;
static const double motor_mass = 0.075000;
static const double body_arm = 0.360000;
static const double N_P = 7.000000;
static const double Km = 0.033500;
static const double Cd = 0.010000;
static const double blade_width = 0.020000;
static const double k_theta = 51.800000;
static const double S = 0.009200;
static const double R = 0.070000;
static const double body_mass = 2.600000;
static const double blade_length = 0.230000;
static const double lambda_m = 0.002760;
static const double blade_mass = 0.021000;
static const double motor_radius = 0.029000;

Eigen::Matrix<double,37,37> get_mm(Eigen::Matrix<double,37,1> states, Eigen::Matrix<double,4,1> inputs);
Eigen::Matrix<double,37,1> get_fo(Eigen::Matrix<double,37,1> states, Eigen::Matrix<double,4,1> inputs);
Eigen::Matrix<double,8,1> get_blade_v_z(Eigen::Matrix<double,37,1> states, Eigen::Matrix<double,4,1> inputs);Eigen::Matrix<double,8,1> get_blade_v_2(Eigen::Matrix<double,37,1> states, Eigen::Matrix<double,4,1> inputs);