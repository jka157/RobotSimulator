#include <stdio.h>
#include <conio.h>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <fstream>
#include <chrono>
#include <thread>
#include <algorithm>

#include "stdafx.h"
#include "ensc-488.h"

#define L10 10
#define L30 30
#define L70 70
#define L80 80
#define L130 130
#define L135 135
#define L140 140
#define L142 142
#define L195 195
#define L405 405
#define L410 410
#define PI 3.14159265
#define N 4
#define FK 666

// Position constraints
#define THETA1_CONS 150
#define THETA2_CONS 100
#define THETA4_CONS 160
#define D3LOWER_200 -200
#define D3UPPER_100 -100
#define A210 210
#define A150 150

// Velocity constraints
#define J1V_LIM 150 
#define J2V_LIM 150
#define J3V_LIM 50
#define J4V_LIM 150

// Acceleration constraints
#define J1A_LIM 600
#define J2A_LIM 600
#define J3A_LIM 200
#define J4A_LIM 600

using namespace std;

typedef double TFORM[4][4];
typedef double RFORM[3][3];
typedef double CFORM[3][4];
typedef double ARR5[5];
typedef double ARR4[4];
typedef double ARR3[3];
typedef double ARR6[6];

JOINT T{0, 0, L135, 0};
JOINT B{0, 0, L405, 0};
TFORM wrelb, brels, trelw; 
JOINT spt;

void main(void);

// menu stuff
void FwdKinDeg(JOINT &joint_vals, JOINT &spt);
void FwdKinRad(JOINT &joint_vals, JOINT &spt);
void InvKin(JOINT &spt);
void SimpleMove(void);
void Move(void);
void ToggleGripper(bool &status);
void TrajPlanPos(vector<vector<double>>& traj_vals);
void TrajPlanJoint(vector<vector<double>>& traj_vals);
void TrajCust(vector<vector<double>>& traj_vals);
void ExecutePath(vector<vector<double>> traj_vals);

// functions for class
// Basic Matrix Transformation Procedures
void UTOI(JOINT &pos, TFORM &mat);
void UTOI_FLIP(JOINT &pos, TFORM &mat);
void ITOU(TFORM &mat, JOINT &pos);
void TMULT(TFORM &brela, TFORM &crelb, TFORM &crela);
void TINVERT(TFORM &tmat);

// Forward Kinematics
void KIN(JOINT &joint_vals, TFORM &wrelb);
void WHERE(JOINT &joint_vals, JOINT &spt);

// Inverse Kinematics
void INVKIN(TFORM &wrelb, JOINT &near, JOINT &far, bool &p_val, bool &n_val);
void SOLVE(JOINT &tar_pos, JOINT &curr_joint, JOINT &near, JOINT &far, bool &p_val, bool &n_val);

// Helper functions
void print(TFORM &mat);
void print(JOINT &arr);
void print(ARR3 &arr);
void print(ARR5 &arr);
void print(ARR6& arr);
void print(vector<double>& vec);
void get_r(TFORM &tmat, RFORM &rmat);
void get_pos(TFORM &tmat, ARR3 &pos);
void pop_mat(TFORM &vals, TFORM &mat);
void pop_mat(RFORM &vals, RFORM &mat);
void pop_arr(JOINT &vals, JOINT &arr);
void pop_arr(ARR3 &vals, ARR3 &arr);
void pop_arr(ARR5 &vals, ARR5 &arr);
void rmult(RFORM &mat1, RFORM &mat2, RFORM &res);
void rmult(RFORM &mat, ARR3 &pos, ARR3 &res);
void pmult(ARR3 &pos, double val, ARR3 &res);
void padd(ARR3 &pos1, ARR3 &pos2, ARR3 &res);
void tconst(RFORM &rmat, ARR3 &pos, TFORM &tmat);
void transpose_mat(RFORM &rmat, RFORM &imat);

void check_joints(JOINT &joint_vals, bool &valid);
void check_vel(vector<vector<double>> vals, bool &valid);
void check_acc(vector<vector<double>> vals, bool &valid);

void write_csv(string filename, vector<pair<string, vector<double>>> dataset);

// Demo 2
void CUBCOEF(double theta0, double thetaf, double vel0, double velf, double tf, JOINT &coeff);
vector<vector<double>> PATHPLAN(double t, TFORM& A, TFORM& B, TFORM& C, TFORM& G, bool debug);

void PATHGEN(double ti, double tf, int sample_rate, JOINT& coeff, vector<double>& pos, vector<double>& curr_time, bool isFull = false);
void VELGEN(double ti, double tf, int sample_rate, JOINT& coeff, vector<double>& vel, vector<double>& curr_time, bool isFull = false);
void ACCGEN(double ti, double tf, int sample_rate, JOINT& coeff, vector<double>& acc, vector<double>& curr_time, bool isFull = false);

//void PATHGEN(double ti, double tf, int sample_rate, ARR6& coeff, vector<double>& pos, vector<double>& curr_time, bool isFull = false);
//void VELGEN(double ti, double tf, int sample_rate, ARR6& coeff, vector<double>& vel, vector<double>& curr_time, bool isFull = false);
//void ACCGEN(double ti, double tf, int sample_rate, ARR6& coeff, vector<double>& acc, vector<double>& curr_time, bool isFull = false);
void POSGEN(vector<double> theta1, vector<double> theta2, vector<double> &d3, vector<double> theta4, 
	vector<double> &x, vector<double> &y, vector<double> &z, vector<double> &phi);

void compute_coeff(ARR5& j, ARR5& times, JOINT& curra, JOINT& ab, JOINT& bc, JOINT& cg);
void compute_quincoeff(ARR5& j, double t, ARR6& curra, ARR6& ab, ARR6& bc, ARR6& cg, ARR5& times);
void get_jv(int idx, JOINT &curr_joint, JOINT &a_joint, JOINT &b_joint, JOINT &c_joint, JOINT& g_joint, ARR5 &joint);
void to_vec(ARR5 &arr, vector<double> &vec);
void to_arr(vector<double> &vec, ARR4 &arr);
void compute_times(double &t, ARR5 &j1, ARR5 &j2, ARR5 &j3, ARR5 &j4, ARR5 &times);
void compute_slopes(ARR5 &t, ARR5 &j, ARR4 &slopes);
void compute_slopes(ARR5 &t, ARR5 &j, ARR4 &slopes);
void compute_vels(ARR4& slopes, ARR5& vels);
void compute_accs(ARR4& slopes, ARR5& accs);

void find_max(vector<double>& vec, int& idx);
void find_min(vector<double>& vec, int& idx);