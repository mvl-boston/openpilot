#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8878700297448567054) {
   out_8878700297448567054[0] = delta_x[0] + nom_x[0];
   out_8878700297448567054[1] = delta_x[1] + nom_x[1];
   out_8878700297448567054[2] = delta_x[2] + nom_x[2];
   out_8878700297448567054[3] = delta_x[3] + nom_x[3];
   out_8878700297448567054[4] = delta_x[4] + nom_x[4];
   out_8878700297448567054[5] = delta_x[5] + nom_x[5];
   out_8878700297448567054[6] = delta_x[6] + nom_x[6];
   out_8878700297448567054[7] = delta_x[7] + nom_x[7];
   out_8878700297448567054[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_513812887149974725) {
   out_513812887149974725[0] = -nom_x[0] + true_x[0];
   out_513812887149974725[1] = -nom_x[1] + true_x[1];
   out_513812887149974725[2] = -nom_x[2] + true_x[2];
   out_513812887149974725[3] = -nom_x[3] + true_x[3];
   out_513812887149974725[4] = -nom_x[4] + true_x[4];
   out_513812887149974725[5] = -nom_x[5] + true_x[5];
   out_513812887149974725[6] = -nom_x[6] + true_x[6];
   out_513812887149974725[7] = -nom_x[7] + true_x[7];
   out_513812887149974725[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2373270938182406718) {
   out_2373270938182406718[0] = 1.0;
   out_2373270938182406718[1] = 0.0;
   out_2373270938182406718[2] = 0.0;
   out_2373270938182406718[3] = 0.0;
   out_2373270938182406718[4] = 0.0;
   out_2373270938182406718[5] = 0.0;
   out_2373270938182406718[6] = 0.0;
   out_2373270938182406718[7] = 0.0;
   out_2373270938182406718[8] = 0.0;
   out_2373270938182406718[9] = 0.0;
   out_2373270938182406718[10] = 1.0;
   out_2373270938182406718[11] = 0.0;
   out_2373270938182406718[12] = 0.0;
   out_2373270938182406718[13] = 0.0;
   out_2373270938182406718[14] = 0.0;
   out_2373270938182406718[15] = 0.0;
   out_2373270938182406718[16] = 0.0;
   out_2373270938182406718[17] = 0.0;
   out_2373270938182406718[18] = 0.0;
   out_2373270938182406718[19] = 0.0;
   out_2373270938182406718[20] = 1.0;
   out_2373270938182406718[21] = 0.0;
   out_2373270938182406718[22] = 0.0;
   out_2373270938182406718[23] = 0.0;
   out_2373270938182406718[24] = 0.0;
   out_2373270938182406718[25] = 0.0;
   out_2373270938182406718[26] = 0.0;
   out_2373270938182406718[27] = 0.0;
   out_2373270938182406718[28] = 0.0;
   out_2373270938182406718[29] = 0.0;
   out_2373270938182406718[30] = 1.0;
   out_2373270938182406718[31] = 0.0;
   out_2373270938182406718[32] = 0.0;
   out_2373270938182406718[33] = 0.0;
   out_2373270938182406718[34] = 0.0;
   out_2373270938182406718[35] = 0.0;
   out_2373270938182406718[36] = 0.0;
   out_2373270938182406718[37] = 0.0;
   out_2373270938182406718[38] = 0.0;
   out_2373270938182406718[39] = 0.0;
   out_2373270938182406718[40] = 1.0;
   out_2373270938182406718[41] = 0.0;
   out_2373270938182406718[42] = 0.0;
   out_2373270938182406718[43] = 0.0;
   out_2373270938182406718[44] = 0.0;
   out_2373270938182406718[45] = 0.0;
   out_2373270938182406718[46] = 0.0;
   out_2373270938182406718[47] = 0.0;
   out_2373270938182406718[48] = 0.0;
   out_2373270938182406718[49] = 0.0;
   out_2373270938182406718[50] = 1.0;
   out_2373270938182406718[51] = 0.0;
   out_2373270938182406718[52] = 0.0;
   out_2373270938182406718[53] = 0.0;
   out_2373270938182406718[54] = 0.0;
   out_2373270938182406718[55] = 0.0;
   out_2373270938182406718[56] = 0.0;
   out_2373270938182406718[57] = 0.0;
   out_2373270938182406718[58] = 0.0;
   out_2373270938182406718[59] = 0.0;
   out_2373270938182406718[60] = 1.0;
   out_2373270938182406718[61] = 0.0;
   out_2373270938182406718[62] = 0.0;
   out_2373270938182406718[63] = 0.0;
   out_2373270938182406718[64] = 0.0;
   out_2373270938182406718[65] = 0.0;
   out_2373270938182406718[66] = 0.0;
   out_2373270938182406718[67] = 0.0;
   out_2373270938182406718[68] = 0.0;
   out_2373270938182406718[69] = 0.0;
   out_2373270938182406718[70] = 1.0;
   out_2373270938182406718[71] = 0.0;
   out_2373270938182406718[72] = 0.0;
   out_2373270938182406718[73] = 0.0;
   out_2373270938182406718[74] = 0.0;
   out_2373270938182406718[75] = 0.0;
   out_2373270938182406718[76] = 0.0;
   out_2373270938182406718[77] = 0.0;
   out_2373270938182406718[78] = 0.0;
   out_2373270938182406718[79] = 0.0;
   out_2373270938182406718[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2381229654239969940) {
   out_2381229654239969940[0] = state[0];
   out_2381229654239969940[1] = state[1];
   out_2381229654239969940[2] = state[2];
   out_2381229654239969940[3] = state[3];
   out_2381229654239969940[4] = state[4];
   out_2381229654239969940[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2381229654239969940[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2381229654239969940[7] = state[7];
   out_2381229654239969940[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6896759999220772990) {
   out_6896759999220772990[0] = 1;
   out_6896759999220772990[1] = 0;
   out_6896759999220772990[2] = 0;
   out_6896759999220772990[3] = 0;
   out_6896759999220772990[4] = 0;
   out_6896759999220772990[5] = 0;
   out_6896759999220772990[6] = 0;
   out_6896759999220772990[7] = 0;
   out_6896759999220772990[8] = 0;
   out_6896759999220772990[9] = 0;
   out_6896759999220772990[10] = 1;
   out_6896759999220772990[11] = 0;
   out_6896759999220772990[12] = 0;
   out_6896759999220772990[13] = 0;
   out_6896759999220772990[14] = 0;
   out_6896759999220772990[15] = 0;
   out_6896759999220772990[16] = 0;
   out_6896759999220772990[17] = 0;
   out_6896759999220772990[18] = 0;
   out_6896759999220772990[19] = 0;
   out_6896759999220772990[20] = 1;
   out_6896759999220772990[21] = 0;
   out_6896759999220772990[22] = 0;
   out_6896759999220772990[23] = 0;
   out_6896759999220772990[24] = 0;
   out_6896759999220772990[25] = 0;
   out_6896759999220772990[26] = 0;
   out_6896759999220772990[27] = 0;
   out_6896759999220772990[28] = 0;
   out_6896759999220772990[29] = 0;
   out_6896759999220772990[30] = 1;
   out_6896759999220772990[31] = 0;
   out_6896759999220772990[32] = 0;
   out_6896759999220772990[33] = 0;
   out_6896759999220772990[34] = 0;
   out_6896759999220772990[35] = 0;
   out_6896759999220772990[36] = 0;
   out_6896759999220772990[37] = 0;
   out_6896759999220772990[38] = 0;
   out_6896759999220772990[39] = 0;
   out_6896759999220772990[40] = 1;
   out_6896759999220772990[41] = 0;
   out_6896759999220772990[42] = 0;
   out_6896759999220772990[43] = 0;
   out_6896759999220772990[44] = 0;
   out_6896759999220772990[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6896759999220772990[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6896759999220772990[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6896759999220772990[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6896759999220772990[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6896759999220772990[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6896759999220772990[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6896759999220772990[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6896759999220772990[53] = -9.8000000000000007*dt;
   out_6896759999220772990[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6896759999220772990[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6896759999220772990[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6896759999220772990[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6896759999220772990[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6896759999220772990[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6896759999220772990[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6896759999220772990[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6896759999220772990[62] = 0;
   out_6896759999220772990[63] = 0;
   out_6896759999220772990[64] = 0;
   out_6896759999220772990[65] = 0;
   out_6896759999220772990[66] = 0;
   out_6896759999220772990[67] = 0;
   out_6896759999220772990[68] = 0;
   out_6896759999220772990[69] = 0;
   out_6896759999220772990[70] = 1;
   out_6896759999220772990[71] = 0;
   out_6896759999220772990[72] = 0;
   out_6896759999220772990[73] = 0;
   out_6896759999220772990[74] = 0;
   out_6896759999220772990[75] = 0;
   out_6896759999220772990[76] = 0;
   out_6896759999220772990[77] = 0;
   out_6896759999220772990[78] = 0;
   out_6896759999220772990[79] = 0;
   out_6896759999220772990[80] = 1;
}
void h_25(double *state, double *unused, double *out_5005375653476039997) {
   out_5005375653476039997[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8470179304788781334) {
   out_8470179304788781334[0] = 0;
   out_8470179304788781334[1] = 0;
   out_8470179304788781334[2] = 0;
   out_8470179304788781334[3] = 0;
   out_8470179304788781334[4] = 0;
   out_8470179304788781334[5] = 0;
   out_8470179304788781334[6] = 1;
   out_8470179304788781334[7] = 0;
   out_8470179304788781334[8] = 0;
}
void h_24(double *state, double *unused, double *out_5195535264926175380) {
   out_5195535264926175380[0] = state[4];
   out_5195535264926175380[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7803915169915270716) {
   out_7803915169915270716[0] = 0;
   out_7803915169915270716[1] = 0;
   out_7803915169915270716[2] = 0;
   out_7803915169915270716[3] = 0;
   out_7803915169915270716[4] = 1;
   out_7803915169915270716[5] = 0;
   out_7803915169915270716[6] = 0;
   out_7803915169915270716[7] = 0;
   out_7803915169915270716[8] = 0;
   out_7803915169915270716[9] = 0;
   out_7803915169915270716[10] = 0;
   out_7803915169915270716[11] = 0;
   out_7803915169915270716[12] = 0;
   out_7803915169915270716[13] = 0;
   out_7803915169915270716[14] = 1;
   out_7803915169915270716[15] = 0;
   out_7803915169915270716[16] = 0;
   out_7803915169915270716[17] = 0;
}
void h_30(double *state, double *unused, double *out_7077774441931695758) {
   out_7077774441931695758[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1553488963297164579) {
   out_1553488963297164579[0] = 0;
   out_1553488963297164579[1] = 0;
   out_1553488963297164579[2] = 0;
   out_1553488963297164579[3] = 0;
   out_1553488963297164579[4] = 1;
   out_1553488963297164579[5] = 0;
   out_1553488963297164579[6] = 0;
   out_1553488963297164579[7] = 0;
   out_1553488963297164579[8] = 0;
}
void h_26(double *state, double *unused, double *out_5726239760696899023) {
   out_5726239760696899023[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5165653335027980733) {
   out_5165653335027980733[0] = 0;
   out_5165653335027980733[1] = 0;
   out_5165653335027980733[2] = 0;
   out_5165653335027980733[3] = 0;
   out_5165653335027980733[4] = 0;
   out_5165653335027980733[5] = 0;
   out_5165653335027980733[6] = 0;
   out_5165653335027980733[7] = 1;
   out_5165653335027980733[8] = 0;
}
void h_27(double *state, double *unused, double *out_8439695152572665974) {
   out_8439695152572665974[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3728252275097589490) {
   out_3728252275097589490[0] = 0;
   out_3728252275097589490[1] = 0;
   out_3728252275097589490[2] = 0;
   out_3728252275097589490[3] = 1;
   out_3728252275097589490[4] = 0;
   out_3728252275097589490[5] = 0;
   out_3728252275097589490[6] = 0;
   out_3728252275097589490[7] = 0;
   out_3728252275097589490[8] = 0;
}
void h_29(double *state, double *unused, double *out_7311639738708674652) {
   out_7311639738708674652[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5441615001967140523) {
   out_5441615001967140523[0] = 0;
   out_5441615001967140523[1] = 1;
   out_5441615001967140523[2] = 0;
   out_5441615001967140523[3] = 0;
   out_5441615001967140523[4] = 0;
   out_5441615001967140523[5] = 0;
   out_5441615001967140523[6] = 0;
   out_5441615001967140523[7] = 0;
   out_5441615001967140523[8] = 0;
}
void h_28(double *state, double *unused, double *out_6959767047998319014) {
   out_6959767047998319014[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7922730054672880519) {
   out_7922730054672880519[0] = 1;
   out_7922730054672880519[1] = 0;
   out_7922730054672880519[2] = 0;
   out_7922730054672880519[3] = 0;
   out_7922730054672880519[4] = 0;
   out_7922730054672880519[5] = 0;
   out_7922730054672880519[6] = 0;
   out_7922730054672880519[7] = 0;
   out_7922730054672880519[8] = 0;
}
void h_31(double *state, double *unused, double *out_1143692177047538345) {
   out_1143692177047538345[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1393504054276964081) {
   out_1393504054276964081[0] = 0;
   out_1393504054276964081[1] = 0;
   out_1393504054276964081[2] = 0;
   out_1393504054276964081[3] = 0;
   out_1393504054276964081[4] = 0;
   out_1393504054276964081[5] = 0;
   out_1393504054276964081[6] = 0;
   out_1393504054276964081[7] = 0;
   out_1393504054276964081[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_8878700297448567054) {
  err_fun(nom_x, delta_x, out_8878700297448567054);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_513812887149974725) {
  inv_err_fun(nom_x, true_x, out_513812887149974725);
}
void car_H_mod_fun(double *state, double *out_2373270938182406718) {
  H_mod_fun(state, out_2373270938182406718);
}
void car_f_fun(double *state, double dt, double *out_2381229654239969940) {
  f_fun(state,  dt, out_2381229654239969940);
}
void car_F_fun(double *state, double dt, double *out_6896759999220772990) {
  F_fun(state,  dt, out_6896759999220772990);
}
void car_h_25(double *state, double *unused, double *out_5005375653476039997) {
  h_25(state, unused, out_5005375653476039997);
}
void car_H_25(double *state, double *unused, double *out_8470179304788781334) {
  H_25(state, unused, out_8470179304788781334);
}
void car_h_24(double *state, double *unused, double *out_5195535264926175380) {
  h_24(state, unused, out_5195535264926175380);
}
void car_H_24(double *state, double *unused, double *out_7803915169915270716) {
  H_24(state, unused, out_7803915169915270716);
}
void car_h_30(double *state, double *unused, double *out_7077774441931695758) {
  h_30(state, unused, out_7077774441931695758);
}
void car_H_30(double *state, double *unused, double *out_1553488963297164579) {
  H_30(state, unused, out_1553488963297164579);
}
void car_h_26(double *state, double *unused, double *out_5726239760696899023) {
  h_26(state, unused, out_5726239760696899023);
}
void car_H_26(double *state, double *unused, double *out_5165653335027980733) {
  H_26(state, unused, out_5165653335027980733);
}
void car_h_27(double *state, double *unused, double *out_8439695152572665974) {
  h_27(state, unused, out_8439695152572665974);
}
void car_H_27(double *state, double *unused, double *out_3728252275097589490) {
  H_27(state, unused, out_3728252275097589490);
}
void car_h_29(double *state, double *unused, double *out_7311639738708674652) {
  h_29(state, unused, out_7311639738708674652);
}
void car_H_29(double *state, double *unused, double *out_5441615001967140523) {
  H_29(state, unused, out_5441615001967140523);
}
void car_h_28(double *state, double *unused, double *out_6959767047998319014) {
  h_28(state, unused, out_6959767047998319014);
}
void car_H_28(double *state, double *unused, double *out_7922730054672880519) {
  H_28(state, unused, out_7922730054672880519);
}
void car_h_31(double *state, double *unused, double *out_1143692177047538345) {
  h_31(state, unused, out_1143692177047538345);
}
void car_H_31(double *state, double *unused, double *out_1393504054276964081) {
  H_31(state, unused, out_1393504054276964081);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
