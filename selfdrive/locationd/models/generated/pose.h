#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_5378340430612614676);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_609280258945418386);
void pose_H_mod_fun(double *state, double *out_2046558306816276168);
void pose_f_fun(double *state, double dt, double *out_5264433078543911283);
void pose_F_fun(double *state, double dt, double *out_176752352757344932);
void pose_h_4(double *state, double *unused, double *out_7116964093078894538);
void pose_H_4(double *state, double *unused, double *out_7199076691855356903);
void pose_h_10(double *state, double *unused, double *out_1609865709454132551);
void pose_H_10(double *state, double *unused, double *out_7711464682056337641);
void pose_h_13(double *state, double *unused, double *out_3359689799482185332);
void pose_H_13(double *state, double *unused, double *out_8035393556521861912);
void pose_h_14(double *state, double *unused, double *out_5734177569564106995);
void pose_H_14(double *state, double *unused, double *out_7284426525514710184);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}