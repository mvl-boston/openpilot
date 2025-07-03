#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_8878700297448567054);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_513812887149974725);
void car_H_mod_fun(double *state, double *out_2373270938182406718);
void car_f_fun(double *state, double dt, double *out_2381229654239969940);
void car_F_fun(double *state, double dt, double *out_6896759999220772990);
void car_h_25(double *state, double *unused, double *out_5005375653476039997);
void car_H_25(double *state, double *unused, double *out_8470179304788781334);
void car_h_24(double *state, double *unused, double *out_5195535264926175380);
void car_H_24(double *state, double *unused, double *out_7803915169915270716);
void car_h_30(double *state, double *unused, double *out_7077774441931695758);
void car_H_30(double *state, double *unused, double *out_1553488963297164579);
void car_h_26(double *state, double *unused, double *out_5726239760696899023);
void car_H_26(double *state, double *unused, double *out_5165653335027980733);
void car_h_27(double *state, double *unused, double *out_8439695152572665974);
void car_H_27(double *state, double *unused, double *out_3728252275097589490);
void car_h_29(double *state, double *unused, double *out_7311639738708674652);
void car_H_29(double *state, double *unused, double *out_5441615001967140523);
void car_h_28(double *state, double *unused, double *out_6959767047998319014);
void car_H_28(double *state, double *unused, double *out_7922730054672880519);
void car_h_31(double *state, double *unused, double *out_1143692177047538345);
void car_H_31(double *state, double *unused, double *out_1393504054276964081);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}