#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5378340430612614676) {
   out_5378340430612614676[0] = delta_x[0] + nom_x[0];
   out_5378340430612614676[1] = delta_x[1] + nom_x[1];
   out_5378340430612614676[2] = delta_x[2] + nom_x[2];
   out_5378340430612614676[3] = delta_x[3] + nom_x[3];
   out_5378340430612614676[4] = delta_x[4] + nom_x[4];
   out_5378340430612614676[5] = delta_x[5] + nom_x[5];
   out_5378340430612614676[6] = delta_x[6] + nom_x[6];
   out_5378340430612614676[7] = delta_x[7] + nom_x[7];
   out_5378340430612614676[8] = delta_x[8] + nom_x[8];
   out_5378340430612614676[9] = delta_x[9] + nom_x[9];
   out_5378340430612614676[10] = delta_x[10] + nom_x[10];
   out_5378340430612614676[11] = delta_x[11] + nom_x[11];
   out_5378340430612614676[12] = delta_x[12] + nom_x[12];
   out_5378340430612614676[13] = delta_x[13] + nom_x[13];
   out_5378340430612614676[14] = delta_x[14] + nom_x[14];
   out_5378340430612614676[15] = delta_x[15] + nom_x[15];
   out_5378340430612614676[16] = delta_x[16] + nom_x[16];
   out_5378340430612614676[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_609280258945418386) {
   out_609280258945418386[0] = -nom_x[0] + true_x[0];
   out_609280258945418386[1] = -nom_x[1] + true_x[1];
   out_609280258945418386[2] = -nom_x[2] + true_x[2];
   out_609280258945418386[3] = -nom_x[3] + true_x[3];
   out_609280258945418386[4] = -nom_x[4] + true_x[4];
   out_609280258945418386[5] = -nom_x[5] + true_x[5];
   out_609280258945418386[6] = -nom_x[6] + true_x[6];
   out_609280258945418386[7] = -nom_x[7] + true_x[7];
   out_609280258945418386[8] = -nom_x[8] + true_x[8];
   out_609280258945418386[9] = -nom_x[9] + true_x[9];
   out_609280258945418386[10] = -nom_x[10] + true_x[10];
   out_609280258945418386[11] = -nom_x[11] + true_x[11];
   out_609280258945418386[12] = -nom_x[12] + true_x[12];
   out_609280258945418386[13] = -nom_x[13] + true_x[13];
   out_609280258945418386[14] = -nom_x[14] + true_x[14];
   out_609280258945418386[15] = -nom_x[15] + true_x[15];
   out_609280258945418386[16] = -nom_x[16] + true_x[16];
   out_609280258945418386[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_2046558306816276168) {
   out_2046558306816276168[0] = 1.0;
   out_2046558306816276168[1] = 0.0;
   out_2046558306816276168[2] = 0.0;
   out_2046558306816276168[3] = 0.0;
   out_2046558306816276168[4] = 0.0;
   out_2046558306816276168[5] = 0.0;
   out_2046558306816276168[6] = 0.0;
   out_2046558306816276168[7] = 0.0;
   out_2046558306816276168[8] = 0.0;
   out_2046558306816276168[9] = 0.0;
   out_2046558306816276168[10] = 0.0;
   out_2046558306816276168[11] = 0.0;
   out_2046558306816276168[12] = 0.0;
   out_2046558306816276168[13] = 0.0;
   out_2046558306816276168[14] = 0.0;
   out_2046558306816276168[15] = 0.0;
   out_2046558306816276168[16] = 0.0;
   out_2046558306816276168[17] = 0.0;
   out_2046558306816276168[18] = 0.0;
   out_2046558306816276168[19] = 1.0;
   out_2046558306816276168[20] = 0.0;
   out_2046558306816276168[21] = 0.0;
   out_2046558306816276168[22] = 0.0;
   out_2046558306816276168[23] = 0.0;
   out_2046558306816276168[24] = 0.0;
   out_2046558306816276168[25] = 0.0;
   out_2046558306816276168[26] = 0.0;
   out_2046558306816276168[27] = 0.0;
   out_2046558306816276168[28] = 0.0;
   out_2046558306816276168[29] = 0.0;
   out_2046558306816276168[30] = 0.0;
   out_2046558306816276168[31] = 0.0;
   out_2046558306816276168[32] = 0.0;
   out_2046558306816276168[33] = 0.0;
   out_2046558306816276168[34] = 0.0;
   out_2046558306816276168[35] = 0.0;
   out_2046558306816276168[36] = 0.0;
   out_2046558306816276168[37] = 0.0;
   out_2046558306816276168[38] = 1.0;
   out_2046558306816276168[39] = 0.0;
   out_2046558306816276168[40] = 0.0;
   out_2046558306816276168[41] = 0.0;
   out_2046558306816276168[42] = 0.0;
   out_2046558306816276168[43] = 0.0;
   out_2046558306816276168[44] = 0.0;
   out_2046558306816276168[45] = 0.0;
   out_2046558306816276168[46] = 0.0;
   out_2046558306816276168[47] = 0.0;
   out_2046558306816276168[48] = 0.0;
   out_2046558306816276168[49] = 0.0;
   out_2046558306816276168[50] = 0.0;
   out_2046558306816276168[51] = 0.0;
   out_2046558306816276168[52] = 0.0;
   out_2046558306816276168[53] = 0.0;
   out_2046558306816276168[54] = 0.0;
   out_2046558306816276168[55] = 0.0;
   out_2046558306816276168[56] = 0.0;
   out_2046558306816276168[57] = 1.0;
   out_2046558306816276168[58] = 0.0;
   out_2046558306816276168[59] = 0.0;
   out_2046558306816276168[60] = 0.0;
   out_2046558306816276168[61] = 0.0;
   out_2046558306816276168[62] = 0.0;
   out_2046558306816276168[63] = 0.0;
   out_2046558306816276168[64] = 0.0;
   out_2046558306816276168[65] = 0.0;
   out_2046558306816276168[66] = 0.0;
   out_2046558306816276168[67] = 0.0;
   out_2046558306816276168[68] = 0.0;
   out_2046558306816276168[69] = 0.0;
   out_2046558306816276168[70] = 0.0;
   out_2046558306816276168[71] = 0.0;
   out_2046558306816276168[72] = 0.0;
   out_2046558306816276168[73] = 0.0;
   out_2046558306816276168[74] = 0.0;
   out_2046558306816276168[75] = 0.0;
   out_2046558306816276168[76] = 1.0;
   out_2046558306816276168[77] = 0.0;
   out_2046558306816276168[78] = 0.0;
   out_2046558306816276168[79] = 0.0;
   out_2046558306816276168[80] = 0.0;
   out_2046558306816276168[81] = 0.0;
   out_2046558306816276168[82] = 0.0;
   out_2046558306816276168[83] = 0.0;
   out_2046558306816276168[84] = 0.0;
   out_2046558306816276168[85] = 0.0;
   out_2046558306816276168[86] = 0.0;
   out_2046558306816276168[87] = 0.0;
   out_2046558306816276168[88] = 0.0;
   out_2046558306816276168[89] = 0.0;
   out_2046558306816276168[90] = 0.0;
   out_2046558306816276168[91] = 0.0;
   out_2046558306816276168[92] = 0.0;
   out_2046558306816276168[93] = 0.0;
   out_2046558306816276168[94] = 0.0;
   out_2046558306816276168[95] = 1.0;
   out_2046558306816276168[96] = 0.0;
   out_2046558306816276168[97] = 0.0;
   out_2046558306816276168[98] = 0.0;
   out_2046558306816276168[99] = 0.0;
   out_2046558306816276168[100] = 0.0;
   out_2046558306816276168[101] = 0.0;
   out_2046558306816276168[102] = 0.0;
   out_2046558306816276168[103] = 0.0;
   out_2046558306816276168[104] = 0.0;
   out_2046558306816276168[105] = 0.0;
   out_2046558306816276168[106] = 0.0;
   out_2046558306816276168[107] = 0.0;
   out_2046558306816276168[108] = 0.0;
   out_2046558306816276168[109] = 0.0;
   out_2046558306816276168[110] = 0.0;
   out_2046558306816276168[111] = 0.0;
   out_2046558306816276168[112] = 0.0;
   out_2046558306816276168[113] = 0.0;
   out_2046558306816276168[114] = 1.0;
   out_2046558306816276168[115] = 0.0;
   out_2046558306816276168[116] = 0.0;
   out_2046558306816276168[117] = 0.0;
   out_2046558306816276168[118] = 0.0;
   out_2046558306816276168[119] = 0.0;
   out_2046558306816276168[120] = 0.0;
   out_2046558306816276168[121] = 0.0;
   out_2046558306816276168[122] = 0.0;
   out_2046558306816276168[123] = 0.0;
   out_2046558306816276168[124] = 0.0;
   out_2046558306816276168[125] = 0.0;
   out_2046558306816276168[126] = 0.0;
   out_2046558306816276168[127] = 0.0;
   out_2046558306816276168[128] = 0.0;
   out_2046558306816276168[129] = 0.0;
   out_2046558306816276168[130] = 0.0;
   out_2046558306816276168[131] = 0.0;
   out_2046558306816276168[132] = 0.0;
   out_2046558306816276168[133] = 1.0;
   out_2046558306816276168[134] = 0.0;
   out_2046558306816276168[135] = 0.0;
   out_2046558306816276168[136] = 0.0;
   out_2046558306816276168[137] = 0.0;
   out_2046558306816276168[138] = 0.0;
   out_2046558306816276168[139] = 0.0;
   out_2046558306816276168[140] = 0.0;
   out_2046558306816276168[141] = 0.0;
   out_2046558306816276168[142] = 0.0;
   out_2046558306816276168[143] = 0.0;
   out_2046558306816276168[144] = 0.0;
   out_2046558306816276168[145] = 0.0;
   out_2046558306816276168[146] = 0.0;
   out_2046558306816276168[147] = 0.0;
   out_2046558306816276168[148] = 0.0;
   out_2046558306816276168[149] = 0.0;
   out_2046558306816276168[150] = 0.0;
   out_2046558306816276168[151] = 0.0;
   out_2046558306816276168[152] = 1.0;
   out_2046558306816276168[153] = 0.0;
   out_2046558306816276168[154] = 0.0;
   out_2046558306816276168[155] = 0.0;
   out_2046558306816276168[156] = 0.0;
   out_2046558306816276168[157] = 0.0;
   out_2046558306816276168[158] = 0.0;
   out_2046558306816276168[159] = 0.0;
   out_2046558306816276168[160] = 0.0;
   out_2046558306816276168[161] = 0.0;
   out_2046558306816276168[162] = 0.0;
   out_2046558306816276168[163] = 0.0;
   out_2046558306816276168[164] = 0.0;
   out_2046558306816276168[165] = 0.0;
   out_2046558306816276168[166] = 0.0;
   out_2046558306816276168[167] = 0.0;
   out_2046558306816276168[168] = 0.0;
   out_2046558306816276168[169] = 0.0;
   out_2046558306816276168[170] = 0.0;
   out_2046558306816276168[171] = 1.0;
   out_2046558306816276168[172] = 0.0;
   out_2046558306816276168[173] = 0.0;
   out_2046558306816276168[174] = 0.0;
   out_2046558306816276168[175] = 0.0;
   out_2046558306816276168[176] = 0.0;
   out_2046558306816276168[177] = 0.0;
   out_2046558306816276168[178] = 0.0;
   out_2046558306816276168[179] = 0.0;
   out_2046558306816276168[180] = 0.0;
   out_2046558306816276168[181] = 0.0;
   out_2046558306816276168[182] = 0.0;
   out_2046558306816276168[183] = 0.0;
   out_2046558306816276168[184] = 0.0;
   out_2046558306816276168[185] = 0.0;
   out_2046558306816276168[186] = 0.0;
   out_2046558306816276168[187] = 0.0;
   out_2046558306816276168[188] = 0.0;
   out_2046558306816276168[189] = 0.0;
   out_2046558306816276168[190] = 1.0;
   out_2046558306816276168[191] = 0.0;
   out_2046558306816276168[192] = 0.0;
   out_2046558306816276168[193] = 0.0;
   out_2046558306816276168[194] = 0.0;
   out_2046558306816276168[195] = 0.0;
   out_2046558306816276168[196] = 0.0;
   out_2046558306816276168[197] = 0.0;
   out_2046558306816276168[198] = 0.0;
   out_2046558306816276168[199] = 0.0;
   out_2046558306816276168[200] = 0.0;
   out_2046558306816276168[201] = 0.0;
   out_2046558306816276168[202] = 0.0;
   out_2046558306816276168[203] = 0.0;
   out_2046558306816276168[204] = 0.0;
   out_2046558306816276168[205] = 0.0;
   out_2046558306816276168[206] = 0.0;
   out_2046558306816276168[207] = 0.0;
   out_2046558306816276168[208] = 0.0;
   out_2046558306816276168[209] = 1.0;
   out_2046558306816276168[210] = 0.0;
   out_2046558306816276168[211] = 0.0;
   out_2046558306816276168[212] = 0.0;
   out_2046558306816276168[213] = 0.0;
   out_2046558306816276168[214] = 0.0;
   out_2046558306816276168[215] = 0.0;
   out_2046558306816276168[216] = 0.0;
   out_2046558306816276168[217] = 0.0;
   out_2046558306816276168[218] = 0.0;
   out_2046558306816276168[219] = 0.0;
   out_2046558306816276168[220] = 0.0;
   out_2046558306816276168[221] = 0.0;
   out_2046558306816276168[222] = 0.0;
   out_2046558306816276168[223] = 0.0;
   out_2046558306816276168[224] = 0.0;
   out_2046558306816276168[225] = 0.0;
   out_2046558306816276168[226] = 0.0;
   out_2046558306816276168[227] = 0.0;
   out_2046558306816276168[228] = 1.0;
   out_2046558306816276168[229] = 0.0;
   out_2046558306816276168[230] = 0.0;
   out_2046558306816276168[231] = 0.0;
   out_2046558306816276168[232] = 0.0;
   out_2046558306816276168[233] = 0.0;
   out_2046558306816276168[234] = 0.0;
   out_2046558306816276168[235] = 0.0;
   out_2046558306816276168[236] = 0.0;
   out_2046558306816276168[237] = 0.0;
   out_2046558306816276168[238] = 0.0;
   out_2046558306816276168[239] = 0.0;
   out_2046558306816276168[240] = 0.0;
   out_2046558306816276168[241] = 0.0;
   out_2046558306816276168[242] = 0.0;
   out_2046558306816276168[243] = 0.0;
   out_2046558306816276168[244] = 0.0;
   out_2046558306816276168[245] = 0.0;
   out_2046558306816276168[246] = 0.0;
   out_2046558306816276168[247] = 1.0;
   out_2046558306816276168[248] = 0.0;
   out_2046558306816276168[249] = 0.0;
   out_2046558306816276168[250] = 0.0;
   out_2046558306816276168[251] = 0.0;
   out_2046558306816276168[252] = 0.0;
   out_2046558306816276168[253] = 0.0;
   out_2046558306816276168[254] = 0.0;
   out_2046558306816276168[255] = 0.0;
   out_2046558306816276168[256] = 0.0;
   out_2046558306816276168[257] = 0.0;
   out_2046558306816276168[258] = 0.0;
   out_2046558306816276168[259] = 0.0;
   out_2046558306816276168[260] = 0.0;
   out_2046558306816276168[261] = 0.0;
   out_2046558306816276168[262] = 0.0;
   out_2046558306816276168[263] = 0.0;
   out_2046558306816276168[264] = 0.0;
   out_2046558306816276168[265] = 0.0;
   out_2046558306816276168[266] = 1.0;
   out_2046558306816276168[267] = 0.0;
   out_2046558306816276168[268] = 0.0;
   out_2046558306816276168[269] = 0.0;
   out_2046558306816276168[270] = 0.0;
   out_2046558306816276168[271] = 0.0;
   out_2046558306816276168[272] = 0.0;
   out_2046558306816276168[273] = 0.0;
   out_2046558306816276168[274] = 0.0;
   out_2046558306816276168[275] = 0.0;
   out_2046558306816276168[276] = 0.0;
   out_2046558306816276168[277] = 0.0;
   out_2046558306816276168[278] = 0.0;
   out_2046558306816276168[279] = 0.0;
   out_2046558306816276168[280] = 0.0;
   out_2046558306816276168[281] = 0.0;
   out_2046558306816276168[282] = 0.0;
   out_2046558306816276168[283] = 0.0;
   out_2046558306816276168[284] = 0.0;
   out_2046558306816276168[285] = 1.0;
   out_2046558306816276168[286] = 0.0;
   out_2046558306816276168[287] = 0.0;
   out_2046558306816276168[288] = 0.0;
   out_2046558306816276168[289] = 0.0;
   out_2046558306816276168[290] = 0.0;
   out_2046558306816276168[291] = 0.0;
   out_2046558306816276168[292] = 0.0;
   out_2046558306816276168[293] = 0.0;
   out_2046558306816276168[294] = 0.0;
   out_2046558306816276168[295] = 0.0;
   out_2046558306816276168[296] = 0.0;
   out_2046558306816276168[297] = 0.0;
   out_2046558306816276168[298] = 0.0;
   out_2046558306816276168[299] = 0.0;
   out_2046558306816276168[300] = 0.0;
   out_2046558306816276168[301] = 0.0;
   out_2046558306816276168[302] = 0.0;
   out_2046558306816276168[303] = 0.0;
   out_2046558306816276168[304] = 1.0;
   out_2046558306816276168[305] = 0.0;
   out_2046558306816276168[306] = 0.0;
   out_2046558306816276168[307] = 0.0;
   out_2046558306816276168[308] = 0.0;
   out_2046558306816276168[309] = 0.0;
   out_2046558306816276168[310] = 0.0;
   out_2046558306816276168[311] = 0.0;
   out_2046558306816276168[312] = 0.0;
   out_2046558306816276168[313] = 0.0;
   out_2046558306816276168[314] = 0.0;
   out_2046558306816276168[315] = 0.0;
   out_2046558306816276168[316] = 0.0;
   out_2046558306816276168[317] = 0.0;
   out_2046558306816276168[318] = 0.0;
   out_2046558306816276168[319] = 0.0;
   out_2046558306816276168[320] = 0.0;
   out_2046558306816276168[321] = 0.0;
   out_2046558306816276168[322] = 0.0;
   out_2046558306816276168[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_5264433078543911283) {
   out_5264433078543911283[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_5264433078543911283[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_5264433078543911283[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_5264433078543911283[3] = dt*state[12] + state[3];
   out_5264433078543911283[4] = dt*state[13] + state[4];
   out_5264433078543911283[5] = dt*state[14] + state[5];
   out_5264433078543911283[6] = state[6];
   out_5264433078543911283[7] = state[7];
   out_5264433078543911283[8] = state[8];
   out_5264433078543911283[9] = state[9];
   out_5264433078543911283[10] = state[10];
   out_5264433078543911283[11] = state[11];
   out_5264433078543911283[12] = state[12];
   out_5264433078543911283[13] = state[13];
   out_5264433078543911283[14] = state[14];
   out_5264433078543911283[15] = state[15];
   out_5264433078543911283[16] = state[16];
   out_5264433078543911283[17] = state[17];
}
void F_fun(double *state, double dt, double *out_176752352757344932) {
   out_176752352757344932[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_176752352757344932[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_176752352757344932[2] = 0;
   out_176752352757344932[3] = 0;
   out_176752352757344932[4] = 0;
   out_176752352757344932[5] = 0;
   out_176752352757344932[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_176752352757344932[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_176752352757344932[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_176752352757344932[9] = 0;
   out_176752352757344932[10] = 0;
   out_176752352757344932[11] = 0;
   out_176752352757344932[12] = 0;
   out_176752352757344932[13] = 0;
   out_176752352757344932[14] = 0;
   out_176752352757344932[15] = 0;
   out_176752352757344932[16] = 0;
   out_176752352757344932[17] = 0;
   out_176752352757344932[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_176752352757344932[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_176752352757344932[20] = 0;
   out_176752352757344932[21] = 0;
   out_176752352757344932[22] = 0;
   out_176752352757344932[23] = 0;
   out_176752352757344932[24] = 0;
   out_176752352757344932[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_176752352757344932[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_176752352757344932[27] = 0;
   out_176752352757344932[28] = 0;
   out_176752352757344932[29] = 0;
   out_176752352757344932[30] = 0;
   out_176752352757344932[31] = 0;
   out_176752352757344932[32] = 0;
   out_176752352757344932[33] = 0;
   out_176752352757344932[34] = 0;
   out_176752352757344932[35] = 0;
   out_176752352757344932[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_176752352757344932[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_176752352757344932[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_176752352757344932[39] = 0;
   out_176752352757344932[40] = 0;
   out_176752352757344932[41] = 0;
   out_176752352757344932[42] = 0;
   out_176752352757344932[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_176752352757344932[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_176752352757344932[45] = 0;
   out_176752352757344932[46] = 0;
   out_176752352757344932[47] = 0;
   out_176752352757344932[48] = 0;
   out_176752352757344932[49] = 0;
   out_176752352757344932[50] = 0;
   out_176752352757344932[51] = 0;
   out_176752352757344932[52] = 0;
   out_176752352757344932[53] = 0;
   out_176752352757344932[54] = 0;
   out_176752352757344932[55] = 0;
   out_176752352757344932[56] = 0;
   out_176752352757344932[57] = 1;
   out_176752352757344932[58] = 0;
   out_176752352757344932[59] = 0;
   out_176752352757344932[60] = 0;
   out_176752352757344932[61] = 0;
   out_176752352757344932[62] = 0;
   out_176752352757344932[63] = 0;
   out_176752352757344932[64] = 0;
   out_176752352757344932[65] = 0;
   out_176752352757344932[66] = dt;
   out_176752352757344932[67] = 0;
   out_176752352757344932[68] = 0;
   out_176752352757344932[69] = 0;
   out_176752352757344932[70] = 0;
   out_176752352757344932[71] = 0;
   out_176752352757344932[72] = 0;
   out_176752352757344932[73] = 0;
   out_176752352757344932[74] = 0;
   out_176752352757344932[75] = 0;
   out_176752352757344932[76] = 1;
   out_176752352757344932[77] = 0;
   out_176752352757344932[78] = 0;
   out_176752352757344932[79] = 0;
   out_176752352757344932[80] = 0;
   out_176752352757344932[81] = 0;
   out_176752352757344932[82] = 0;
   out_176752352757344932[83] = 0;
   out_176752352757344932[84] = 0;
   out_176752352757344932[85] = dt;
   out_176752352757344932[86] = 0;
   out_176752352757344932[87] = 0;
   out_176752352757344932[88] = 0;
   out_176752352757344932[89] = 0;
   out_176752352757344932[90] = 0;
   out_176752352757344932[91] = 0;
   out_176752352757344932[92] = 0;
   out_176752352757344932[93] = 0;
   out_176752352757344932[94] = 0;
   out_176752352757344932[95] = 1;
   out_176752352757344932[96] = 0;
   out_176752352757344932[97] = 0;
   out_176752352757344932[98] = 0;
   out_176752352757344932[99] = 0;
   out_176752352757344932[100] = 0;
   out_176752352757344932[101] = 0;
   out_176752352757344932[102] = 0;
   out_176752352757344932[103] = 0;
   out_176752352757344932[104] = dt;
   out_176752352757344932[105] = 0;
   out_176752352757344932[106] = 0;
   out_176752352757344932[107] = 0;
   out_176752352757344932[108] = 0;
   out_176752352757344932[109] = 0;
   out_176752352757344932[110] = 0;
   out_176752352757344932[111] = 0;
   out_176752352757344932[112] = 0;
   out_176752352757344932[113] = 0;
   out_176752352757344932[114] = 1;
   out_176752352757344932[115] = 0;
   out_176752352757344932[116] = 0;
   out_176752352757344932[117] = 0;
   out_176752352757344932[118] = 0;
   out_176752352757344932[119] = 0;
   out_176752352757344932[120] = 0;
   out_176752352757344932[121] = 0;
   out_176752352757344932[122] = 0;
   out_176752352757344932[123] = 0;
   out_176752352757344932[124] = 0;
   out_176752352757344932[125] = 0;
   out_176752352757344932[126] = 0;
   out_176752352757344932[127] = 0;
   out_176752352757344932[128] = 0;
   out_176752352757344932[129] = 0;
   out_176752352757344932[130] = 0;
   out_176752352757344932[131] = 0;
   out_176752352757344932[132] = 0;
   out_176752352757344932[133] = 1;
   out_176752352757344932[134] = 0;
   out_176752352757344932[135] = 0;
   out_176752352757344932[136] = 0;
   out_176752352757344932[137] = 0;
   out_176752352757344932[138] = 0;
   out_176752352757344932[139] = 0;
   out_176752352757344932[140] = 0;
   out_176752352757344932[141] = 0;
   out_176752352757344932[142] = 0;
   out_176752352757344932[143] = 0;
   out_176752352757344932[144] = 0;
   out_176752352757344932[145] = 0;
   out_176752352757344932[146] = 0;
   out_176752352757344932[147] = 0;
   out_176752352757344932[148] = 0;
   out_176752352757344932[149] = 0;
   out_176752352757344932[150] = 0;
   out_176752352757344932[151] = 0;
   out_176752352757344932[152] = 1;
   out_176752352757344932[153] = 0;
   out_176752352757344932[154] = 0;
   out_176752352757344932[155] = 0;
   out_176752352757344932[156] = 0;
   out_176752352757344932[157] = 0;
   out_176752352757344932[158] = 0;
   out_176752352757344932[159] = 0;
   out_176752352757344932[160] = 0;
   out_176752352757344932[161] = 0;
   out_176752352757344932[162] = 0;
   out_176752352757344932[163] = 0;
   out_176752352757344932[164] = 0;
   out_176752352757344932[165] = 0;
   out_176752352757344932[166] = 0;
   out_176752352757344932[167] = 0;
   out_176752352757344932[168] = 0;
   out_176752352757344932[169] = 0;
   out_176752352757344932[170] = 0;
   out_176752352757344932[171] = 1;
   out_176752352757344932[172] = 0;
   out_176752352757344932[173] = 0;
   out_176752352757344932[174] = 0;
   out_176752352757344932[175] = 0;
   out_176752352757344932[176] = 0;
   out_176752352757344932[177] = 0;
   out_176752352757344932[178] = 0;
   out_176752352757344932[179] = 0;
   out_176752352757344932[180] = 0;
   out_176752352757344932[181] = 0;
   out_176752352757344932[182] = 0;
   out_176752352757344932[183] = 0;
   out_176752352757344932[184] = 0;
   out_176752352757344932[185] = 0;
   out_176752352757344932[186] = 0;
   out_176752352757344932[187] = 0;
   out_176752352757344932[188] = 0;
   out_176752352757344932[189] = 0;
   out_176752352757344932[190] = 1;
   out_176752352757344932[191] = 0;
   out_176752352757344932[192] = 0;
   out_176752352757344932[193] = 0;
   out_176752352757344932[194] = 0;
   out_176752352757344932[195] = 0;
   out_176752352757344932[196] = 0;
   out_176752352757344932[197] = 0;
   out_176752352757344932[198] = 0;
   out_176752352757344932[199] = 0;
   out_176752352757344932[200] = 0;
   out_176752352757344932[201] = 0;
   out_176752352757344932[202] = 0;
   out_176752352757344932[203] = 0;
   out_176752352757344932[204] = 0;
   out_176752352757344932[205] = 0;
   out_176752352757344932[206] = 0;
   out_176752352757344932[207] = 0;
   out_176752352757344932[208] = 0;
   out_176752352757344932[209] = 1;
   out_176752352757344932[210] = 0;
   out_176752352757344932[211] = 0;
   out_176752352757344932[212] = 0;
   out_176752352757344932[213] = 0;
   out_176752352757344932[214] = 0;
   out_176752352757344932[215] = 0;
   out_176752352757344932[216] = 0;
   out_176752352757344932[217] = 0;
   out_176752352757344932[218] = 0;
   out_176752352757344932[219] = 0;
   out_176752352757344932[220] = 0;
   out_176752352757344932[221] = 0;
   out_176752352757344932[222] = 0;
   out_176752352757344932[223] = 0;
   out_176752352757344932[224] = 0;
   out_176752352757344932[225] = 0;
   out_176752352757344932[226] = 0;
   out_176752352757344932[227] = 0;
   out_176752352757344932[228] = 1;
   out_176752352757344932[229] = 0;
   out_176752352757344932[230] = 0;
   out_176752352757344932[231] = 0;
   out_176752352757344932[232] = 0;
   out_176752352757344932[233] = 0;
   out_176752352757344932[234] = 0;
   out_176752352757344932[235] = 0;
   out_176752352757344932[236] = 0;
   out_176752352757344932[237] = 0;
   out_176752352757344932[238] = 0;
   out_176752352757344932[239] = 0;
   out_176752352757344932[240] = 0;
   out_176752352757344932[241] = 0;
   out_176752352757344932[242] = 0;
   out_176752352757344932[243] = 0;
   out_176752352757344932[244] = 0;
   out_176752352757344932[245] = 0;
   out_176752352757344932[246] = 0;
   out_176752352757344932[247] = 1;
   out_176752352757344932[248] = 0;
   out_176752352757344932[249] = 0;
   out_176752352757344932[250] = 0;
   out_176752352757344932[251] = 0;
   out_176752352757344932[252] = 0;
   out_176752352757344932[253] = 0;
   out_176752352757344932[254] = 0;
   out_176752352757344932[255] = 0;
   out_176752352757344932[256] = 0;
   out_176752352757344932[257] = 0;
   out_176752352757344932[258] = 0;
   out_176752352757344932[259] = 0;
   out_176752352757344932[260] = 0;
   out_176752352757344932[261] = 0;
   out_176752352757344932[262] = 0;
   out_176752352757344932[263] = 0;
   out_176752352757344932[264] = 0;
   out_176752352757344932[265] = 0;
   out_176752352757344932[266] = 1;
   out_176752352757344932[267] = 0;
   out_176752352757344932[268] = 0;
   out_176752352757344932[269] = 0;
   out_176752352757344932[270] = 0;
   out_176752352757344932[271] = 0;
   out_176752352757344932[272] = 0;
   out_176752352757344932[273] = 0;
   out_176752352757344932[274] = 0;
   out_176752352757344932[275] = 0;
   out_176752352757344932[276] = 0;
   out_176752352757344932[277] = 0;
   out_176752352757344932[278] = 0;
   out_176752352757344932[279] = 0;
   out_176752352757344932[280] = 0;
   out_176752352757344932[281] = 0;
   out_176752352757344932[282] = 0;
   out_176752352757344932[283] = 0;
   out_176752352757344932[284] = 0;
   out_176752352757344932[285] = 1;
   out_176752352757344932[286] = 0;
   out_176752352757344932[287] = 0;
   out_176752352757344932[288] = 0;
   out_176752352757344932[289] = 0;
   out_176752352757344932[290] = 0;
   out_176752352757344932[291] = 0;
   out_176752352757344932[292] = 0;
   out_176752352757344932[293] = 0;
   out_176752352757344932[294] = 0;
   out_176752352757344932[295] = 0;
   out_176752352757344932[296] = 0;
   out_176752352757344932[297] = 0;
   out_176752352757344932[298] = 0;
   out_176752352757344932[299] = 0;
   out_176752352757344932[300] = 0;
   out_176752352757344932[301] = 0;
   out_176752352757344932[302] = 0;
   out_176752352757344932[303] = 0;
   out_176752352757344932[304] = 1;
   out_176752352757344932[305] = 0;
   out_176752352757344932[306] = 0;
   out_176752352757344932[307] = 0;
   out_176752352757344932[308] = 0;
   out_176752352757344932[309] = 0;
   out_176752352757344932[310] = 0;
   out_176752352757344932[311] = 0;
   out_176752352757344932[312] = 0;
   out_176752352757344932[313] = 0;
   out_176752352757344932[314] = 0;
   out_176752352757344932[315] = 0;
   out_176752352757344932[316] = 0;
   out_176752352757344932[317] = 0;
   out_176752352757344932[318] = 0;
   out_176752352757344932[319] = 0;
   out_176752352757344932[320] = 0;
   out_176752352757344932[321] = 0;
   out_176752352757344932[322] = 0;
   out_176752352757344932[323] = 1;
}
void h_4(double *state, double *unused, double *out_7116964093078894538) {
   out_7116964093078894538[0] = state[6] + state[9];
   out_7116964093078894538[1] = state[7] + state[10];
   out_7116964093078894538[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_7199076691855356903) {
   out_7199076691855356903[0] = 0;
   out_7199076691855356903[1] = 0;
   out_7199076691855356903[2] = 0;
   out_7199076691855356903[3] = 0;
   out_7199076691855356903[4] = 0;
   out_7199076691855356903[5] = 0;
   out_7199076691855356903[6] = 1;
   out_7199076691855356903[7] = 0;
   out_7199076691855356903[8] = 0;
   out_7199076691855356903[9] = 1;
   out_7199076691855356903[10] = 0;
   out_7199076691855356903[11] = 0;
   out_7199076691855356903[12] = 0;
   out_7199076691855356903[13] = 0;
   out_7199076691855356903[14] = 0;
   out_7199076691855356903[15] = 0;
   out_7199076691855356903[16] = 0;
   out_7199076691855356903[17] = 0;
   out_7199076691855356903[18] = 0;
   out_7199076691855356903[19] = 0;
   out_7199076691855356903[20] = 0;
   out_7199076691855356903[21] = 0;
   out_7199076691855356903[22] = 0;
   out_7199076691855356903[23] = 0;
   out_7199076691855356903[24] = 0;
   out_7199076691855356903[25] = 1;
   out_7199076691855356903[26] = 0;
   out_7199076691855356903[27] = 0;
   out_7199076691855356903[28] = 1;
   out_7199076691855356903[29] = 0;
   out_7199076691855356903[30] = 0;
   out_7199076691855356903[31] = 0;
   out_7199076691855356903[32] = 0;
   out_7199076691855356903[33] = 0;
   out_7199076691855356903[34] = 0;
   out_7199076691855356903[35] = 0;
   out_7199076691855356903[36] = 0;
   out_7199076691855356903[37] = 0;
   out_7199076691855356903[38] = 0;
   out_7199076691855356903[39] = 0;
   out_7199076691855356903[40] = 0;
   out_7199076691855356903[41] = 0;
   out_7199076691855356903[42] = 0;
   out_7199076691855356903[43] = 0;
   out_7199076691855356903[44] = 1;
   out_7199076691855356903[45] = 0;
   out_7199076691855356903[46] = 0;
   out_7199076691855356903[47] = 1;
   out_7199076691855356903[48] = 0;
   out_7199076691855356903[49] = 0;
   out_7199076691855356903[50] = 0;
   out_7199076691855356903[51] = 0;
   out_7199076691855356903[52] = 0;
   out_7199076691855356903[53] = 0;
}
void h_10(double *state, double *unused, double *out_1609865709454132551) {
   out_1609865709454132551[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_1609865709454132551[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_1609865709454132551[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_7711464682056337641) {
   out_7711464682056337641[0] = 0;
   out_7711464682056337641[1] = 9.8100000000000005*cos(state[1]);
   out_7711464682056337641[2] = 0;
   out_7711464682056337641[3] = 0;
   out_7711464682056337641[4] = -state[8];
   out_7711464682056337641[5] = state[7];
   out_7711464682056337641[6] = 0;
   out_7711464682056337641[7] = state[5];
   out_7711464682056337641[8] = -state[4];
   out_7711464682056337641[9] = 0;
   out_7711464682056337641[10] = 0;
   out_7711464682056337641[11] = 0;
   out_7711464682056337641[12] = 1;
   out_7711464682056337641[13] = 0;
   out_7711464682056337641[14] = 0;
   out_7711464682056337641[15] = 1;
   out_7711464682056337641[16] = 0;
   out_7711464682056337641[17] = 0;
   out_7711464682056337641[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_7711464682056337641[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_7711464682056337641[20] = 0;
   out_7711464682056337641[21] = state[8];
   out_7711464682056337641[22] = 0;
   out_7711464682056337641[23] = -state[6];
   out_7711464682056337641[24] = -state[5];
   out_7711464682056337641[25] = 0;
   out_7711464682056337641[26] = state[3];
   out_7711464682056337641[27] = 0;
   out_7711464682056337641[28] = 0;
   out_7711464682056337641[29] = 0;
   out_7711464682056337641[30] = 0;
   out_7711464682056337641[31] = 1;
   out_7711464682056337641[32] = 0;
   out_7711464682056337641[33] = 0;
   out_7711464682056337641[34] = 1;
   out_7711464682056337641[35] = 0;
   out_7711464682056337641[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_7711464682056337641[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_7711464682056337641[38] = 0;
   out_7711464682056337641[39] = -state[7];
   out_7711464682056337641[40] = state[6];
   out_7711464682056337641[41] = 0;
   out_7711464682056337641[42] = state[4];
   out_7711464682056337641[43] = -state[3];
   out_7711464682056337641[44] = 0;
   out_7711464682056337641[45] = 0;
   out_7711464682056337641[46] = 0;
   out_7711464682056337641[47] = 0;
   out_7711464682056337641[48] = 0;
   out_7711464682056337641[49] = 0;
   out_7711464682056337641[50] = 1;
   out_7711464682056337641[51] = 0;
   out_7711464682056337641[52] = 0;
   out_7711464682056337641[53] = 1;
}
void h_13(double *state, double *unused, double *out_3359689799482185332) {
   out_3359689799482185332[0] = state[3];
   out_3359689799482185332[1] = state[4];
   out_3359689799482185332[2] = state[5];
}
void H_13(double *state, double *unused, double *out_8035393556521861912) {
   out_8035393556521861912[0] = 0;
   out_8035393556521861912[1] = 0;
   out_8035393556521861912[2] = 0;
   out_8035393556521861912[3] = 1;
   out_8035393556521861912[4] = 0;
   out_8035393556521861912[5] = 0;
   out_8035393556521861912[6] = 0;
   out_8035393556521861912[7] = 0;
   out_8035393556521861912[8] = 0;
   out_8035393556521861912[9] = 0;
   out_8035393556521861912[10] = 0;
   out_8035393556521861912[11] = 0;
   out_8035393556521861912[12] = 0;
   out_8035393556521861912[13] = 0;
   out_8035393556521861912[14] = 0;
   out_8035393556521861912[15] = 0;
   out_8035393556521861912[16] = 0;
   out_8035393556521861912[17] = 0;
   out_8035393556521861912[18] = 0;
   out_8035393556521861912[19] = 0;
   out_8035393556521861912[20] = 0;
   out_8035393556521861912[21] = 0;
   out_8035393556521861912[22] = 1;
   out_8035393556521861912[23] = 0;
   out_8035393556521861912[24] = 0;
   out_8035393556521861912[25] = 0;
   out_8035393556521861912[26] = 0;
   out_8035393556521861912[27] = 0;
   out_8035393556521861912[28] = 0;
   out_8035393556521861912[29] = 0;
   out_8035393556521861912[30] = 0;
   out_8035393556521861912[31] = 0;
   out_8035393556521861912[32] = 0;
   out_8035393556521861912[33] = 0;
   out_8035393556521861912[34] = 0;
   out_8035393556521861912[35] = 0;
   out_8035393556521861912[36] = 0;
   out_8035393556521861912[37] = 0;
   out_8035393556521861912[38] = 0;
   out_8035393556521861912[39] = 0;
   out_8035393556521861912[40] = 0;
   out_8035393556521861912[41] = 1;
   out_8035393556521861912[42] = 0;
   out_8035393556521861912[43] = 0;
   out_8035393556521861912[44] = 0;
   out_8035393556521861912[45] = 0;
   out_8035393556521861912[46] = 0;
   out_8035393556521861912[47] = 0;
   out_8035393556521861912[48] = 0;
   out_8035393556521861912[49] = 0;
   out_8035393556521861912[50] = 0;
   out_8035393556521861912[51] = 0;
   out_8035393556521861912[52] = 0;
   out_8035393556521861912[53] = 0;
}
void h_14(double *state, double *unused, double *out_5734177569564106995) {
   out_5734177569564106995[0] = state[6];
   out_5734177569564106995[1] = state[7];
   out_5734177569564106995[2] = state[8];
}
void H_14(double *state, double *unused, double *out_7284426525514710184) {
   out_7284426525514710184[0] = 0;
   out_7284426525514710184[1] = 0;
   out_7284426525514710184[2] = 0;
   out_7284426525514710184[3] = 0;
   out_7284426525514710184[4] = 0;
   out_7284426525514710184[5] = 0;
   out_7284426525514710184[6] = 1;
   out_7284426525514710184[7] = 0;
   out_7284426525514710184[8] = 0;
   out_7284426525514710184[9] = 0;
   out_7284426525514710184[10] = 0;
   out_7284426525514710184[11] = 0;
   out_7284426525514710184[12] = 0;
   out_7284426525514710184[13] = 0;
   out_7284426525514710184[14] = 0;
   out_7284426525514710184[15] = 0;
   out_7284426525514710184[16] = 0;
   out_7284426525514710184[17] = 0;
   out_7284426525514710184[18] = 0;
   out_7284426525514710184[19] = 0;
   out_7284426525514710184[20] = 0;
   out_7284426525514710184[21] = 0;
   out_7284426525514710184[22] = 0;
   out_7284426525514710184[23] = 0;
   out_7284426525514710184[24] = 0;
   out_7284426525514710184[25] = 1;
   out_7284426525514710184[26] = 0;
   out_7284426525514710184[27] = 0;
   out_7284426525514710184[28] = 0;
   out_7284426525514710184[29] = 0;
   out_7284426525514710184[30] = 0;
   out_7284426525514710184[31] = 0;
   out_7284426525514710184[32] = 0;
   out_7284426525514710184[33] = 0;
   out_7284426525514710184[34] = 0;
   out_7284426525514710184[35] = 0;
   out_7284426525514710184[36] = 0;
   out_7284426525514710184[37] = 0;
   out_7284426525514710184[38] = 0;
   out_7284426525514710184[39] = 0;
   out_7284426525514710184[40] = 0;
   out_7284426525514710184[41] = 0;
   out_7284426525514710184[42] = 0;
   out_7284426525514710184[43] = 0;
   out_7284426525514710184[44] = 1;
   out_7284426525514710184[45] = 0;
   out_7284426525514710184[46] = 0;
   out_7284426525514710184[47] = 0;
   out_7284426525514710184[48] = 0;
   out_7284426525514710184[49] = 0;
   out_7284426525514710184[50] = 0;
   out_7284426525514710184[51] = 0;
   out_7284426525514710184[52] = 0;
   out_7284426525514710184[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_5378340430612614676) {
  err_fun(nom_x, delta_x, out_5378340430612614676);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_609280258945418386) {
  inv_err_fun(nom_x, true_x, out_609280258945418386);
}
void pose_H_mod_fun(double *state, double *out_2046558306816276168) {
  H_mod_fun(state, out_2046558306816276168);
}
void pose_f_fun(double *state, double dt, double *out_5264433078543911283) {
  f_fun(state,  dt, out_5264433078543911283);
}
void pose_F_fun(double *state, double dt, double *out_176752352757344932) {
  F_fun(state,  dt, out_176752352757344932);
}
void pose_h_4(double *state, double *unused, double *out_7116964093078894538) {
  h_4(state, unused, out_7116964093078894538);
}
void pose_H_4(double *state, double *unused, double *out_7199076691855356903) {
  H_4(state, unused, out_7199076691855356903);
}
void pose_h_10(double *state, double *unused, double *out_1609865709454132551) {
  h_10(state, unused, out_1609865709454132551);
}
void pose_H_10(double *state, double *unused, double *out_7711464682056337641) {
  H_10(state, unused, out_7711464682056337641);
}
void pose_h_13(double *state, double *unused, double *out_3359689799482185332) {
  h_13(state, unused, out_3359689799482185332);
}
void pose_H_13(double *state, double *unused, double *out_8035393556521861912) {
  H_13(state, unused, out_8035393556521861912);
}
void pose_h_14(double *state, double *unused, double *out_5734177569564106995) {
  h_14(state, unused, out_5734177569564106995);
}
void pose_H_14(double *state, double *unused, double *out_7284426525514710184) {
  H_14(state, unused, out_7284426525514710184);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
