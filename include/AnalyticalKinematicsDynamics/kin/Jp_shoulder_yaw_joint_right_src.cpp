/*
 * Automatically Generated from Mathematica.
 * Mon 4 Jul 2022 20:55:26 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_shoulder_yaw_joint_right_src.h"

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE inline        /* use standard inline */
#endif

/**
 * Copied from Wolfram Mathematica C Definitions file mdefs.hpp
 * Changed marcos to inline functions (Eric Cousineau)
 */
INLINE double Power(double x, double y) { return pow(x, y); }
INLINE double Sqrt(double x) { return sqrt(x); }

INLINE double Abs(double x) { return fabs(x); }

INLINE double Exp(double x) { return exp(x); }
INLINE double Log(double x) { return log(x); }

INLINE double Sin(double x) { return sin(x); }
INLINE double Cos(double x) { return cos(x); }
INLINE double Tan(double x) { return tan(x); }

INLINE double Csc(double x) { return 1.0/sin(x); }
INLINE double Sec(double x) { return 1.0/cos(x); }

INLINE double ArcSin(double x) { return asin(x); }
INLINE double ArcCos(double x) { return acos(x); }
//INLINE double ArcTan(double x) { return atan(x); }

/* update ArcTan function to use atan2 instead. */
INLINE double ArcTan(double x, double y) { return atan2(y,x); }

INLINE double Sinh(double x) { return sinh(x); }
INLINE double Cosh(double x) { return cosh(x); }
INLINE double Tanh(double x) { return tanh(x); }

#define E 2.71828182845904523536029
#define Pi 3.14159265358979323846264
#define Degree 0.01745329251994329576924

/*
 * Sub functions
 */
static void output1(double *p_output1,const double *var1)
{
  double t275;
  double t682;
  double t757;
  double t816;
  double t885;
  double t1198;
  double t1387;
  double t1304;
  double t1332;
  double t1404;
  double t88;
  double t97;
  double t122;
  double t1378;
  double t1410;
  double t1430;
  double t1591;
  double t1622;
  double t1680;
  double t2906;
  double t2977;
  double t3051;
  double t3157;
  double t67;
  double t2663;
  double t2691;
  double t2436;
  double t2496;
  double t2595;
  double t2748;
  double t2772;
  double t2847;
  double t3138;
  double t1954;
  double t2159;
  double t3573;
  double t3270;
  double t312;
  double t320;
  double t492;
  double t547;
  double t3930;
  double t3308;
  double t3316;
  double t3450;
  double t3451;
  double t3486;
  double t3503;
  double t3845;
  double t3739;
  double t3740;
  double t3744;
  double t3757;
  double t3767;
  double t3768;
  double t4012;
  double t4074;
  double t4086;
  double t4088;
  double t4104;
  double t4126;
  double t3177;
  double t4174;
  double t3299;
  double t4475;
  double t3706;
  double t4338;
  double t3592;
  double t125;
  double t437;
  double t623;
  double t651;
  double t1129;
  double t1258;
  double t1265;
  double t1515;
  double t1535;
  double t1588;
  double t1885;
  double t1946;
  double t2208;
  double t2227;
  double t5001;
  double t5025;
  double t5035;
  double t5045;
  double t5046;
  double t5050;
  double t2645;
  double t2646;
  double t2701;
  double t2746;
  double t2885;
  double t3120;
  double t3215;
  double t3256;
  double t3301;
  double t3302;
  double t3304;
  double t5122;
  double t5128;
  double t5132;
  double t5148;
  double t5152;
  double t5157;
  double t3517;
  double t3542;
  double t3629;
  double t3641;
  double t3718;
  double t3737;
  double t3738;
  double t3783;
  double t3816;
  double t3887;
  double t3890;
  double t3961;
  double t3994;
  double t4010;
  double t4178;
  double t5194;
  double t5195;
  double t5209;
  double t5232;
  double t4351;
  double t5289;
  double t5290;
  double t5295;
  double t5299;
  double t4377;
  double t4391;
  double t5306;
  double t5308;
  double t5324;
  double t5333;
  double t4454;
  double t4461;
  double t4529;
  double t4606;
  double t4714;
  double t4719;
  double t4766;
  double t4779;
  double t5522;
  double t5527;
  double t5542;
  double t5546;
  double t5547;
  double t5555;
  double t5571;
  double t5573;
  double t5587;
  double t5588;
  double t5613;
  double t5615;
  double t5623;
  double t5628;
  double t5630;
  double t5634;
  double t5635;
  double t5637;
  double t5769;
  double t5771;
  double t5772;
  double t5790;
  double t5791;
  double t5799;
  double t5808;
  double t5820;
  double t5823;
  double t5827;
  double t5832;
  double t5842;
  double t5847;
  double t5855;
  double t5858;
  double t5860;
  double t5863;
  double t5865;
  double t5976;
  double t5978;
  double t5982;
  double t5987;
  double t5990;
  double t5994;
  double t6010;
  double t6012;
  double t6014;
  double t6017;
  double t6024;
  double t6030;
  double t6032;
  double t6035;
  double t6038;
  double t6039;
  double t6040;
  double t6041;
  double t6102;
  double t6115;
  double t6118;
  double t6122;
  double t6125;
  double t6126;
  double t6135;
  double t6136;
  double t6142;
  double t6143;
  double t6144;
  double t6146;
  double t6156;
  double t6157;
  double t6173;
  double t6174;
  double t6177;
  double t6236;
  double t6237;
  double t6240;
  double t6248;
  double t6249;
  double t6251;
  double t6252;
  double t6254;
  double t6258;
  double t6263;
  double t6264;
  double t6268;
  double t6271;
  double t6272;
  double t6276;
  double t6279;
  double t6280;
  double t6325;
  double t6329;
  double t6330;
  double t6335;
  double t6336;
  double t6338;
  double t6341;
  double t6349;
  double t6352;
  double t6354;
  double t6355;
  double t6360;
  double t6368;
  double t6369;
  double t6370;
  double t6440;
  double t6441;
  double t6442;
  double t6452;
  double t6455;
  double t6462;
  double t6463;
  double t6464;
  double t6467;
  double t6468;
  double t6469;
  double t6418;
  double t6420;
  double t6422;
  double t6424;
  double t6430;
  double t6507;
  double t6510;
  double t6516;
  double t6522;
  double t6523;
  double t6524;
  double t6527;
  double t6528;
  double t6534;
  double t6537;
  double t6539;
  double t6541;
  double t6543;
  double t6546;
  double t6547;
  double t6550;
  double t6551;
  double t6552;
  double t6597;
  double t6598;
  double t6599;
  double t6602;
  double t6603;
  double t6604;
  double t6606;
  double t6607;
  double t6608;
  double t6613;
  double t6616;
  double t6621;
  double t6623;
  double t6627;
  double t6628;
  double t6697;
  double t6699;
  double t6700;
  double t6701;
  double t6703;
  double t6704;
  double t6708;
  double t6710;
  double t6715;
  double t6717;
  double t6725;
  double t6730;
  double t6670;
  double t6671;
  double t6672;
  double t6678;
  double t6679;
  double t6680;
  double t6687;
  double t6690;
  double t6693;
  double t6763;
  double t6765;
  double t6770;
  double t6775;
  double t6777;
  double t6778;
  double t6783;
  double t6787;
  double t6788;
  double t6794;
  double t6796;
  double t6797;
  double t6798;
  double t6799;
  double t6826;
  double t6828;
  double t6829;
  double t6830;
  double t6832;
  double t6834;
  double t6835;
  double t6837;
  double t6839;
  double t6840;
  double t6841;
  double t6844;
  double t6871;
  double t6889;
  double t6875;
  double t6870;
  double t6874;
  double t6882;
  double t6907;
  double t6888;
  double t6898;
  double t6911;
  double t6895;
  double t6924;
  double t6872;
  double t6873;
  double t6876;
  double t6877;
  double t6879;
  double t6880;
  double t6886;
  double t6887;
  double t6890;
  double t6891;
  double t6892;
  double t6893;
  double t6896;
  double t6897;
  double t6900;
  double t6901;
  double t6902;
  double t6903;
  double t6946;
  double t6948;
  double t6949;
  double t6909;
  double t6951;
  double t6952;
  double t6953;
  double t6954;
  double t6913;
  double t6956;
  double t6958;
  double t6962;
  double t6963;
  double t6919;
  double t6926;
  double t6934;
  double t6938;
  double t6998;
  double t7008;
  double t7009;
  double t7011;
  double t7012;
  double t7014;
  double t7016;
  double t7018;
  double t7019;
  double t7020;
  t275 = Sin(var1[25]);
  t682 = Sin(var1[3]);
  t757 = Cos(var1[24]);
  t816 = -1.*t757;
  t885 = 1. + t816;
  t1198 = Sin(var1[24]);
  t1387 = Cos(var1[3]);
  t1304 = Cos(var1[5]);
  t1332 = Sin(var1[4]);
  t1404 = Sin(var1[5]);
  t88 = Cos(var1[25]);
  t97 = -1.*t88;
  t122 = 1. + t97;
  t1378 = -1.*t1304*t682*t1332;
  t1410 = t1387*t1404;
  t1430 = t1378 + t1410;
  t1591 = -1.*t1387*t1304;
  t1622 = -1.*t682*t1332*t1404;
  t1680 = t1591 + t1622;
  t2906 = Cos(var1[26]);
  t2977 = -1.*t2906;
  t3051 = 1. + t2977;
  t3157 = Sin(var1[26]);
  t67 = Cos(var1[4]);
  t2663 = -0.994522*t275;
  t2691 = 0. + t2663;
  t2436 = -1.*t1198*t1430;
  t2496 = t757*t1680;
  t2595 = t2436 + t2496;
  t2748 = t757*t1430;
  t2772 = t1198*t1680;
  t2847 = t2748 + t2772;
  t3138 = -0.051978134642000004*t3051;
  t1954 = -0.104528*t275;
  t2159 = 0. + t1954;
  t3573 = 0.05226439969100001*t3051;
  t3270 = 0.49726168403800003*t3051;
  t312 = 0.104528*t275;
  t320 = 0. + t312;
  t492 = 0.994522*t275;
  t547 = 0. + t492;
  t3930 = 0.073913*t3157;
  t3308 = -1.*t67*t2691*t682;
  t3316 = -0.103955395616*t122*t2595;
  t3450 = -0.9890740084840001*t122;
  t3451 = 1. + t3450;
  t3486 = t3451*t2847;
  t3503 = t3308 + t3316 + t3486;
  t3845 = -0.703234*t3157;
  t3739 = -1.*t67*t2159*t682;
  t3740 = -0.010926102783999999*t122;
  t3744 = 1. + t3740;
  t3757 = t3744*t2595;
  t3767 = -0.103955395616*t122*t2847;
  t3768 = t3739 + t3757 + t3767;
  t4012 = -1.0000001112680001*t122;
  t4074 = 1. + t4012;
  t4086 = -1.*t4074*t67*t682;
  t4088 = t320*t2595;
  t4104 = t547*t2847;
  t4126 = t4086 + t4088 + t4104;
  t3177 = -0.707107*t3157;
  t4174 = -0.49726168403800003*t3051;
  t3299 = -0.073913*t3157;
  t4475 = 0.051978134642000004*t3051;
  t3706 = 0.707107*t3157;
  t4338 = -0.05226439969100001*t3051;
  t3592 = 0.703234*t3157;
  t125 = -0.056500534356700764*t122;
  t437 = 0.040271188976*t320;
  t623 = 0.38315650737400003*t547;
  t651 = 0. + t125 + t437 + t623;
  t1129 = 0.4*t885;
  t1258 = -0.12*t1198;
  t1265 = 0. + t1129 + t1258;
  t1515 = -0.12*t885;
  t1535 = -0.4*t1198;
  t1588 = 0. + t1515 + t1535;
  t1885 = 1.1345904784751044e-7*var1[25];
  t1946 = 0.04027119345689465*t122;
  t2208 = -0.05650052807*t2159;
  t2227 = t1885 + t1946 + t2208;
  t5001 = t1387*t1304*t1332;
  t5025 = t682*t1404;
  t5035 = t5001 + t5025;
  t5045 = -1.*t1304*t682;
  t5046 = t1387*t1332*t1404;
  t5050 = t5045 + t5046;
  t2645 = -1.1924972351948546e-8*var1[25];
  t2646 = 0.38315655000705834*t122;
  t2701 = -0.05650052807*t2691;
  t2746 = t2645 + t2646 + t2701;
  t2885 = 1.5601527583902087e-7*var1[26];
  t3120 = 0.09582494577057615*t3051;
  t3215 = t3138 + t3177;
  t3256 = -0.231098203479*t3215;
  t3301 = t3270 + t3299;
  t3302 = 0.164383620275*t3301;
  t3304 = t2885 + t3120 + t3256 + t3302;
  t5122 = -1.*t1198*t5035;
  t5128 = t757*t5050;
  t5132 = t5122 + t5128;
  t5148 = t757*t5035;
  t5152 = t1198*t5050;
  t5157 = t5148 + t5152;
  t3517 = 1.639789470231751e-8*var1[26];
  t3542 = -0.22983603018311177*t3051;
  t3629 = t3573 + t3592;
  t3641 = 0.164383620275*t3629;
  t3718 = t3138 + t3706;
  t3737 = 0.18957839082800002*t3718;
  t3738 = t3517 + t3542 + t3641 + t3737;
  t3783 = -1.568745163810375e-7*var1[26];
  t3816 = 0.08219200580743281*t3051;
  t3887 = t3573 + t3845;
  t3890 = -0.231098203479*t3887;
  t3961 = t3270 + t3930;
  t3994 = 0.18957839082800002*t3961;
  t4010 = t3783 + t3816 + t3890 + t3994;
  t4178 = t4174 + t3930;
  t5194 = t1387*t67*t2691;
  t5195 = -0.103955395616*t122*t5132;
  t5209 = t3451*t5157;
  t5232 = t5194 + t5195 + t5209;
  t4351 = t4338 + t3845;
  t5289 = t1387*t67*t2159;
  t5290 = t3744*t5132;
  t5295 = -0.103955395616*t122*t5157;
  t5299 = t5289 + t5290 + t5295;
  t4377 = -0.500001190325*t3051;
  t4391 = 1. + t4377;
  t5306 = t4074*t1387*t67;
  t5308 = t320*t5132;
  t5324 = t547*t5157;
  t5333 = t5306 + t5308 + t5324;
  t4454 = -0.5054634410180001*t3051;
  t4461 = 1. + t4454;
  t4529 = t4475 + t3177;
  t4606 = t4174 + t3299;
  t4714 = t4475 + t3706;
  t4719 = -0.9945383682050002*t3051;
  t4766 = 1. + t4719;
  t4779 = t4338 + t3592;
  t5522 = -1.*t1387*t67*t1304*t1198;
  t5527 = t757*t1387*t67*t1404;
  t5542 = t5522 + t5527;
  t5546 = t757*t1387*t67*t1304;
  t5547 = t1387*t67*t1198*t1404;
  t5555 = t5546 + t5547;
  t5571 = -1.*t1387*t2691*t1332;
  t5573 = -0.103955395616*t122*t5542;
  t5587 = t3451*t5555;
  t5588 = t5571 + t5573 + t5587;
  t5613 = -1.*t1387*t2159*t1332;
  t5615 = t3744*t5542;
  t5623 = -0.103955395616*t122*t5555;
  t5628 = t5613 + t5615 + t5623;
  t5630 = -1.*t4074*t1387*t1332;
  t5634 = t320*t5542;
  t5635 = t547*t5555;
  t5637 = t5630 + t5634 + t5635;
  t5769 = -1.*t67*t1304*t1198*t682;
  t5771 = t757*t67*t682*t1404;
  t5772 = t5769 + t5771;
  t5790 = t757*t67*t1304*t682;
  t5791 = t67*t1198*t682*t1404;
  t5799 = t5790 + t5791;
  t5808 = -1.*t2691*t682*t1332;
  t5820 = -0.103955395616*t122*t5772;
  t5823 = t3451*t5799;
  t5827 = t5808 + t5820 + t5823;
  t5832 = -1.*t2159*t682*t1332;
  t5842 = t3744*t5772;
  t5847 = -0.103955395616*t122*t5799;
  t5855 = t5832 + t5842 + t5847;
  t5858 = -1.*t4074*t682*t1332;
  t5860 = t320*t5772;
  t5863 = t547*t5799;
  t5865 = t5858 + t5860 + t5863;
  t5976 = t1304*t1198*t1332;
  t5978 = -1.*t757*t1332*t1404;
  t5982 = t5976 + t5978;
  t5987 = -1.*t757*t1304*t1332;
  t5990 = -1.*t1198*t1332*t1404;
  t5994 = t5987 + t5990;
  t6010 = -1.*t67*t2691;
  t6012 = -0.103955395616*t122*t5982;
  t6014 = t3451*t5994;
  t6017 = t6010 + t6012 + t6014;
  t6024 = -1.*t67*t2159;
  t6030 = t3744*t5982;
  t6032 = -0.103955395616*t122*t5994;
  t6035 = t6024 + t6030 + t6032;
  t6038 = -1.*t4074*t67;
  t6039 = t320*t5982;
  t6040 = t547*t5994;
  t6041 = t6038 + t6039 + t6040;
  t6102 = t1304*t682;
  t6115 = -1.*t1387*t1332*t1404;
  t6118 = t6102 + t6115;
  t6122 = t1198*t5035;
  t6125 = t757*t6118;
  t6126 = t6122 + t6125;
  t6135 = -1.*t1198*t6118;
  t6136 = t5148 + t6135;
  t6142 = -0.103955395616*t122*t6126;
  t6143 = t3744*t6136;
  t6144 = t6142 + t6143;
  t6146 = t3451*t6126;
  t6156 = -0.103955395616*t122*t6136;
  t6157 = t6146 + t6156;
  t6173 = t547*t6126;
  t6174 = t320*t6136;
  t6177 = t6173 + t6174;
  t6236 = t1304*t682*t1332;
  t6237 = -1.*t1387*t1404;
  t6240 = t6236 + t6237;
  t6248 = t1198*t6240;
  t6249 = t6248 + t2496;
  t6251 = t757*t6240;
  t6252 = -1.*t1198*t1680;
  t6254 = t6251 + t6252;
  t6258 = -0.103955395616*t122*t6249;
  t6263 = t3744*t6254;
  t6264 = t6258 + t6263;
  t6268 = t3451*t6249;
  t6271 = -0.103955395616*t122*t6254;
  t6272 = t6268 + t6271;
  t6276 = t547*t6249;
  t6279 = t320*t6254;
  t6280 = t6276 + t6279;
  t6325 = t67*t1304*t1198;
  t6329 = -1.*t757*t67*t1404;
  t6330 = t6325 + t6329;
  t6335 = t757*t67*t1304;
  t6336 = t67*t1198*t1404;
  t6338 = t6335 + t6336;
  t6341 = -0.103955395616*t122*t6330;
  t6349 = t3744*t6338;
  t6352 = t6341 + t6349;
  t6354 = t3451*t6330;
  t6355 = -0.103955395616*t122*t6338;
  t6360 = t6354 + t6355;
  t6368 = t547*t6330;
  t6369 = t320*t6338;
  t6370 = t6368 + t6369;
  t6440 = -1.*t757*t5035;
  t6441 = -1.*t1198*t5050;
  t6442 = t6440 + t6441;
  t6452 = t3744*t6442;
  t6455 = t5195 + t6452;
  t6462 = t3451*t5132;
  t6463 = -0.103955395616*t122*t6442;
  t6464 = t6462 + t6463;
  t6467 = t547*t5132;
  t6468 = t320*t6442;
  t6469 = t6467 + t6468;
  t6418 = -0.12*t757;
  t6420 = 0.4*t1198;
  t6422 = t6418 + t6420;
  t6424 = -0.4*t757;
  t6430 = t6424 + t1258;
  t6507 = t1387*t1304;
  t6510 = t682*t1332*t1404;
  t6516 = t6507 + t6510;
  t6522 = -1.*t1198*t6240;
  t6523 = t757*t6516;
  t6524 = t6522 + t6523;
  t6527 = -1.*t757*t6240;
  t6528 = -1.*t1198*t6516;
  t6534 = t6527 + t6528;
  t6537 = -0.103955395616*t122*t6524;
  t6539 = t3744*t6534;
  t6541 = t6537 + t6539;
  t6543 = t3451*t6524;
  t6546 = -0.103955395616*t122*t6534;
  t6547 = t6543 + t6546;
  t6550 = t547*t6524;
  t6551 = t320*t6534;
  t6552 = t6550 + t6551;
  t6597 = -1.*t67*t1304*t1198;
  t6598 = t757*t67*t1404;
  t6599 = t6597 + t6598;
  t6602 = -1.*t757*t67*t1304;
  t6603 = -1.*t67*t1198*t1404;
  t6604 = t6602 + t6603;
  t6606 = -0.103955395616*t122*t6599;
  t6607 = t3744*t6604;
  t6608 = t6606 + t6607;
  t6613 = t3451*t6599;
  t6616 = -0.103955395616*t122*t6604;
  t6621 = t6613 + t6616;
  t6623 = t547*t6599;
  t6627 = t320*t6604;
  t6628 = t6623 + t6627;
  t6697 = -1.0000001112680001*t1387*t67*t275;
  t6699 = 0.104528*t88*t5132;
  t6700 = 0.994522*t88*t5157;
  t6701 = t6697 + t6699 + t6700;
  t6703 = -0.994522*t88*t1387*t67;
  t6704 = -0.103955395616*t275*t5132;
  t6708 = -0.9890740084840001*t275*t5157;
  t6710 = t6703 + t6704 + t6708;
  t6715 = -0.104528*t88*t1387*t67;
  t6717 = -0.010926102783999999*t275*t5132;
  t6725 = -0.103955395616*t275*t5157;
  t6730 = t6715 + t6717 + t6725;
  t6670 = 0.3852670428678886*t88;
  t6671 = -0.056500534356700764*t275;
  t6672 = t6670 + t6671;
  t6678 = 0.0059058871981009595*t88;
  t6679 = 0.04027119345689465*t275;
  t6680 = 1.1345904784751044e-7 + t6678 + t6679;
  t6687 = 0.05619101817723254*t88;
  t6690 = 0.38315655000705834*t275;
  t6693 = -1.1924972351948546e-8 + t6687 + t6690;
  t6763 = t1198*t6516;
  t6765 = t6251 + t6763;
  t6770 = -1.0000001112680001*t67*t275*t682;
  t6775 = 0.104528*t88*t6524;
  t6777 = 0.994522*t88*t6765;
  t6778 = t6770 + t6775 + t6777;
  t6783 = -0.994522*t88*t67*t682;
  t6787 = -0.103955395616*t275*t6524;
  t6788 = -0.9890740084840001*t275*t6765;
  t6794 = t6783 + t6787 + t6788;
  t6796 = -0.104528*t88*t67*t682;
  t6797 = -0.010926102783999999*t275*t6524;
  t6798 = -0.103955395616*t275*t6765;
  t6799 = t6796 + t6797 + t6798;
  t6826 = 1.0000001112680001*t275*t1332;
  t6828 = 0.104528*t88*t6599;
  t6829 = 0.994522*t88*t6338;
  t6830 = t6826 + t6828 + t6829;
  t6832 = 0.994522*t88*t1332;
  t6834 = -0.103955395616*t275*t6599;
  t6835 = -0.9890740084840001*t275*t6338;
  t6837 = t6832 + t6834 + t6835;
  t6839 = 0.104528*t88*t1332;
  t6840 = -0.010926102783999999*t275*t6599;
  t6841 = -0.103955395616*t275*t6338;
  t6844 = t6839 + t6840 + t6841;
  t6871 = -0.051978134642000004*t3157;
  t6889 = 0.05226439969100001*t3157;
  t6875 = 0.49726168403800003*t3157;
  t6870 = -0.707107*t2906;
  t6874 = -0.073913*t2906;
  t6882 = 0.707107*t2906;
  t6907 = 0.051978134642000004*t3157;
  t6888 = 0.703234*t2906;
  t6898 = 0.073913*t2906;
  t6911 = -0.49726168403800003*t3157;
  t6895 = -0.703234*t2906;
  t6924 = -0.05226439969100001*t3157;
  t6872 = t6870 + t6871;
  t6873 = -0.231098203479*t6872;
  t6876 = t6874 + t6875;
  t6877 = 0.164383620275*t6876;
  t6879 = 0.09582494577057615*t3157;
  t6880 = 1.5601527583902087e-7 + t6873 + t6877 + t6879;
  t6886 = t6882 + t6871;
  t6887 = 0.18957839082800002*t6886;
  t6890 = t6888 + t6889;
  t6891 = 0.164383620275*t6890;
  t6892 = -0.22983603018311177*t3157;
  t6893 = 1.639789470231751e-8 + t6887 + t6891 + t6892;
  t6896 = t6895 + t6889;
  t6897 = -0.231098203479*t6896;
  t6900 = t6898 + t6875;
  t6901 = 0.18957839082800002*t6900;
  t6902 = 0.08219200580743281*t3157;
  t6903 = -1.568745163810375e-7 + t6897 + t6901 + t6902;
  t6946 = t67*t2691*t682;
  t6948 = t3451*t6765;
  t6949 = t6946 + t6537 + t6948;
  t6909 = t6870 + t6907;
  t6951 = t67*t2159*t682;
  t6952 = t3744*t6524;
  t6953 = -0.103955395616*t122*t6765;
  t6954 = t6951 + t6952 + t6953;
  t6913 = t6874 + t6911;
  t6956 = t4074*t67*t682;
  t6958 = t320*t6524;
  t6962 = t547*t6765;
  t6963 = t6956 + t6958 + t6962;
  t6919 = t6882 + t6907;
  t6926 = t6888 + t6924;
  t6934 = t6898 + t6911;
  t6938 = t6895 + t6924;
  t6998 = -1.*t2691*t1332;
  t7008 = t3451*t6338;
  t7009 = t6998 + t6606 + t7008;
  t7011 = -1.*t2159*t1332;
  t7012 = t3744*t6599;
  t7014 = t7011 + t7012 + t6355;
  t7016 = -1.*t4074*t1332;
  t7018 = t320*t6599;
  t7019 = t547*t6338;
  t7020 = t7016 + t7018 + t7019;
  p_output1[0]=1.;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=1.;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=1.;
  p_output1[9]=t1265*t1430 + t1588*t1680 + t2227*t2595 + t2746*t2847 + t3304*t3503 + t3738*t3768 + t4010*t4126 + 0.060173*(t3503*t4178 + t3768*t4351 + t4126*t4391) + 0.293218*(t3503*t4461 + t3768*t4529 + t4126*t4606) - 0.220205*(t3503*t4714 + t3768*t4766 + t4126*t4779) - 1.*t651*t67*t682;
  p_output1[10]=t1265*t5035 + t1588*t5050 + t2227*t5132 + t2746*t5157 + t3304*t5232 + t3738*t5299 + t4010*t5333 + 0.060173*(t4178*t5232 + t4351*t5299 + t4391*t5333) + 0.293218*(t4461*t5232 + t4529*t5299 + t4606*t5333) - 0.220205*(t4714*t5232 + t4766*t5299 + t4779*t5333) + t1387*t651*t67;
  p_output1[11]=0;
  p_output1[12]=t2227*t5542 + t2746*t5555 + t3304*t5588 + t3738*t5628 + t4010*t5637 + 0.060173*(t4178*t5588 + t4351*t5628 + t4391*t5637) + 0.293218*(t4461*t5588 + t4529*t5628 + t4606*t5637) - 0.220205*(t4714*t5588 + t4766*t5628 + t4779*t5637) - 1.*t1332*t1387*t651 + t1265*t1304*t1387*t67 + t1387*t1404*t1588*t67;
  p_output1[13]=t2227*t5772 + t2746*t5799 + t3304*t5827 + t3738*t5855 + t4010*t5865 + 0.060173*(t4178*t5827 + t4351*t5855 + t4391*t5865) + 0.293218*(t4461*t5827 + t4529*t5855 + t4606*t5865) - 0.220205*(t4714*t5827 + t4766*t5855 + t4779*t5865) - 1.*t1332*t651*t682 + t1265*t1304*t67*t682 + t1404*t1588*t67*t682;
  p_output1[14]=-1.*t1265*t1304*t1332 - 1.*t1332*t1404*t1588 + t2227*t5982 + t2746*t5994 + t3304*t6017 + t3738*t6035 + t4010*t6041 + 0.060173*(t4178*t6017 + t4351*t6035 + t4391*t6041) + 0.293218*(t4461*t6017 + t4529*t6035 + t4606*t6041) - 0.220205*(t4714*t6017 + t4766*t6035 + t4779*t6041) - 1.*t651*t67;
  p_output1[15]=t1588*t5035 + t1265*t6118 + t2746*t6126 + t2227*t6136 + t3738*t6144 + t3304*t6157 + t4010*t6177 + 0.060173*(t4351*t6144 + t4178*t6157 + t4391*t6177) + 0.293218*(t4529*t6144 + t4461*t6157 + t4606*t6177) - 0.220205*(t4766*t6144 + t4714*t6157 + t4779*t6177);
  p_output1[16]=t1265*t1680 + t1588*t6240 + t2746*t6249 + t2227*t6254 + t3738*t6264 + t3304*t6272 + t4010*t6280 + 0.060173*(t4351*t6264 + t4178*t6272 + t4391*t6280) + 0.293218*(t4529*t6264 + t4461*t6272 + t4606*t6280) - 0.220205*(t4766*t6264 + t4714*t6272 + t4779*t6280);
  p_output1[17]=t2746*t6330 + t2227*t6338 + t3738*t6352 + t3304*t6360 + t4010*t6370 + 0.060173*(t4351*t6352 + t4178*t6360 + t4391*t6370) + 0.293218*(t4529*t6352 + t4461*t6360 + t4606*t6370) - 0.220205*(t4766*t6352 + t4714*t6360 + t4779*t6370) - 1.*t1265*t1404*t67 + t1304*t1588*t67;
  p_output1[18]=0;
  p_output1[19]=0;
  p_output1[20]=0;
  p_output1[21]=0;
  p_output1[22]=0;
  p_output1[23]=0;
  p_output1[24]=0;
  p_output1[25]=0;
  p_output1[26]=0;
  p_output1[27]=0;
  p_output1[28]=0;
  p_output1[29]=0;
  p_output1[30]=0;
  p_output1[31]=0;
  p_output1[32]=0;
  p_output1[33]=0;
  p_output1[34]=0;
  p_output1[35]=0;
  p_output1[36]=0;
  p_output1[37]=0;
  p_output1[38]=0;
  p_output1[39]=0;
  p_output1[40]=0;
  p_output1[41]=0;
  p_output1[42]=0;
  p_output1[43]=0;
  p_output1[44]=0;
  p_output1[45]=0;
  p_output1[46]=0;
  p_output1[47]=0;
  p_output1[48]=0;
  p_output1[49]=0;
  p_output1[50]=0;
  p_output1[51]=0;
  p_output1[52]=0;
  p_output1[53]=0;
  p_output1[54]=0;
  p_output1[55]=0;
  p_output1[56]=0;
  p_output1[57]=0;
  p_output1[58]=0;
  p_output1[59]=0;
  p_output1[60]=0;
  p_output1[61]=0;
  p_output1[62]=0;
  p_output1[63]=0;
  p_output1[64]=0;
  p_output1[65]=0;
  p_output1[66]=0;
  p_output1[67]=0;
  p_output1[68]=0;
  p_output1[69]=0;
  p_output1[70]=0;
  p_output1[71]=0;
  p_output1[72]=t2746*t5132 + t5035*t6422 + t5050*t6430 + t2227*t6442 + t3738*t6455 + t3304*t6464 + t4010*t6469 + 0.060173*(t4351*t6455 + t4178*t6464 + t4391*t6469) + 0.293218*(t4529*t6455 + t4461*t6464 + t4606*t6469) - 0.220205*(t4766*t6455 + t4714*t6464 + t4779*t6469);
  p_output1[73]=t6240*t6422 + t6430*t6516 + t2746*t6524 + t2227*t6534 + t3738*t6541 + t3304*t6547 + t4010*t6552 + 0.060173*(t4351*t6541 + t4178*t6547 + t4391*t6552) + 0.293218*(t4529*t6541 + t4461*t6547 + t4606*t6552) - 0.220205*(t4766*t6541 + t4714*t6547 + t4779*t6552);
  p_output1[74]=t2746*t6599 + t2227*t6604 + t3738*t6608 + t3304*t6621 + t4010*t6628 + 0.060173*(t4351*t6608 + t4178*t6621 + t4391*t6628) + 0.293218*(t4529*t6608 + t4461*t6621 + t4606*t6628) - 0.220205*(t4766*t6608 + t4714*t6621 + t4779*t6628) + t1304*t6422*t67 + t1404*t6430*t67;
  p_output1[75]=t5132*t6680 + t5157*t6693 + t1387*t6672*t67 + t4010*t6701 + t3304*t6710 + t3738*t6730 + 0.060173*(t4391*t6701 + t4178*t6710 + t4351*t6730) + 0.293218*(t4606*t6701 + t4461*t6710 + t4529*t6730) - 0.220205*(t4779*t6701 + t4714*t6710 + t4766*t6730);
  p_output1[76]=t6524*t6680 + t6693*t6765 + t4010*t6778 + t3304*t6794 + t3738*t6799 + 0.060173*(t4391*t6778 + t4178*t6794 + t4351*t6799) + 0.293218*(t4606*t6778 + t4461*t6794 + t4529*t6799) - 0.220205*(t4779*t6778 + t4714*t6794 + t4766*t6799) + t6672*t67*t682;
  p_output1[77]=-1.*t1332*t6672 + t6599*t6680 + t6338*t6693 + t4010*t6830 + t3304*t6837 + t3738*t6844 + 0.060173*(t4391*t6830 + t4178*t6837 + t4351*t6844) + 0.293218*(t4606*t6830 + t4461*t6837 + t4529*t6844) - 0.220205*(t4779*t6830 + t4714*t6837 + t4766*t6844);
  p_output1[78]=t5232*t6880 + t5299*t6893 + t5333*t6903 + 0.293218*(-0.5054634410180001*t3157*t5232 + t5299*t6909 + t5333*t6913) - 0.220205*(-0.9945383682050002*t3157*t5299 + t5232*t6919 + t5333*t6926) + 0.060173*(-0.500001190325*t3157*t5333 + t5232*t6934 + t5299*t6938);
  p_output1[79]=t6880*t6949 + t6893*t6954 + t6903*t6963 + 0.060173*(t6934*t6949 + t6938*t6954 - 0.500001190325*t3157*t6963) + 0.293218*(-0.5054634410180001*t3157*t6949 + t6909*t6954 + t6913*t6963) - 0.220205*(t6919*t6949 - 0.9945383682050002*t3157*t6954 + t6926*t6963);
  p_output1[80]=t6880*t7009 + t6893*t7014 + t6903*t7020 + 0.060173*(t6934*t7009 + t6938*t7014 - 0.500001190325*t3157*t7020) + 0.293218*(-0.5054634410180001*t3157*t7009 + t6909*t7014 + t6913*t7020) - 0.220205*(t6919*t7009 - 0.9945383682050002*t3157*t7014 + t6926*t7020);
  p_output1[81]=0;
  p_output1[82]=0;
  p_output1[83]=0;
}



void Jp_shoulder_yaw_joint_right_src(double *p_output1, const double *var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
