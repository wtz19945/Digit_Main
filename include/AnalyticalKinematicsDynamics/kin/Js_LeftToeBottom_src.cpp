/*
 * Automatically Generated from Mathematica.
 * Thu 10 Nov 2022 14:17:01 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Js_LeftToeBottom_src.h"

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
  double t151;
  double t170;
  double t382;
  double t406;
  double t808;
  double t861;
  double t847;
  double t895;
  double t898;
  double t1080;
  double t1087;
  double t1111;
  double t1130;
  double t1157;
  double t936;
  double t937;
  double t939;
  double t1312;
  double t1351;
  double t1359;
  double t1361;
  double t1437;
  double t1442;
  double t1368;
  double t1378;
  double t1421;
  double t1435;
  double t1473;
  double t1474;
  double t1547;
  double t1556;
  double t1559;
  double t1560;
  double t1567;
  double t1574;
  double t1583;
  double t1656;
  double t1659;
  double t1661;
  double t102;
  double t1487;
  double t1494;
  double t1500;
  double t1140;
  double t1160;
  double t1171;
  double t1731;
  double t1733;
  double t1741;
  double t1640;
  double t1641;
  double t1643;
  double t1757;
  double t1760;
  double t1763;
  double t1220;
  double t1222;
  double t1241;
  double t1863;
  double t1864;
  double t1878;
  double t1881;
  double t1846;
  double t1848;
  double t1851;
  double t2031;
  double t2014;
  double t2020;
  double t2028;
  double t2127;
  double t2129;
  double t2029;
  double t2048;
  double t2052;
  double t2057;
  double t2062;
  double t2066;
  double t2067;
  double t2084;
  double t2101;
  double t2119;
  double t2136;
  double t2137;
  double t2141;
  double t2158;
  double t2159;
  double t2161;
  double t2164;
  double t2170;
  double t2184;
  double t2186;
  double t2095;
  double t2138;
  double t2172;
  double t2179;
  double t2208;
  double t2212;
  double t2213;
  double t2214;
  double t2261;
  double t2265;
  double t2329;
  double t2330;
  double t2334;
  double t2335;
  double t2218;
  double t2220;
  double t2231;
  double t2234;
  double t2365;
  double t2367;
  double t2368;
  double t2378;
  double t2341;
  double t2342;
  double t2345;
  double t2346;
  double t2384;
  double t2385;
  double t2394;
  double t2401;
  double t2286;
  double t2290;
  double t2292;
  double t2294;
  double t2310;
  double t2311;
  double t2181;
  double t2197;
  double t2199;
  double t2202;
  double t2273;
  double t2278;
  double t2280;
  double t2281;
  double t2464;
  double t2469;
  double t2474;
  double t2477;
  double t2485;
  double t2492;
  double t2496;
  double t2499;
  double t2426;
  double t2435;
  double t2437;
  double t2438;
  double t2596;
  double t2598;
  double t2599;
  double t2603;
  double t2522;
  double t2523;
  double t2525;
  double t2530;
  double t2607;
  double t2608;
  double t2594;
  double t2600;
  double t2610;
  double t2612;
  double t2623;
  double t2624;
  double t2631;
  double t2632;
  double t2638;
  double t2655;
  double t2656;
  double t2658;
  double t2662;
  double t2664;
  double t2666;
  double t2668;
  double t2673;
  double t2674;
  double t2689;
  double t2691;
  double t2620;
  double t2661;
  double t2678;
  double t2681;
  double t2708;
  double t2712;
  double t2713;
  double t2714;
  double t2739;
  double t2744;
  double t2315;
  double t2318;
  double t2319;
  double t2320;
  double t2683;
  double t2693;
  double t2699;
  double t2706;
  double t2811;
  double t2812;
  double t2817;
  double t2823;
  double t2871;
  double t2882;
  double t2891;
  double t2898;
  double t2746;
  double t2748;
  double t2749;
  double t2755;
  double t2908;
  double t2909;
  double t2911;
  double t2916;
  double t2857;
  double t2858;
  double t2861;
  double t2863;
  double t2717;
  double t2720;
  double t2724;
  double t2725;
  double t2829;
  double t2837;
  double t2763;
  double t2775;
  double t2776;
  double t2779;
  double t2965;
  double t2973;
  double t2974;
  double t2975;
  double t2989;
  double t2991;
  double t2994;
  double t2998;
  double t3090;
  double t3092;
  double t3094;
  double t3108;
  double t3021;
  double t3023;
  double t3028;
  double t3030;
  double t3163;
  double t3177;
  double t2937;
  double t2943;
  double t2944;
  double t2953;
  double t3217;
  double t3221;
  double t3084;
  double t3102;
  double t3110;
  double t3116;
  double t3120;
  double t3122;
  double t3131;
  double t3133;
  double t3134;
  double t3140;
  double t3141;
  double t3143;
  double t3146;
  double t3152;
  double t3157;
  double t3161;
  double t3188;
  double t3202;
  double t3127;
  double t3155;
  double t3205;
  double t3208;
  double t3260;
  double t3264;
  double t3238;
  double t3242;
  double t3244;
  double t3247;
  double t2839;
  double t2846;
  double t2847;
  double t2849;
  double t3299;
  double t3301;
  double t3303;
  double t3304;
  double t3231;
  double t3233;
  double t3235;
  double t3237;
  double t3356;
  double t3364;
  double t3368;
  double t3374;
  double t3312;
  double t3313;
  double t3315;
  double t3322;
  double t3281;
  double t3285;
  double t3286;
  double t3288;
  double t3381;
  double t3386;
  double t3389;
  double t3393;
  double t3210;
  double t3212;
  double t3224;
  double t3226;
  double t3266;
  double t3273;
  double t3274;
  double t3278;
  double t3326;
  double t3329;
  double t3446;
  double t3453;
  double t3457;
  double t3460;
  double t3463;
  double t3464;
  double t3467;
  double t3469;
  double t3546;
  double t3325;
  double t3331;
  double t3332;
  double t3338;
  double t3574;
  double t3576;
  double t3579;
  double t3592;
  double t3597;
  double t3618;
  double t3624;
  double t3434;
  double t3437;
  double t3438;
  double t3439;
  double t3651;
  double t3656;
  double t3564;
  double t3565;
  double t3555;
  double t3560;
  double t3489;
  double t3491;
  double t3493;
  double t3497;
  double t3677;
  double t3680;
  double t3682;
  double t3684;
  double t3687;
  double t3691;
  double t3692;
  double t3693;
  double t3700;
  double t3707;
  double t3708;
  double t3709;
  double t3642;
  double t3649;
  double t3686;
  double t3695;
  double t3710;
  double t3715;
  double t3733;
  double t3734;
  double t3735;
  double t3737;
  double t3607;
  double t3609;
  double t3629;
  double t3631;
  double t3717;
  double t3719;
  double t3725;
  double t3731;
  double t3640;
  double t3650;
  double t3660;
  double t3661;
  double t3806;
  double t3809;
  double t3811;
  double t3812;
  double t3772;
  double t3773;
  double t3774;
  double t3775;
  double t3818;
  double t3821;
  double t3823;
  double t3826;
  double t3580;
  double t3583;
  double t3669;
  double t3673;
  double t3675;
  double t3676;
  double t3750;
  double t3751;
  double t3753;
  double t3756;
  double t3898;
  double t3900;
  double t3902;
  double t3904;
  double t3909;
  double t3911;
  double t3914;
  double t3915;
  double t3562;
  double t3572;
  double t3587;
  double t3588;
  double t3955;
  double t3957;
  double t3959;
  double t3947;
  double t3948;
  double t3950;
  double t3856;
  double t3863;
  double t3865;
  double t3867;
  double t3964;
  double t3966;
  double t4014;
  double t4016;
  double t4007;
  double t4009;
  double t3881;
  double t3886;
  double t3889;
  double t3892;
  double t3972;
  double t3973;
  double t4034;
  double t4037;
  double t4038;
  double t4039;
  double t4042;
  double t4044;
  double t4046;
  double t4047;
  double t4049;
  double t4050;
  double t4053;
  double t4055;
  double t3978;
  double t3981;
  double t4040;
  double t4048;
  double t4057;
  double t4065;
  double t4097;
  double t4101;
  double t4102;
  double t4104;
  double t3998;
  double t4001;
  double t4029;
  double t4030;
  double t4032;
  double t4033;
  double t4121;
  double t4126;
  double t4128;
  double t4134;
  double t4168;
  double t4172;
  double t4177;
  double t4180;
  double t3954;
  double t3961;
  double t3967;
  double t3968;
  double t4078;
  double t4083;
  double t4088;
  double t4094;
  double t3974;
  double t3983;
  double t3984;
  double t3986;
  double t4215;
  double t4219;
  double t4222;
  double t4224;
  double t4145;
  double t4149;
  double t4152;
  double t4154;
  double t4228;
  double t4229;
  double t4230;
  double t4234;
  double t4004;
  double t4010;
  double t4020;
  double t4022;
  double t4185;
  double t4190;
  double t4191;
  double t4192;
  double t4268;
  double t4272;
  double t4273;
  double t4274;
  double t4279;
  double t4280;
  double t4282;
  double t4287;
  t151 = Cos(var1[3]);
  t170 = Sin(var1[3]);
  t382 = Cos(var1[4]);
  t406 = Sin(var1[4]);
  t808 = Cos(var1[5]);
  t861 = Sin(var1[5]);
  t847 = t151*t808*t406;
  t895 = t170*t861;
  t898 = t847 + t895;
  t1080 = Cos(var1[6]);
  t1087 = -1.*t808*t170;
  t1111 = t151*t406*t861;
  t1130 = t1087 + t1111;
  t1157 = Sin(var1[6]);
  t936 = t808*t170*t406;
  t937 = -1.*t151*t861;
  t939 = t936 + t937;
  t1312 = -1.*t1080;
  t1351 = 1. + t1312;
  t1359 = 0.091*t1351;
  t1361 = 0. + t1359;
  t1437 = 0.091*t1157;
  t1442 = 0. + t1437;
  t1368 = t151*t808;
  t1378 = t170*t406*t861;
  t1421 = t1368 + t1378;
  t1435 = t1361*t1421;
  t1473 = t939*t1442;
  t1474 = 0. + var1[1] + t1435 + t1473;
  t1547 = -1.*var1[2];
  t1556 = -1.*t382*t1361*t861;
  t1559 = -1.*t382*t808*t1442;
  t1560 = 0. + t1547 + t1556 + t1559;
  t1567 = t1080*t1421;
  t1574 = -1.*t939*t1157;
  t1583 = t1567 + t1574;
  t1656 = t1080*t939;
  t1659 = t1421*t1157;
  t1661 = t1656 + t1659;
  t102 = -1.*var1[0];
  t1487 = t382*t1080*t861;
  t1494 = -1.*t382*t808*t1157;
  t1500 = t1487 + t1494;
  t1140 = t1080*t1130;
  t1160 = -1.*t898*t1157;
  t1171 = t1140 + t1160;
  t1731 = -1.*t1361*t1130;
  t1733 = -1.*t898*t1442;
  t1741 = 0. + t102 + t1731 + t1733;
  t1640 = t382*t808*t1080;
  t1641 = t382*t861*t1157;
  t1643 = t1640 + t1641;
  t1757 = t382*t1361*t861;
  t1760 = t382*t808*t1442;
  t1763 = 0. + var1[2] + t1757 + t1760;
  t1220 = t1080*t898;
  t1222 = t1130*t1157;
  t1241 = t1220 + t1222;
  t1863 = -1.*var1[1];
  t1864 = -1.*t1361*t1421;
  t1878 = -1.*t939*t1442;
  t1881 = 0. + t1863 + t1864 + t1878;
  t1846 = t1361*t1130;
  t1848 = t898*t1442;
  t1851 = 0. + var1[0] + t1846 + t1848;
  t2031 = Sin(var1[7]);
  t2014 = Cos(var1[7]);
  t2020 = -1.*t2014;
  t2028 = 1. + t2020;
  t2127 = 0.366501*t2031;
  t2129 = 0. + t2127;
  t2029 = -0.04500040093286238*t2028;
  t2048 = -0.930418*t2031;
  t2052 = 0. + t2048;
  t2057 = 0.07877663122399998*t2052;
  t2062 = -0.366501*t2031;
  t2066 = 0. + t2062;
  t2067 = 0.031030906668*t2066;
  t2084 = 0. + t2029 + t2057 + t2067;
  t2101 = -3.2909349868922137e-7*var1[7];
  t2119 = 0.03103092645718495*t2028;
  t2136 = -0.045000372235*t2129;
  t2137 = t2101 + t2119 + t2136;
  t2141 = 1.296332362046933e-7*var1[7];
  t2158 = 0.07877668146182712*t2028;
  t2159 = 0.930418*t2031;
  t2161 = 0. + t2159;
  t2164 = -0.045000372235*t2161;
  t2170 = t2141 + t2158 + t2164;
  t2184 = -0.134322983001*t2028;
  t2186 = 1. + t2184;
  t2095 = t406*t2084;
  t2138 = -1.*t1643*t2137;
  t2172 = -1.*t1500*t2170;
  t2179 = 0. + t1547 + t1556 + t1559 + t2095 + t2138 + t2172;
  t2208 = t382*t170*t2084;
  t2212 = t1661*t2137;
  t2213 = t1583*t2170;
  t2214 = 0. + var1[1] + t1435 + t1473 + t2208 + t2212 + t2213;
  t2261 = -0.8656776547239999*t2028;
  t2265 = 1. + t2261;
  t2329 = -0.340999127418*t2028*t1171;
  t2330 = t2186*t1241;
  t2334 = t151*t382*t2129;
  t2335 = t2329 + t2330 + t2334;
  t2218 = -0.340999127418*t2028*t1500;
  t2220 = t2186*t1643;
  t2231 = -1.*t406*t2129;
  t2234 = t2218 + t2220 + t2231;
  t2365 = -1.*t406*t2084;
  t2367 = t1643*t2137;
  t2368 = t1500*t2170;
  t2378 = 0. + var1[2] + t1757 + t1760 + t2365 + t2367 + t2368;
  t2341 = t2265*t1171;
  t2342 = -0.340999127418*t2028*t1241;
  t2345 = t151*t382*t2161;
  t2346 = t2341 + t2342 + t2345;
  t2384 = -1.*t151*t382*t2084;
  t2385 = -1.*t1241*t2137;
  t2394 = -1.*t1171*t2170;
  t2401 = 0. + t102 + t1731 + t1733 + t2384 + t2385 + t2394;
  t2286 = t2265*t1500;
  t2290 = -0.340999127418*t2028*t1643;
  t2292 = -1.*t406*t2161;
  t2294 = t2286 + t2290 + t2292;
  t2310 = -1.000000637725*t2028;
  t2311 = 1. + t2310;
  t2181 = -0.340999127418*t2028*t1583;
  t2197 = t2186*t1661;
  t2199 = t382*t170*t2129;
  t2202 = t2181 + t2197 + t2199;
  t2273 = t2265*t1583;
  t2278 = -0.340999127418*t2028*t1661;
  t2280 = t382*t170*t2161;
  t2281 = t2273 + t2278 + t2280;
  t2464 = -1.*t382*t170*t2084;
  t2469 = -1.*t1661*t2137;
  t2474 = -1.*t1583*t2170;
  t2477 = 0. + t1863 + t1864 + t1878 + t2464 + t2469 + t2474;
  t2485 = t151*t382*t2084;
  t2492 = t1241*t2137;
  t2496 = t1171*t2170;
  t2499 = 0. + var1[0] + t1846 + t1848 + t2485 + t2492 + t2496;
  t2426 = t382*t2311*t170;
  t2435 = t1583*t2052;
  t2437 = t1661*t2066;
  t2438 = t2426 + t2435 + t2437;
  t2596 = Cos(var1[8]);
  t2598 = -1.*t2596;
  t2599 = 1. + t2598;
  t2603 = Sin(var1[8]);
  t2522 = -1.*t2311*t406;
  t2523 = t1500*t2052;
  t2525 = t1643*t2066;
  t2530 = t2522 + t2523 + t2525;
  t2607 = -0.366501*t2603;
  t2608 = 0. + t2607;
  t2594 = 3.2909349868922137e-7*var1[8];
  t2600 = 0.055653945343889656*t2599;
  t2610 = -0.045000372235*t2608;
  t2612 = t2594 + t2600 + t2610;
  t2623 = -0.04500040093286238*t2599;
  t2624 = -0.930418*t2603;
  t2631 = 0. + t2624;
  t2632 = -0.141285834136*t2631;
  t2638 = 0.366501*t2603;
  t2655 = 0. + t2638;
  t2656 = 0.055653909852*t2655;
  t2658 = 0. + t2623 + t2632 + t2656;
  t2662 = 1.296332362046933e-7*var1[8];
  t2664 = -0.14128592423750855*t2599;
  t2666 = 0.930418*t2603;
  t2668 = 0. + t2666;
  t2673 = -0.045000372235*t2668;
  t2674 = t2662 + t2664 + t2673;
  t2689 = -0.134322983001*t2599;
  t2691 = 1. + t2689;
  t2620 = t2281*t2612;
  t2661 = t2438*t2658;
  t2678 = t2202*t2674;
  t2681 = 0. + var1[1] + t1435 + t1473 + t2208 + t2212 + t2213 + t2620 + t2661 + t2678;
  t2708 = -1.*t2294*t2612;
  t2712 = -1.*t2530*t2658;
  t2713 = -1.*t2234*t2674;
  t2714 = 0. + t1547 + t1556 + t1559 + t2095 + t2138 + t2172 + t2708 + t2712 + t2713;
  t2739 = -0.8656776547239999*t2599;
  t2744 = 1. + t2739;
  t2315 = t151*t382*t2311;
  t2318 = t1171*t2052;
  t2319 = t1241*t2066;
  t2320 = t2315 + t2318 + t2319;
  t2683 = 0.340999127418*t2599*t2234;
  t2693 = t2691*t2294;
  t2699 = t2530*t2608;
  t2706 = t2683 + t2693 + t2699;
  t2811 = 0.340999127418*t2599*t2335;
  t2812 = t2691*t2346;
  t2817 = t2320*t2608;
  t2823 = t2811 + t2812 + t2817;
  t2871 = -1.*t2346*t2612;
  t2882 = -1.*t2320*t2658;
  t2891 = -1.*t2335*t2674;
  t2898 = 0. + t102 + t1731 + t1733 + t2384 + t2385 + t2394 + t2871 + t2882 + t2891;
  t2746 = t2744*t2234;
  t2748 = 0.340999127418*t2599*t2294;
  t2749 = t2530*t2668;
  t2755 = t2746 + t2748 + t2749;
  t2908 = t2294*t2612;
  t2909 = t2530*t2658;
  t2911 = t2234*t2674;
  t2916 = 0. + var1[2] + t1757 + t1760 + t2365 + t2367 + t2368 + t2908 + t2909 + t2911;
  t2857 = t2744*t2335;
  t2858 = 0.340999127418*t2599*t2346;
  t2861 = t2320*t2668;
  t2863 = t2857 + t2858 + t2861;
  t2717 = 0.340999127418*t2599*t2202;
  t2720 = t2691*t2281;
  t2724 = t2438*t2608;
  t2725 = t2717 + t2720 + t2724;
  t2829 = -1.000000637725*t2599;
  t2837 = 1. + t2829;
  t2763 = t2744*t2202;
  t2775 = 0.340999127418*t2599*t2281;
  t2776 = t2438*t2668;
  t2779 = t2763 + t2775 + t2776;
  t2965 = -1.*t2281*t2612;
  t2973 = -1.*t2438*t2658;
  t2974 = -1.*t2202*t2674;
  t2975 = 0. + t1863 + t1864 + t1878 + t2464 + t2469 + t2474 + t2965 + t2973 + t2974;
  t2989 = t2346*t2612;
  t2991 = t2320*t2658;
  t2994 = t2335*t2674;
  t2998 = 0. + var1[0] + t1846 + t1848 + t2485 + t2492 + t2496 + t2989 + t2991 + t2994;
  t3090 = Cos(var1[9]);
  t3092 = -1.*t3090;
  t3094 = 1. + t3092;
  t3108 = Sin(var1[9]);
  t3021 = t2837*t2530;
  t3023 = t2234*t2631;
  t3028 = t2294*t2655;
  t3030 = t3021 + t3023 + t3028;
  t3163 = -0.930418*t3108;
  t3177 = 0. + t3163;
  t2937 = t2837*t2438;
  t2943 = t2202*t2631;
  t2944 = t2281*t2655;
  t2953 = t2937 + t2943 + t2944;
  t3217 = -0.8656776547239999*t3094;
  t3221 = 1. + t3217;
  t3084 = -1.5981976069815686e-7*var1[9];
  t3102 = 0.08675267452931407*t3094;
  t3110 = 0.366501*t3108;
  t3116 = 0. + t3110;
  t3120 = 0.039853013046*t3116;
  t3122 = t3084 + t3102 + t3120;
  t3131 = 0.039853038461262744*t3094;
  t3133 = -0.366501*t3108;
  t3134 = 0. + t3133;
  t3140 = 0.086752619205*t3134;
  t3141 = 0.930418*t3108;
  t3143 = 0. + t3141;
  t3146 = -0.22023459268999998*t3143;
  t3152 = 0. + t3131 + t3140 + t3146;
  t3157 = -6.295460977284962e-8*var1[9];
  t3161 = -0.22023473313910558*t3094;
  t3188 = 0.039853013046*t3177;
  t3202 = t3157 + t3161 + t3188;
  t3127 = -1.*t3122*t2706;
  t3155 = -1.*t3152*t3030;
  t3205 = -1.*t3202*t2755;
  t3208 = 0. + t1547 + t1556 + t1559 + t2095 + t2138 + t2172 + t2708 + t3127 + t2712 + t3155 + t2713 + t3205;
  t3260 = -0.134322983001*t3094;
  t3264 = 1. + t3260;
  t3238 = t3122*t2725;
  t3242 = t3152*t2953;
  t3244 = t3202*t2779;
  t3247 = 0. + var1[1] + t1435 + t1473 + t2208 + t2212 + t2213 + t2620 + t3238 + t2661 + t3242 + t2678 + t3244;
  t2839 = t2837*t2320;
  t2846 = t2335*t2631;
  t2847 = t2346*t2655;
  t2849 = t2839 + t2846 + t2847;
  t3299 = 0.340999127418*t3094*t2823;
  t3301 = t3177*t2849;
  t3303 = t3221*t2863;
  t3304 = t3299 + t3301 + t3303;
  t3231 = 0.340999127418*t3094*t2706;
  t3233 = t3177*t3030;
  t3235 = t3221*t2755;
  t3237 = t3231 + t3233 + t3235;
  t3356 = t3122*t2706;
  t3364 = t3152*t3030;
  t3368 = t3202*t2755;
  t3374 = 0. + var1[2] + t1757 + t1760 + t2365 + t2367 + t2368 + t2908 + t3356 + t2909 + t3364 + t2911 + t3368;
  t3312 = t3264*t2823;
  t3313 = t3116*t2849;
  t3315 = 0.340999127418*t3094*t2863;
  t3322 = t3312 + t3313 + t3315;
  t3281 = t3264*t2706;
  t3285 = t3116*t3030;
  t3286 = 0.340999127418*t3094*t2755;
  t3288 = t3281 + t3285 + t3286;
  t3381 = -1.*t3122*t2823;
  t3386 = -1.*t3152*t2849;
  t3389 = -1.*t3202*t2863;
  t3393 = 0. + t102 + t1731 + t1733 + t2384 + t2385 + t2394 + t2871 + t3381 + t2882 + t3386 + t2891 + t3389;
  t3210 = 0.340999127418*t3094*t2725;
  t3212 = t3177*t2953;
  t3224 = t3221*t2779;
  t3226 = t3210 + t3212 + t3224;
  t3266 = t3264*t2725;
  t3273 = t3116*t2953;
  t3274 = 0.340999127418*t3094*t2779;
  t3278 = t3266 + t3273 + t3274;
  t3326 = -1.000000637725*t3094;
  t3329 = 1. + t3326;
  t3446 = t3122*t2823;
  t3453 = t3152*t2849;
  t3457 = t3202*t2863;
  t3460 = 0. + var1[0] + t1846 + t1848 + t2485 + t2492 + t2496 + t2989 + t3446 + t2991 + t3453 + t2994 + t3457;
  t3463 = -1.*t3122*t2725;
  t3464 = -1.*t3152*t2953;
  t3467 = -1.*t3202*t2779;
  t3469 = 0. + t1863 + t1864 + t1878 + t2464 + t2469 + t2474 + t2965 + t3463 + t2973 + t3464 + t2974 + t3467;
  t3546 = Sin(var1[10]);
  t3325 = t3134*t2823;
  t3331 = t3329*t2849;
  t3332 = t3143*t2863;
  t3338 = t3325 + t3331 + t3332;
  t3574 = Cos(var1[10]);
  t3576 = -1.*t3574;
  t3579 = 1. + t3576;
  t3592 = -0.8656776547239999*t3579;
  t3597 = 1. + t3592;
  t3618 = -0.930418*t3546;
  t3624 = 0. + t3618;
  t3434 = t3134*t2725;
  t3437 = t3329*t2953;
  t3438 = t3143*t2779;
  t3439 = t3434 + t3437 + t3438;
  t3651 = 0.366501*t3546;
  t3656 = 0. + t3651;
  t3564 = -0.366501*t3546;
  t3565 = 0. + t3564;
  t3555 = 0.930418*t3546;
  t3560 = 0. + t3555;
  t3489 = t3134*t2706;
  t3491 = t3329*t3030;
  t3493 = t3143*t2755;
  t3497 = t3489 + t3491 + t3493;
  t3677 = 2.281945176511838e-8*var1[10];
  t3680 = -0.5905366811997648*t3579;
  t3682 = -0.262809976934*t3624;
  t3684 = t3677 + t3680 + t3682;
  t3687 = 5.7930615939377813e-8*var1[10];
  t3691 = 0.23261833304643187*t3579;
  t3692 = -0.262809976934*t3656;
  t3693 = t3687 + t3691 + t3692;
  t3700 = -0.26281014453449253*t3579;
  t3707 = 0.23261818470000004*t3565;
  t3708 = -0.5905363046000001*t3560;
  t3709 = 0. + t3700 + t3707 + t3708;
  t3642 = -0.134322983001*t3579;
  t3649 = 1. + t3642;
  t3686 = -1.*t3684*t3237;
  t3695 = -1.*t3693*t3288;
  t3710 = -1.*t3709*t3497;
  t3715 = 0. + t1547 + t1556 + t1559 + t2095 + t2138 + t2172 + t3686 + t3695 + t3710 + t2708 + t3127 + t2712 + t3155 + t2713 + t3205;
  t3733 = t3684*t3226;
  t3734 = t3693*t3278;
  t3735 = t3709*t3439;
  t3737 = 0. + var1[1] + t1435 + t1473 + t2208 + t2212 + t2213 + t3733 + t3734 + t3735 + t2620 + t3238 + t2661 + t3242 + t2678 + t3244;
  t3607 = t3597*t3304;
  t3609 = 0.340999127418*t3579*t3322;
  t3629 = t3624*t3338;
  t3631 = t3607 + t3609 + t3629;
  t3717 = t3597*t3237;
  t3719 = 0.340999127418*t3579*t3288;
  t3725 = t3624*t3497;
  t3731 = t3717 + t3719 + t3725;
  t3640 = 0.340999127418*t3579*t3304;
  t3650 = t3649*t3322;
  t3660 = t3656*t3338;
  t3661 = t3640 + t3650 + t3660;
  t3806 = t3684*t3237;
  t3809 = t3693*t3288;
  t3811 = t3709*t3497;
  t3812 = 0. + var1[2] + t1757 + t1760 + t2365 + t2367 + t2368 + t3806 + t3809 + t3811 + t2908 + t3356 + t2909 + t3364 + t2911 + t3368;
  t3772 = 0.340999127418*t3579*t3237;
  t3773 = t3649*t3288;
  t3774 = t3656*t3497;
  t3775 = t3772 + t3773 + t3774;
  t3818 = -1.*t3684*t3304;
  t3821 = -1.*t3693*t3322;
  t3823 = -1.*t3709*t3338;
  t3826 = 0. + t102 + t1731 + t1733 + t2384 + t2385 + t2394 + t3818 + t3821 + t3823 + t2871 + t3381 + t2882 + t3386 + t2891 + t3389;
  t3580 = -1.000000637725*t3579;
  t3583 = 1. + t3580;
  t3669 = t3597*t3226;
  t3673 = 0.340999127418*t3579*t3278;
  t3675 = t3624*t3439;
  t3676 = t3669 + t3673 + t3675;
  t3750 = 0.340999127418*t3579*t3226;
  t3751 = t3649*t3278;
  t3753 = t3656*t3439;
  t3756 = t3750 + t3751 + t3753;
  t3898 = t3684*t3304;
  t3900 = t3693*t3322;
  t3902 = t3709*t3338;
  t3904 = 0. + var1[0] + t1846 + t1848 + t2485 + t2492 + t2496 + t3898 + t3900 + t3902 + t2989 + t3446 + t2991 + t3453 + t2994 + t3457;
  t3909 = -1.*t3684*t3226;
  t3911 = -1.*t3693*t3278;
  t3914 = -1.*t3709*t3439;
  t3915 = 0. + t1863 + t1864 + t1878 + t2464 + t2469 + t2474 + t3909 + t3911 + t3914 + t2965 + t3463 + t2973 + t3464 + t2974 + t3467;
  t3562 = t3560*t3304;
  t3572 = t3565*t3322;
  t3587 = t3583*t3338;
  t3588 = t3562 + t3572 + t3587;
  t3955 = Cos(var1[11]);
  t3957 = -1.*t3955;
  t3959 = 1. + t3957;
  t3947 = Sin(var1[11]);
  t3948 = 0.366501*t3947;
  t3950 = 0. + t3948;
  t3856 = t3560*t3226;
  t3863 = t3565*t3278;
  t3865 = t3583*t3439;
  t3867 = t3856 + t3863 + t3865;
  t3964 = -0.134322983001*t3959;
  t3966 = 1. + t3964;
  t4014 = -0.366501*t3947;
  t4016 = 0. + t4014;
  t4007 = 0.930418*t3947;
  t4009 = 0. + t4007;
  t3881 = t3560*t3237;
  t3886 = t3565*t3288;
  t3889 = t3583*t3497;
  t3892 = t3881 + t3886 + t3889;
  t3972 = -0.930418*t3947;
  t3973 = 0. + t3972;
  t4034 = 0.06199697675299678*t3959;
  t4037 = 0.324290713329*t4016;
  t4038 = -0.823260828522*t4009;
  t4039 = 0. + t4034 + t4037 + t4038;
  t4042 = 2.95447451120871e-8*var1[11];
  t4044 = -0.8232613535360118*t3959;
  t4046 = 0.061996937216*t3973;
  t4047 = t4042 + t4044 + t4046;
  t4049 = 7.500378623168247e-8*var1[11];
  t4050 = 0.32429092013729516*t3959;
  t4053 = 0.061996937216*t3950;
  t4055 = t4049 + t4050 + t4053;
  t3978 = -0.8656776547239999*t3959;
  t3981 = 1. + t3978;
  t4040 = -1.*t4039*t3892;
  t4048 = -1.*t4047*t3731;
  t4057 = -1.*t4055*t3775;
  t4065 = 0. + t1547 + t1556 + t1559 + t2095 + t2138 + t2172 + t4040 + t4048 + t4057 + t3686 + t3695 + t3710 + t2708 + t3127 + t2712 + t3155 + t2713 + t3205;
  t4097 = t4039*t3867;
  t4101 = t4047*t3676;
  t4102 = t4055*t3756;
  t4104 = 0. + var1[1] + t1435 + t1473 + t2208 + t2212 + t2213 + t4097 + t4101 + t4102 + t3733 + t3734 + t3735 + t2620 + t3238 + t2661 + t3242 + t2678 + t3244;
  t3998 = -1.000000637725*t3959;
  t4001 = 1. + t3998;
  t4029 = t3950*t3867;
  t4030 = 0.340999127418*t3959*t3676;
  t4032 = t3966*t3756;
  t4033 = t4029 + t4030 + t4032;
  t4121 = t3973*t3867;
  t4126 = t3981*t3676;
  t4128 = 0.340999127418*t3959*t3756;
  t4134 = t4121 + t4126 + t4128;
  t4168 = t4001*t3867;
  t4172 = t4009*t3676;
  t4177 = t4016*t3756;
  t4180 = t4168 + t4172 + t4177;
  t3954 = t3950*t3588;
  t3961 = 0.340999127418*t3959*t3631;
  t3967 = t3966*t3661;
  t3968 = t3954 + t3961 + t3967;
  t4078 = t3950*t3892;
  t4083 = 0.340999127418*t3959*t3731;
  t4088 = t3966*t3775;
  t4094 = t4078 + t4083 + t4088;
  t3974 = t3973*t3588;
  t3983 = t3981*t3631;
  t3984 = 0.340999127418*t3959*t3661;
  t3986 = t3974 + t3983 + t3984;
  t4215 = t4039*t3892;
  t4219 = t4047*t3731;
  t4222 = t4055*t3775;
  t4224 = 0. + var1[2] + t1757 + t1760 + t2365 + t2367 + t2368 + t4215 + t4219 + t4222 + t3806 + t3809 + t3811 + t2908 + t3356 + t2909 + t3364 + t2911 + t3368;
  t4145 = t3973*t3892;
  t4149 = t3981*t3731;
  t4152 = 0.340999127418*t3959*t3775;
  t4154 = t4145 + t4149 + t4152;
  t4228 = -1.*t4039*t3588;
  t4229 = -1.*t4047*t3631;
  t4230 = -1.*t4055*t3661;
  t4234 = 0. + t102 + t1731 + t1733 + t2384 + t2385 + t2394 + t4228 + t4229 + t4230 + t3818 + t3821 + t3823 + t2871 + t3381 + t2882 + t3386 + t2891 + t3389;
  t4004 = t4001*t3588;
  t4010 = t4009*t3631;
  t4020 = t4016*t3661;
  t4022 = t4004 + t4010 + t4020;
  t4185 = t4001*t3892;
  t4190 = t4009*t3731;
  t4191 = t4016*t3775;
  t4192 = t4185 + t4190 + t4191;
  t4268 = t4039*t3588;
  t4272 = t4047*t3631;
  t4273 = t4055*t3661;
  t4274 = 0. + var1[0] + t1846 + t1848 + t2485 + t2492 + t2496 + t4268 + t4272 + t4273 + t3898 + t3900 + t3902 + t2989 + t3446 + t2991 + t3453 + t2994 + t3457;
  t4279 = -1.*t4039*t3867;
  t4280 = -1.*t4047*t3676;
  t4282 = -1.*t4055*t3756;
  t4287 = 0. + t1863 + t1864 + t1878 + t2464 + t2469 + t2474 + t4279 + t4280 + t4282 + t3909 + t3911 + t3914 + t2965 + t3463 + t2973 + t3464 + t2974 + t3467;
  p_output1[0]=1.;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=0;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=1.;
  p_output1[8]=0;
  p_output1[9]=0;
  p_output1[10]=0;
  p_output1[11]=0;
  p_output1[12]=0;
  p_output1[13]=0;
  p_output1[14]=1.;
  p_output1[15]=0;
  p_output1[16]=0;
  p_output1[17]=0;
  p_output1[18]=var1[1];
  p_output1[19]=t102;
  p_output1[20]=0;
  p_output1[21]=0;
  p_output1[22]=0;
  p_output1[23]=1.;
  p_output1[24]=-1.*t151*var1[2];
  p_output1[25]=-1.*t170*var1[2];
  p_output1[26]=t151*var1[0] + t170*var1[1];
  p_output1[27]=-1.*t170;
  p_output1[28]=t151;
  p_output1[29]=0;
  p_output1[30]=-1.*t406*var1[1] - 1.*t170*t382*var1[2];
  p_output1[31]=t406*var1[0] + t151*t382*var1[2];
  p_output1[32]=t170*t382*var1[0] - 1.*t151*t382*var1[1];
  p_output1[33]=t151*t382;
  p_output1[34]=t170*t382;
  p_output1[35]=-1.*t406;
  p_output1[36]=0.091*t898 + t406*var1[1] + t170*t382*var1[2];
  p_output1[37]=0.091*t939 - 1.*t406*var1[0] - 1.*t151*t382*var1[2];
  p_output1[38]=0.091*t382*t808 - 1.*t170*t382*var1[0] + t151*t382*var1[1];
  p_output1[39]=0. - 1.*t151*t382;
  p_output1[40]=0. - 1.*t170*t382;
  p_output1[41]=0. + t406;
  p_output1[42]=-0.041869*t1171 - 0.016493*t1241 + 0.366501*(t1474*t1500 + t1560*t1583) - 0.930418*(t1474*t1643 + t1560*t1661) - 0.084668*t151*t382;
  p_output1[43]=-0.041869*t1583 - 0.016493*t1661 + 0.366501*(t1500*t1741 + t1171*t1763) - 0.930418*(t1643*t1741 + t1241*t1763) - 0.084668*t170*t382;
  p_output1[44]=-0.041869*t1500 - 0.016493*t1643 + 0.366501*(t1583*t1851 + t1171*t1881) - 0.930418*(t1661*t1851 + t1241*t1881) + 0.084668*t406;
  p_output1[45]=0. + 0.366501*t1171 - 0.930418*t1241;
  p_output1[46]=0. + 0.366501*t1583 - 0.930418*t1661;
  p_output1[47]=0. + 0.366501*t1500 - 0.930418*t1643;
  p_output1[48]=0.366501*(t2179*t2202 + t2214*t2234) + 0.930418*(t2179*t2281 + t2214*t2294) + 0.151852*t2320 - 0.041869*t2335 + 0.016493*t2346;
  p_output1[49]=-0.041869*t2202 + 0.016493*t2281 + 0.366501*(t2335*t2378 + t2234*t2401) + 0.930418*(t2346*t2378 + t2294*t2401) + 0.151852*t2438;
  p_output1[50]=-0.041869*t2234 + 0.016493*t2294 + 0.366501*(t2335*t2477 + t2202*t2499) + 0.930418*(t2346*t2477 + t2281*t2499) + 0.151852*t2530;
  p_output1[51]=0. + 0.366501*t2335 + 0.930418*t2346;
  p_output1[52]=0. + 0.366501*t2202 + 0.930418*t2281;
  p_output1[53]=0. + 0.366501*t2234 + 0.930418*t2294;
  p_output1[54]=-0.930418*(t2681*t2706 + t2714*t2725) - 0.366501*(t2681*t2755 + t2714*t2779) + 0.014606*t2823 - 0.236705*t2849 - 0.03708*t2863;
  p_output1[55]=0.014606*t2725 - 0.03708*t2779 - 0.930418*(t2706*t2898 + t2823*t2916) - 0.366501*(t2755*t2898 + t2863*t2916) - 0.236705*t2953;
  p_output1[56]=0.014606*t2706 - 0.03708*t2755 - 0.930418*(t2823*t2975 + t2725*t2998) - 0.366501*(t2863*t2975 + t2779*t2998) - 0.236705*t3030;
  p_output1[57]=0. - 0.930418*t2823 - 0.366501*t2863;
  p_output1[58]=0. - 0.930418*t2725 - 0.366501*t2779;
  p_output1[59]=0. - 0.930418*t2706 - 0.366501*t2755;
  p_output1[60]=-0.366501*(t3208*t3226 + t3237*t3247) - 0.930418*(t3208*t3278 + t3247*t3288) + 0.244523*t3304 - 0.09632*t3322 - 0.6347*t3338;
  p_output1[61]=0.244523*t3226 - 0.09632*t3278 - 0.366501*(t3304*t3374 + t3237*t3393) - 0.930418*(t3322*t3374 + t3288*t3393) - 0.6347*t3439;
  p_output1[62]=0.244523*t3237 - 0.09632*t3288 - 0.366501*(t3226*t3460 + t3304*t3469) - 0.930418*(t3278*t3460 + t3322*t3469) - 0.6347*t3497;
  p_output1[63]=0. - 0.366501*t3304 - 0.930418*t3322;
  p_output1[64]=0. - 0.366501*t3226 - 0.930418*t3278;
  p_output1[65]=0. - 0.366501*t3237 - 0.930418*t3288;
  p_output1[66]=-0.884829*t3588 - 0.057683*t3631 + 0.022722*t3661 - 0.366501*(t3676*t3715 + t3731*t3737) - 0.930418*(t3715*t3756 + t3737*t3775);
  p_output1[67]=-0.057683*t3676 + 0.022722*t3756 - 0.366501*(t3631*t3812 + t3731*t3826) - 0.930418*(t3661*t3812 + t3775*t3826) - 0.884829*t3867;
  p_output1[68]=-0.057683*t3731 + 0.022722*t3775 - 0.884829*t3892 - 0.366501*(t3676*t3904 + t3631*t3915) - 0.930418*(t3756*t3904 + t3661*t3915);
  p_output1[69]=0. - 0.366501*t3631 - 0.930418*t3661;
  p_output1[70]=0. - 0.366501*t3676 - 0.930418*t3756;
  p_output1[71]=0. - 0.366501*t3731 - 0.930418*t3775;
  p_output1[72]=-0.671277*t3968 - 0.337139*t3986 + 0.050068*t4022 - 0.218018*(t4033*t4065 + t4094*t4104) + 0.553471*(t4065*t4134 + t4104*t4154) + 0.803828*(t4065*t4180 + t4104*t4192);
  p_output1[73]=-0.671277*t4033 - 0.337139*t4134 + 0.050068*t4180 - 0.218018*(t3968*t4224 + t4094*t4234) + 0.553471*(t3986*t4224 + t4154*t4234) + 0.803828*(t4022*t4224 + t4192*t4234);
  p_output1[74]=-0.671277*t4094 - 0.337139*t4154 + 0.050068*t4192 - 0.218018*(t4033*t4274 + t3968*t4287) + 0.553471*(t4134*t4274 + t3986*t4287) + 0.803828*(t4180*t4274 + t4022*t4287);
  p_output1[75]=0. - 0.218018*t3968 + 0.553471*t3986 + 0.803828*t4022;
  p_output1[76]=0. - 0.218018*t4033 + 0.553471*t4134 + 0.803828*t4180;
  p_output1[77]=0. - 0.218018*t4094 + 0.553471*t4154 + 0.803828*t4192;
  p_output1[78]=0;
  p_output1[79]=0;
  p_output1[80]=0;
  p_output1[81]=0;
  p_output1[82]=0;
  p_output1[83]=0;
  p_output1[84]=0;
  p_output1[85]=0;
  p_output1[86]=0;
  p_output1[87]=0;
  p_output1[88]=0;
  p_output1[89]=0;
  p_output1[90]=0;
  p_output1[91]=0;
  p_output1[92]=0;
  p_output1[93]=0;
  p_output1[94]=0;
  p_output1[95]=0;
  p_output1[96]=0;
  p_output1[97]=0;
  p_output1[98]=0;
  p_output1[99]=0;
  p_output1[100]=0;
  p_output1[101]=0;
  p_output1[102]=0;
  p_output1[103]=0;
  p_output1[104]=0;
  p_output1[105]=0;
  p_output1[106]=0;
  p_output1[107]=0;
  p_output1[108]=0;
  p_output1[109]=0;
  p_output1[110]=0;
  p_output1[111]=0;
  p_output1[112]=0;
  p_output1[113]=0;
  p_output1[114]=0;
  p_output1[115]=0;
  p_output1[116]=0;
  p_output1[117]=0;
  p_output1[118]=0;
  p_output1[119]=0;
}



void Js_LeftToeBottom_src(double *p_output1, const double *var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
