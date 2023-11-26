/*
 * Automatically Generated from Mathematica.
 * Mon 4 Jul 2022 20:55:01 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_toe_roll_joint_right_src.h"

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
  double t617;
  double t1258;
  double t1353;
  double t1281;
  double t1358;
  double t969;
  double t1079;
  double t1102;
  double t1602;
  double t1287;
  double t1382;
  double t1385;
  double t1178;
  double t1611;
  double t1612;
  double t1613;
  double t291;
  double t623;
  double t640;
  double t1530;
  double t1614;
  double t1635;
  double t1694;
  double t1698;
  double t1700;
  double t1937;
  double t1940;
  double t1981;
  double t701;
  double t943;
  double t947;
  double t1638;
  double t1642;
  double t1662;
  double t1761;
  double t1766;
  double t1879;
  double t1901;
  double t1905;
  double t1906;
  double t1910;
  double t1913;
  double t1922;
  double t1926;
  double t2030;
  double t2035;
  double t2036;
  double t2056;
  double t2060;
  double t2082;
  double t2099;
  double t2108;
  double t2111;
  double t2112;
  double t2147;
  double t2154;
  double t2159;
  double t242;
  double t251;
  double t352;
  double t448;
  double t1768;
  double t1819;
  double t1878;
  double t1934;
  double t2007;
  double t2019;
  double t2120;
  double t2135;
  double t2166;
  double t2167;
  double t2170;
  double t2177;
  double t2179;
  double t2180;
  double t2183;
  double t2207;
  double t2235;
  double t2258;
  double t2262;
  double t2263;
  double t2264;
  double t2265;
  double t2266;
  double t2279;
  double t2359;
  double t2362;
  double t2377;
  double t260;
  double t267;
  double t2139;
  double t2212;
  double t2213;
  double t2215;
  double t2287;
  double t2290;
  double t2307;
  double t2322;
  double t2339;
  double t2342;
  double t2343;
  double t2352;
  double t2353;
  double t2357;
  double t2394;
  double t2398;
  double t2400;
  double t2402;
  double t2403;
  double t2404;
  double t2411;
  double t2412;
  double t2418;
  double t2423;
  double t2439;
  double t2440;
  double t2442;
  double t115;
  double t123;
  double t126;
  double t185;
  double t244;
  double t247;
  double t2296;
  double t2298;
  double t2301;
  double t2358;
  double t2382;
  double t2387;
  double t2426;
  double t2428;
  double t2445;
  double t2446;
  double t2447;
  double t2458;
  double t2461;
  double t2462;
  double t2471;
  double t2478;
  double t2486;
  double t2488;
  double t2489;
  double t2490;
  double t2492;
  double t2500;
  double t2502;
  double t2512;
  double t140;
  double t186;
  double t196;
  double t2432;
  double t2480;
  double t2481;
  double t2482;
  double t2517;
  double t2518;
  double t128;
  double t2524;
  double t2525;
  double t2528;
  double t2529;
  double t2535;
  double t2536;
  double t2537;
  double t2540;
  double t2547;
  double t2556;
  double t2559;
  double t2561;
  double t2562;
  double t2570;
  double t2571;
  double t2572;
  double t2578;
  double t2581;
  double t149;
  double t172;
  double t2623;
  double t2624;
  double t2626;
  double t2635;
  double t2636;
  double t2638;
  double t2631;
  double t2641;
  double t2643;
  double t2645;
  double t2646;
  double t2647;
  double t2619;
  double t2644;
  double t2650;
  double t2652;
  double t2654;
  double t2655;
  double t2657;
  double t2668;
  double t2685;
  double t2689;
  double t2697;
  double t2699;
  double t2653;
  double t2670;
  double t2715;
  double t2719;
  double t2725;
  double t2726;
  double t2729;
  double t2731;
  double t2744;
  double t2745;
  double t2747;
  double t2749;
  double t2723;
  double t2743;
  double t2750;
  double t2751;
  double t2758;
  double t2760;
  double t2761;
  double t2762;
  double t2766;
  double t2767;
  double t2770;
  double t2771;
  double t2522;
  double t2523;
  double t2754;
  double t2763;
  double t2773;
  double t2776;
  double t2779;
  double t2781;
  double t2782;
  double t2792;
  double t2803;
  double t2804;
  double t2806;
  double t2810;
  double t2542;
  double t2545;
  double t2546;
  double t2595;
  double t2599;
  double t2778;
  double t2795;
  double t2817;
  double t2819;
  double t2601;
  double t2604;
  double t2821;
  double t2827;
  double t2828;
  double t2830;
  double t2607;
  double t2610;
  double t2611;
  double t2839;
  double t2842;
  double t2843;
  double t2849;
  double t2906;
  double t2908;
  double t2909;
  double t2916;
  double t2921;
  double t2924;
  double t2905;
  double t2914;
  double t2927;
  double t2929;
  double t2939;
  double t2941;
  double t2947;
  double t2949;
  double t2953;
  double t2957;
  double t2959;
  double t2962;
  double t2937;
  double t2950;
  double t2966;
  double t2967;
  double t2971;
  double t2973;
  double t2975;
  double t2976;
  double t2982;
  double t2984;
  double t2985;
  double t2991;
  double t2968;
  double t2980;
  double t2992;
  double t2993;
  double t2995;
  double t2997;
  double t2999;
  double t3000;
  double t3003;
  double t3004;
  double t3007;
  double t3008;
  double t2994;
  double t3002;
  double t3012;
  double t3017;
  double t3023;
  double t3025;
  double t3026;
  double t3029;
  double t3031;
  double t3032;
  double t3033;
  double t3039;
  double t3018;
  double t3030;
  double t3040;
  double t3041;
  double t3047;
  double t3048;
  double t3050;
  double t3056;
  double t3058;
  double t3061;
  double t3064;
  double t3069;
  double t2521;
  double t2541;
  double t2583;
  double t2584;
  double t2600;
  double t2606;
  double t2613;
  double t2616;
  double t3108;
  double t3111;
  double t3123;
  double t3124;
  double t3130;
  double t3131;
  double t2820;
  double t2838;
  double t2851;
  double t2855;
  double t2868;
  double t2879;
  double t2880;
  double t2884;
  double t3043;
  double t3057;
  double t3073;
  double t3078;
  double t3084;
  double t3090;
  double t3095;
  double t3104;
  double t3117;
  double t3125;
  double t3135;
  double t3136;
  double t3147;
  double t3149;
  double t3150;
  double t3155;
  double t3180;
  double t3183;
  double t3184;
  double t3198;
  t617 = Cos(var1[3]);
  t1258 = Cos(var1[5]);
  t1353 = Sin(var1[3]);
  t1281 = Sin(var1[4]);
  t1358 = Sin(var1[5]);
  t969 = Cos(var1[18]);
  t1079 = -1.*t969;
  t1102 = 1. + t1079;
  t1602 = Cos(var1[17]);
  t1287 = t617*t1258*t1281;
  t1382 = t1353*t1358;
  t1385 = t1287 + t1382;
  t1178 = Sin(var1[17]);
  t1611 = -1.*t1258*t1353;
  t1612 = t617*t1281*t1358;
  t1613 = t1611 + t1612;
  t291 = Sin(var1[19]);
  t623 = Cos(var1[4]);
  t640 = Sin(var1[18]);
  t1530 = -1.*t1178*t1385;
  t1614 = t1602*t1613;
  t1635 = t1530 + t1614;
  t1694 = t1602*t1385;
  t1698 = t1178*t1613;
  t1700 = t1694 + t1698;
  t1937 = Cos(var1[19]);
  t1940 = -1.*t1937;
  t1981 = 1. + t1940;
  t701 = -0.366501*t640;
  t943 = 0. + t701;
  t947 = t617*t623*t943;
  t1638 = 0.340999127418*t1102*t1635;
  t1642 = -0.134322983001*t1102;
  t1662 = 1. + t1642;
  t1761 = t1662*t1700;
  t1766 = t947 + t1638 + t1761;
  t1879 = 0.930418*t640;
  t1901 = 0. + t1879;
  t1905 = t617*t623*t1901;
  t1906 = -0.8656776547239999*t1102;
  t1910 = 1. + t1906;
  t1913 = t1910*t1635;
  t1922 = 0.340999127418*t1102*t1700;
  t1926 = t1905 + t1913 + t1922;
  t2030 = -1.000000637725*t1102;
  t2035 = 1. + t2030;
  t2036 = t2035*t617*t623;
  t2056 = -0.930418*t640;
  t2060 = 0. + t2056;
  t2082 = t2060*t1635;
  t2099 = 0.366501*t640;
  t2108 = 0. + t2099;
  t2111 = t2108*t1700;
  t2112 = t2036 + t2082 + t2111;
  t2147 = Cos(var1[20]);
  t2154 = -1.*t2147;
  t2159 = 1. + t2154;
  t242 = Sin(var1[21]);
  t251 = Sin(var1[20]);
  t352 = 0.930418*t291;
  t448 = 0. + t352;
  t1768 = t448*t1766;
  t1819 = 0.366501*t291;
  t1878 = 0. + t1819;
  t1934 = t1878*t1926;
  t2007 = -1.000000637725*t1981;
  t2019 = 1. + t2007;
  t2120 = t2019*t2112;
  t2135 = t1768 + t1934 + t2120;
  t2166 = -0.8656776547239999*t1981;
  t2167 = 1. + t2166;
  t2170 = t2167*t1766;
  t2177 = -0.340999127418*t1981*t1926;
  t2179 = -0.930418*t291;
  t2180 = 0. + t2179;
  t2183 = t2180*t2112;
  t2207 = t2170 + t2177 + t2183;
  t2235 = -0.340999127418*t1981*t1766;
  t2258 = -0.134322983001*t1981;
  t2262 = 1. + t2258;
  t2263 = t2262*t1926;
  t2264 = -0.366501*t291;
  t2265 = 0. + t2264;
  t2266 = t2265*t2112;
  t2279 = t2235 + t2263 + t2266;
  t2359 = Cos(var1[21]);
  t2362 = -1.*t2359;
  t2377 = 1. + t2362;
  t260 = 0.366501*t251;
  t267 = 0. + t260;
  t2139 = t267*t2135;
  t2212 = -0.340999127418*t2159*t2207;
  t2213 = -0.134322983001*t2159;
  t2215 = 1. + t2213;
  t2287 = t2215*t2279;
  t2290 = t2139 + t2212 + t2287;
  t2307 = 0.930418*t251;
  t2322 = 0. + t2307;
  t2339 = t2322*t2135;
  t2342 = -0.8656776547239999*t2159;
  t2343 = 1. + t2342;
  t2352 = t2343*t2207;
  t2353 = -0.340999127418*t2159*t2279;
  t2357 = t2339 + t2352 + t2353;
  t2394 = -1.000000637725*t2159;
  t2398 = 1. + t2394;
  t2400 = t2398*t2135;
  t2402 = -0.930418*t251;
  t2403 = 0. + t2402;
  t2404 = t2403*t2207;
  t2411 = -0.366501*t251;
  t2412 = 0. + t2411;
  t2418 = t2412*t2279;
  t2423 = t2400 + t2404 + t2418;
  t2439 = Cos(var1[22]);
  t2440 = -1.*t2439;
  t2442 = 1. + t2440;
  t115 = Cos(var1[23]);
  t123 = -1.*t115;
  t126 = 1. + t123;
  t185 = Sin(var1[22]);
  t244 = -0.366501*t242;
  t247 = 0. + t244;
  t2296 = t247*t2290;
  t2298 = -0.930418*t242;
  t2301 = 0. + t2298;
  t2358 = t2301*t2357;
  t2382 = -1.000000637725*t2377;
  t2387 = 1. + t2382;
  t2426 = t2387*t2423;
  t2428 = t2296 + t2358 + t2426;
  t2445 = -0.134322983001*t2377;
  t2446 = 1. + t2445;
  t2447 = t2446*t2290;
  t2458 = -0.340999127418*t2377*t2357;
  t2461 = 0.366501*t242;
  t2462 = 0. + t2461;
  t2471 = t2462*t2423;
  t2478 = t2447 + t2458 + t2471;
  t2486 = -0.340999127418*t2377*t2290;
  t2488 = -0.8656776547239999*t2377;
  t2489 = 1. + t2488;
  t2490 = t2489*t2357;
  t2492 = 0.930418*t242;
  t2500 = 0. + t2492;
  t2502 = t2500*t2423;
  t2512 = t2486 + t2490 + t2502;
  t140 = Sin(var1[23]);
  t186 = 0.930418*t185;
  t196 = 0. + t186;
  t2432 = t196*t2428;
  t2480 = -0.340999127418*t2442*t2478;
  t2481 = -0.8656776547239999*t2442;
  t2482 = 1. + t2481;
  t2517 = t2482*t2512;
  t2518 = t2432 + t2480 + t2517;
  t128 = 0.120666640478*t126;
  t2524 = 0.366501*t185;
  t2525 = 0. + t2524;
  t2528 = t2525*t2428;
  t2529 = -0.134322983001*t2442;
  t2535 = 1. + t2529;
  t2536 = t2535*t2478;
  t2537 = -0.340999127418*t2442*t2512;
  t2540 = t2528 + t2536 + t2537;
  t2547 = -1.000000637725*t2442;
  t2556 = 1. + t2547;
  t2559 = t2556*t2428;
  t2561 = -0.366501*t185;
  t2562 = 0. + t2561;
  t2570 = t2562*t2478;
  t2571 = -0.930418*t185;
  t2572 = 0. + t2571;
  t2578 = t2572*t2512;
  t2581 = t2559 + t2570 + t2578;
  t149 = 0.803828*t140;
  t172 = t128 + t149;
  t2623 = t1258*t1353*t1281;
  t2624 = -1.*t617*t1358;
  t2626 = t2623 + t2624;
  t2635 = t617*t1258;
  t2636 = t1353*t1281*t1358;
  t2638 = t2635 + t2636;
  t2631 = -1.*t1178*t2626;
  t2641 = t1602*t2638;
  t2643 = t2631 + t2641;
  t2645 = t1602*t2626;
  t2646 = t1178*t2638;
  t2647 = t2645 + t2646;
  t2619 = t623*t943*t1353;
  t2644 = 0.340999127418*t1102*t2643;
  t2650 = t1662*t2647;
  t2652 = t2619 + t2644 + t2650;
  t2654 = t623*t1901*t1353;
  t2655 = t1910*t2643;
  t2657 = 0.340999127418*t1102*t2647;
  t2668 = t2654 + t2655 + t2657;
  t2685 = t2035*t623*t1353;
  t2689 = t2060*t2643;
  t2697 = t2108*t2647;
  t2699 = t2685 + t2689 + t2697;
  t2653 = t448*t2652;
  t2670 = t1878*t2668;
  t2715 = t2019*t2699;
  t2719 = t2653 + t2670 + t2715;
  t2725 = t2167*t2652;
  t2726 = -0.340999127418*t1981*t2668;
  t2729 = t2180*t2699;
  t2731 = t2725 + t2726 + t2729;
  t2744 = -0.340999127418*t1981*t2652;
  t2745 = t2262*t2668;
  t2747 = t2265*t2699;
  t2749 = t2744 + t2745 + t2747;
  t2723 = t267*t2719;
  t2743 = -0.340999127418*t2159*t2731;
  t2750 = t2215*t2749;
  t2751 = t2723 + t2743 + t2750;
  t2758 = t2322*t2719;
  t2760 = t2343*t2731;
  t2761 = -0.340999127418*t2159*t2749;
  t2762 = t2758 + t2760 + t2761;
  t2766 = t2398*t2719;
  t2767 = t2403*t2731;
  t2770 = t2412*t2749;
  t2771 = t2766 + t2767 + t2770;
  t2522 = -0.952469601425*t126;
  t2523 = 1. + t2522;
  t2754 = t247*t2751;
  t2763 = t2301*t2762;
  t2773 = t2387*t2771;
  t2776 = t2754 + t2763 + t2773;
  t2779 = t2446*t2751;
  t2781 = -0.340999127418*t2377*t2762;
  t2782 = t2462*t2771;
  t2792 = t2779 + t2781 + t2782;
  t2803 = -0.340999127418*t2377*t2751;
  t2804 = t2489*t2762;
  t2806 = t2500*t2771;
  t2810 = t2803 + t2804 + t2806;
  t2542 = 0.175248972904*t126;
  t2545 = -0.553471*t140;
  t2546 = t2542 + t2545;
  t2595 = -0.693671301908*t126;
  t2599 = 1. + t2595;
  t2778 = t196*t2776;
  t2795 = -0.340999127418*t2442*t2792;
  t2817 = t2482*t2810;
  t2819 = t2778 + t2795 + t2817;
  t2601 = -0.803828*t140;
  t2604 = t128 + t2601;
  t2821 = t2525*t2776;
  t2827 = t2535*t2792;
  t2828 = -0.340999127418*t2442*t2810;
  t2830 = t2821 + t2827 + t2828;
  t2607 = 0.444895486988*t126;
  t2610 = 0.218018*t140;
  t2611 = t2607 + t2610;
  t2839 = t2556*t2776;
  t2842 = t2562*t2792;
  t2843 = t2572*t2810;
  t2849 = t2839 + t2842 + t2843;
  t2906 = -1.*t623*t1258*t1178;
  t2908 = t1602*t623*t1358;
  t2909 = t2906 + t2908;
  t2916 = t1602*t623*t1258;
  t2921 = t623*t1178*t1358;
  t2924 = t2916 + t2921;
  t2905 = -1.*t943*t1281;
  t2914 = 0.340999127418*t1102*t2909;
  t2927 = t1662*t2924;
  t2929 = t2905 + t2914 + t2927;
  t2939 = -1.*t1901*t1281;
  t2941 = t1910*t2909;
  t2947 = 0.340999127418*t1102*t2924;
  t2949 = t2939 + t2941 + t2947;
  t2953 = -1.*t2035*t1281;
  t2957 = t2060*t2909;
  t2959 = t2108*t2924;
  t2962 = t2953 + t2957 + t2959;
  t2937 = t448*t2929;
  t2950 = t1878*t2949;
  t2966 = t2019*t2962;
  t2967 = t2937 + t2950 + t2966;
  t2971 = t2167*t2929;
  t2973 = -0.340999127418*t1981*t2949;
  t2975 = t2180*t2962;
  t2976 = t2971 + t2973 + t2975;
  t2982 = -0.340999127418*t1981*t2929;
  t2984 = t2262*t2949;
  t2985 = t2265*t2962;
  t2991 = t2982 + t2984 + t2985;
  t2968 = t267*t2967;
  t2980 = -0.340999127418*t2159*t2976;
  t2992 = t2215*t2991;
  t2993 = t2968 + t2980 + t2992;
  t2995 = t2322*t2967;
  t2997 = t2343*t2976;
  t2999 = -0.340999127418*t2159*t2991;
  t3000 = t2995 + t2997 + t2999;
  t3003 = t2398*t2967;
  t3004 = t2403*t2976;
  t3007 = t2412*t2991;
  t3008 = t3003 + t3004 + t3007;
  t2994 = t247*t2993;
  t3002 = t2301*t3000;
  t3012 = t2387*t3008;
  t3017 = t2994 + t3002 + t3012;
  t3023 = t2446*t2993;
  t3025 = -0.340999127418*t2377*t3000;
  t3026 = t2462*t3008;
  t3029 = t3023 + t3025 + t3026;
  t3031 = -0.340999127418*t2377*t2993;
  t3032 = t2489*t3000;
  t3033 = t2500*t3008;
  t3039 = t3031 + t3032 + t3033;
  t3018 = t196*t3017;
  t3030 = -0.340999127418*t2442*t3029;
  t3040 = t2482*t3039;
  t3041 = t3018 + t3030 + t3040;
  t3047 = t2525*t3017;
  t3048 = t2535*t3029;
  t3050 = -0.340999127418*t2442*t3039;
  t3056 = t3047 + t3048 + t3050;
  t3058 = t2556*t3017;
  t3061 = t2562*t3029;
  t3064 = t2572*t3039;
  t3069 = t3058 + t3061 + t3064;
  t2521 = t172*t2518;
  t2541 = t2523*t2540;
  t2583 = t2546*t2581;
  t2584 = t2521 + t2541 + t2583;
  t2600 = t2599*t2518;
  t2606 = t2604*t2540;
  t2613 = t2611*t2581;
  t2616 = t2600 + t2606 + t2613;
  t3108 = -0.218018*t140;
  t3111 = t2607 + t3108;
  t3123 = 0.553471*t140;
  t3124 = t2542 + t3123;
  t3130 = -0.353861996165*t126;
  t3131 = 1. + t3130;
  t2820 = t172*t2819;
  t2838 = t2523*t2830;
  t2851 = t2546*t2849;
  t2855 = t2820 + t2838 + t2851;
  t2868 = t2599*t2819;
  t2879 = t2604*t2830;
  t2880 = t2611*t2849;
  t2884 = t2868 + t2879 + t2880;
  t3043 = t172*t3041;
  t3057 = t2523*t3056;
  t3073 = t2546*t3069;
  t3078 = t3043 + t3057 + t3073;
  t3084 = t2599*t3041;
  t3090 = t2604*t3056;
  t3095 = t2611*t3069;
  t3104 = t3084 + t3090 + t3095;
  t3117 = t3111*t2518;
  t3125 = t3124*t2540;
  t3135 = t3131*t2581;
  t3136 = t3117 + t3125 + t3135;
  t3147 = t3111*t2819;
  t3149 = t3124*t2830;
  t3150 = t3131*t2849;
  t3155 = t3147 + t3149 + t3150;
  t3180 = t3111*t3041;
  t3183 = t3124*t3056;
  t3184 = t3131*t3069;
  t3198 = t3180 + t3183 + t3184;
  p_output1[0]=-0.930418*t2584 + 0.366501*t2616;
  p_output1[1]=-0.930418*t2855 + 0.366501*t2884;
  p_output1[2]=-0.930418*t3078 + 0.366501*t3104;
  p_output1[3]=-0.294604*t2584 - 0.747896*t2616 + 0.594863*t3136;
  p_output1[4]=-0.294604*t2855 - 0.747896*t2884 + 0.594863*t3155;
  p_output1[5]=-0.294604*t3078 - 0.747896*t3104 + 0.594863*t3198;
  p_output1[6]=0.218018*t2584 + 0.553471*t2616 + 0.803828*t3136;
  p_output1[7]=0.218018*t2855 + 0.553471*t2884 + 0.803828*t3155;
  p_output1[8]=0.218018*t3078 + 0.553471*t3104 + 0.803828*t3198;
}



void R_toe_roll_joint_right_src(double *p_output1, const double *var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
