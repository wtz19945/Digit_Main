/*
 * Automatically Generated from Mathematica.
 * Thu 10 Nov 2022 15:00:56 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RightToeBottomFront_src.h"

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
  double t467;
  double t1088;
  double t1241;
  double t1119;
  double t1244;
  double t866;
  double t967;
  double t1020;
  double t1412;
  double t1155;
  double t1247;
  double t1353;
  double t1041;
  double t1413;
  double t1428;
  double t1432;
  double t282;
  double t651;
  double t671;
  double t1402;
  double t1469;
  double t1490;
  double t1576;
  double t1608;
  double t1629;
  double t1927;
  double t1951;
  double t1958;
  double t674;
  double t694;
  double t696;
  double t1531;
  double t1552;
  double t1560;
  double t1655;
  double t1660;
  double t1782;
  double t1823;
  double t1859;
  double t1860;
  double t1863;
  double t1865;
  double t1876;
  double t1880;
  double t1966;
  double t2017;
  double t2031;
  double t2032;
  double t2035;
  double t2036;
  double t2045;
  double t2051;
  double t2057;
  double t2061;
  double t2113;
  double t2120;
  double t2132;
  double t211;
  double t237;
  double t289;
  double t458;
  double t1776;
  double t1778;
  double t1779;
  double t1899;
  double t1961;
  double t1963;
  double t2108;
  double t2109;
  double t2137;
  double t2139;
  double t2149;
  double t2167;
  double t2179;
  double t2180;
  double t2182;
  double t2188;
  double t2201;
  double t2202;
  double t2231;
  double t2238;
  double t2253;
  double t2256;
  double t2258;
  double t2259;
  double t2336;
  double t2339;
  double t2343;
  double t242;
  double t251;
  double t2111;
  double t2195;
  double t2198;
  double t2200;
  double t2260;
  double t2261;
  double t2285;
  double t2299;
  double t2304;
  double t2305;
  double t2316;
  double t2326;
  double t2330;
  double t2332;
  double t2348;
  double t2349;
  double t2351;
  double t2373;
  double t2379;
  double t2385;
  double t2387;
  double t2406;
  double t2408;
  double t2409;
  double t2414;
  double t2416;
  double t2417;
  double t105;
  double t111;
  double t113;
  double t154;
  double t197;
  double t213;
  double t214;
  double t2262;
  double t2278;
  double t2284;
  double t2333;
  double t2346;
  double t2347;
  double t2410;
  double t2411;
  double t2418;
  double t2422;
  double t2423;
  double t2424;
  double t2430;
  double t2433;
  double t2439;
  double t2441;
  double t2447;
  double t2456;
  double t2458;
  double t2460;
  double t2461;
  double t2462;
  double t2464;
  double t2465;
  double t202;
  double t207;
  double t2413;
  double t2442;
  double t2445;
  double t2446;
  double t2468;
  double t2470;
  double t2495;
  double t2507;
  double t2509;
  double t2517;
  double t2523;
  double t2525;
  double t2526;
  double t2527;
  double t2474;
  double t2537;
  double t2538;
  double t2545;
  double t2551;
  double t2552;
  double t2555;
  double t2559;
  double t2560;
  double t2568;
  double t2573;
  double t2588;
  double t148;
  double t160;
  double t167;
  double t2670;
  double t2671;
  double t2676;
  double t2678;
  double t2679;
  double t2680;
  double t2677;
  double t2682;
  double t2686;
  double t2688;
  double t2689;
  double t2701;
  double t2667;
  double t2687;
  double t2703;
  double t2705;
  double t2711;
  double t2715;
  double t2716;
  double t2717;
  double t2724;
  double t2726;
  double t2730;
  double t2732;
  double t2707;
  double t2718;
  double t2743;
  double t2745;
  double t2747;
  double t2758;
  double t2765;
  double t2766;
  double t2775;
  double t2779;
  double t2780;
  double t2804;
  double t2746;
  double t2767;
  double t2809;
  double t2810;
  double t2812;
  double t2819;
  double t2820;
  double t2826;
  double t2828;
  double t2839;
  double t2850;
  double t2855;
  double t2479;
  double t2492;
  double t2811;
  double t2827;
  double t2856;
  double t2862;
  double t2868;
  double t2870;
  double t2873;
  double t2882;
  double t2886;
  double t2889;
  double t2890;
  double t2899;
  double t2530;
  double t2531;
  double t2591;
  double t2592;
  double t2866;
  double t2885;
  double t2902;
  double t2903;
  double t2605;
  double t2606;
  double t2905;
  double t2906;
  double t2907;
  double t2908;
  double t2609;
  double t2610;
  double t2917;
  double t2918;
  double t2921;
  double t2923;
  double t2628;
  double t2635;
  double t2645;
  double t2647;
  double t2653;
  double t2655;
  double t2966;
  double t2968;
  double t2970;
  double t2972;
  double t2973;
  double t2974;
  double t2965;
  double t2971;
  double t2977;
  double t2979;
  double t2982;
  double t2983;
  double t2984;
  double t2988;
  double t2990;
  double t3001;
  double t3003;
  double t3008;
  double t2980;
  double t2989;
  double t3009;
  double t3013;
  double t3018;
  double t3019;
  double t3024;
  double t3026;
  double t3028;
  double t3030;
  double t3033;
  double t3044;
  double t3016;
  double t3027;
  double t3047;
  double t3051;
  double t3062;
  double t3073;
  double t3075;
  double t3086;
  double t3093;
  double t3094;
  double t3095;
  double t3100;
  double t3056;
  double t3090;
  double t3105;
  double t3108;
  double t3117;
  double t3118;
  double t3120;
  double t3121;
  double t3124;
  double t3128;
  double t3130;
  double t3137;
  double t3114;
  double t3123;
  double t3138;
  double t3139;
  double t3148;
  double t3153;
  double t3154;
  double t3157;
  double t3166;
  double t3177;
  double t3178;
  double t3179;
  double t2595;
  double t2608;
  double t2615;
  double t2616;
  double t2641;
  double t2652;
  double t2656;
  double t2659;
  double t2933;
  double t2934;
  double t2935;
  double t2939;
  double t2942;
  double t2943;
  double t2944;
  double t2947;
  double t3189;
  double t3194;
  double t3195;
  double t3197;
  double t3202;
  double t3205;
  double t3206;
  double t3216;
  double t2472;
  double t2529;
  double t2575;
  double t2578;
  double t2904;
  double t2914;
  double t2926;
  double t2927;
  double t3140;
  double t3160;
  double t3182;
  double t3183;
  t467 = Cos(var1[3]);
  t1088 = Cos(var1[5]);
  t1241 = Sin(var1[3]);
  t1119 = Sin(var1[4]);
  t1244 = Sin(var1[5]);
  t866 = Cos(var1[14]);
  t967 = -1.*t866;
  t1020 = 1. + t967;
  t1412 = Cos(var1[13]);
  t1155 = t467*t1088*t1119;
  t1247 = t1241*t1244;
  t1353 = t1155 + t1247;
  t1041 = Sin(var1[13]);
  t1413 = -1.*t1088*t1241;
  t1428 = t467*t1119*t1244;
  t1432 = t1413 + t1428;
  t282 = Sin(var1[15]);
  t651 = Cos(var1[4]);
  t671 = Sin(var1[14]);
  t1402 = -1.*t1041*t1353;
  t1469 = t1412*t1432;
  t1490 = t1402 + t1469;
  t1576 = t1412*t1353;
  t1608 = t1041*t1432;
  t1629 = t1576 + t1608;
  t1927 = Cos(var1[15]);
  t1951 = -1.*t1927;
  t1958 = 1. + t1951;
  t674 = -0.366501*t671;
  t694 = 0. + t674;
  t696 = t467*t651*t694;
  t1531 = 0.340999127418*t1020*t1490;
  t1552 = -0.134322983001*t1020;
  t1560 = 1. + t1552;
  t1655 = t1560*t1629;
  t1660 = t696 + t1531 + t1655;
  t1782 = 0.930418*t671;
  t1823 = 0. + t1782;
  t1859 = t467*t651*t1823;
  t1860 = -0.8656776547239999*t1020;
  t1863 = 1. + t1860;
  t1865 = t1863*t1490;
  t1876 = 0.340999127418*t1020*t1629;
  t1880 = t1859 + t1865 + t1876;
  t1966 = -1.000000637725*t1020;
  t2017 = 1. + t1966;
  t2031 = t2017*t467*t651;
  t2032 = -0.930418*t671;
  t2035 = 0. + t2032;
  t2036 = t2035*t1490;
  t2045 = 0.366501*t671;
  t2051 = 0. + t2045;
  t2057 = t2051*t1629;
  t2061 = t2031 + t2036 + t2057;
  t2113 = Cos(var1[16]);
  t2120 = -1.*t2113;
  t2132 = 1. + t2120;
  t211 = Sin(var1[17]);
  t237 = Sin(var1[16]);
  t289 = 0.930418*t282;
  t458 = 0. + t289;
  t1776 = t458*t1660;
  t1778 = 0.366501*t282;
  t1779 = 0. + t1778;
  t1899 = t1779*t1880;
  t1961 = -1.000000637725*t1958;
  t1963 = 1. + t1961;
  t2108 = t1963*t2061;
  t2109 = t1776 + t1899 + t2108;
  t2137 = -0.8656776547239999*t1958;
  t2139 = 1. + t2137;
  t2149 = t2139*t1660;
  t2167 = -0.340999127418*t1958*t1880;
  t2179 = -0.930418*t282;
  t2180 = 0. + t2179;
  t2182 = t2180*t2061;
  t2188 = t2149 + t2167 + t2182;
  t2201 = -0.340999127418*t1958*t1660;
  t2202 = -0.134322983001*t1958;
  t2231 = 1. + t2202;
  t2238 = t2231*t1880;
  t2253 = -0.366501*t282;
  t2256 = 0. + t2253;
  t2258 = t2256*t2061;
  t2259 = t2201 + t2238 + t2258;
  t2336 = Cos(var1[17]);
  t2339 = -1.*t2336;
  t2343 = 1. + t2339;
  t242 = 0.366501*t237;
  t251 = 0. + t242;
  t2111 = t251*t2109;
  t2195 = -0.340999127418*t2132*t2188;
  t2198 = -0.134322983001*t2132;
  t2200 = 1. + t2198;
  t2260 = t2200*t2259;
  t2261 = t2111 + t2195 + t2260;
  t2285 = 0.930418*t237;
  t2299 = 0. + t2285;
  t2304 = t2299*t2109;
  t2305 = -0.8656776547239999*t2132;
  t2316 = 1. + t2305;
  t2326 = t2316*t2188;
  t2330 = -0.340999127418*t2132*t2259;
  t2332 = t2304 + t2326 + t2330;
  t2348 = -1.000000637725*t2132;
  t2349 = 1. + t2348;
  t2351 = t2349*t2109;
  t2373 = -0.930418*t237;
  t2379 = 0. + t2373;
  t2385 = t2379*t2188;
  t2387 = -0.366501*t237;
  t2406 = 0. + t2387;
  t2408 = t2406*t2259;
  t2409 = t2351 + t2385 + t2408;
  t2414 = Cos(var1[18]);
  t2416 = -1.*t2414;
  t2417 = 1. + t2416;
  t105 = Cos(var1[19]);
  t111 = -1.*t105;
  t113 = 1. + t111;
  t154 = Sin(var1[19]);
  t197 = Sin(var1[18]);
  t213 = -0.366501*t211;
  t214 = 0. + t213;
  t2262 = t214*t2261;
  t2278 = -0.930418*t211;
  t2284 = 0. + t2278;
  t2333 = t2284*t2332;
  t2346 = -1.000000637725*t2343;
  t2347 = 1. + t2346;
  t2410 = t2347*t2409;
  t2411 = t2262 + t2333 + t2410;
  t2418 = -0.134322983001*t2343;
  t2422 = 1. + t2418;
  t2423 = t2422*t2261;
  t2424 = -0.340999127418*t2343*t2332;
  t2430 = 0.366501*t211;
  t2433 = 0. + t2430;
  t2439 = t2433*t2409;
  t2441 = t2423 + t2424 + t2439;
  t2447 = -0.340999127418*t2343*t2261;
  t2456 = -0.8656776547239999*t2343;
  t2458 = 1. + t2456;
  t2460 = t2458*t2332;
  t2461 = 0.930418*t211;
  t2462 = 0. + t2461;
  t2464 = t2462*t2409;
  t2465 = t2447 + t2460 + t2464;
  t202 = 0.930418*t197;
  t207 = 0. + t202;
  t2413 = t207*t2411;
  t2442 = -0.340999127418*t2417*t2441;
  t2445 = -0.8656776547239999*t2417;
  t2446 = 1. + t2445;
  t2468 = t2446*t2465;
  t2470 = t2413 + t2442 + t2468;
  t2495 = 0.366501*t197;
  t2507 = 0. + t2495;
  t2509 = t2507*t2411;
  t2517 = -0.134322983001*t2417;
  t2523 = 1. + t2517;
  t2525 = t2523*t2441;
  t2526 = -0.340999127418*t2417*t2465;
  t2527 = t2509 + t2525 + t2526;
  t2474 = 0.175248972904*t113;
  t2537 = -1.000000637725*t2417;
  t2538 = 1. + t2537;
  t2545 = t2538*t2411;
  t2551 = -0.366501*t197;
  t2552 = 0. + t2551;
  t2555 = t2552*t2441;
  t2559 = -0.930418*t197;
  t2560 = 0. + t2559;
  t2568 = t2560*t2465;
  t2573 = t2545 + t2555 + t2568;
  t2588 = 0.120666640478*t113;
  t148 = 0.444895486988*t113;
  t160 = -0.218018*t154;
  t167 = t148 + t160;
  t2670 = t1088*t1241*t1119;
  t2671 = -1.*t467*t1244;
  t2676 = t2670 + t2671;
  t2678 = t467*t1088;
  t2679 = t1241*t1119*t1244;
  t2680 = t2678 + t2679;
  t2677 = -1.*t1041*t2676;
  t2682 = t1412*t2680;
  t2686 = t2677 + t2682;
  t2688 = t1412*t2676;
  t2689 = t1041*t2680;
  t2701 = t2688 + t2689;
  t2667 = t651*t694*t1241;
  t2687 = 0.340999127418*t1020*t2686;
  t2703 = t1560*t2701;
  t2705 = t2667 + t2687 + t2703;
  t2711 = t651*t1823*t1241;
  t2715 = t1863*t2686;
  t2716 = 0.340999127418*t1020*t2701;
  t2717 = t2711 + t2715 + t2716;
  t2724 = t2017*t651*t1241;
  t2726 = t2035*t2686;
  t2730 = t2051*t2701;
  t2732 = t2724 + t2726 + t2730;
  t2707 = t458*t2705;
  t2718 = t1779*t2717;
  t2743 = t1963*t2732;
  t2745 = t2707 + t2718 + t2743;
  t2747 = t2139*t2705;
  t2758 = -0.340999127418*t1958*t2717;
  t2765 = t2180*t2732;
  t2766 = t2747 + t2758 + t2765;
  t2775 = -0.340999127418*t1958*t2705;
  t2779 = t2231*t2717;
  t2780 = t2256*t2732;
  t2804 = t2775 + t2779 + t2780;
  t2746 = t251*t2745;
  t2767 = -0.340999127418*t2132*t2766;
  t2809 = t2200*t2804;
  t2810 = t2746 + t2767 + t2809;
  t2812 = t2299*t2745;
  t2819 = t2316*t2766;
  t2820 = -0.340999127418*t2132*t2804;
  t2826 = t2812 + t2819 + t2820;
  t2828 = t2349*t2745;
  t2839 = t2379*t2766;
  t2850 = t2406*t2804;
  t2855 = t2828 + t2839 + t2850;
  t2479 = 0.553471*t154;
  t2492 = t2474 + t2479;
  t2811 = t214*t2810;
  t2827 = t2284*t2826;
  t2856 = t2347*t2855;
  t2862 = t2811 + t2827 + t2856;
  t2868 = t2422*t2810;
  t2870 = -0.340999127418*t2343*t2826;
  t2873 = t2433*t2855;
  t2882 = t2868 + t2870 + t2873;
  t2886 = -0.340999127418*t2343*t2810;
  t2889 = t2458*t2826;
  t2890 = t2462*t2855;
  t2899 = t2886 + t2889 + t2890;
  t2530 = -0.353861996165*t113;
  t2531 = 1. + t2530;
  t2591 = 0.803828*t154;
  t2592 = t2588 + t2591;
  t2866 = t207*t2862;
  t2885 = -0.340999127418*t2417*t2882;
  t2902 = t2446*t2899;
  t2903 = t2866 + t2885 + t2902;
  t2605 = -0.952469601425*t113;
  t2606 = 1. + t2605;
  t2905 = t2507*t2862;
  t2906 = t2523*t2882;
  t2907 = -0.340999127418*t2417*t2899;
  t2908 = t2905 + t2906 + t2907;
  t2609 = -0.553471*t154;
  t2610 = t2474 + t2609;
  t2917 = t2538*t2862;
  t2918 = t2552*t2882;
  t2921 = t2560*t2899;
  t2923 = t2917 + t2918 + t2921;
  t2628 = -0.693671301908*t113;
  t2635 = 1. + t2628;
  t2645 = -0.803828*t154;
  t2647 = t2588 + t2645;
  t2653 = 0.218018*t154;
  t2655 = t148 + t2653;
  t2966 = -1.*t651*t1088*t1041;
  t2968 = t1412*t651*t1244;
  t2970 = t2966 + t2968;
  t2972 = t1412*t651*t1088;
  t2973 = t651*t1041*t1244;
  t2974 = t2972 + t2973;
  t2965 = -1.*t694*t1119;
  t2971 = 0.340999127418*t1020*t2970;
  t2977 = t1560*t2974;
  t2979 = t2965 + t2971 + t2977;
  t2982 = -1.*t1823*t1119;
  t2983 = t1863*t2970;
  t2984 = 0.340999127418*t1020*t2974;
  t2988 = t2982 + t2983 + t2984;
  t2990 = -1.*t2017*t1119;
  t3001 = t2035*t2970;
  t3003 = t2051*t2974;
  t3008 = t2990 + t3001 + t3003;
  t2980 = t458*t2979;
  t2989 = t1779*t2988;
  t3009 = t1963*t3008;
  t3013 = t2980 + t2989 + t3009;
  t3018 = t2139*t2979;
  t3019 = -0.340999127418*t1958*t2988;
  t3024 = t2180*t3008;
  t3026 = t3018 + t3019 + t3024;
  t3028 = -0.340999127418*t1958*t2979;
  t3030 = t2231*t2988;
  t3033 = t2256*t3008;
  t3044 = t3028 + t3030 + t3033;
  t3016 = t251*t3013;
  t3027 = -0.340999127418*t2132*t3026;
  t3047 = t2200*t3044;
  t3051 = t3016 + t3027 + t3047;
  t3062 = t2299*t3013;
  t3073 = t2316*t3026;
  t3075 = -0.340999127418*t2132*t3044;
  t3086 = t3062 + t3073 + t3075;
  t3093 = t2349*t3013;
  t3094 = t2379*t3026;
  t3095 = t2406*t3044;
  t3100 = t3093 + t3094 + t3095;
  t3056 = t214*t3051;
  t3090 = t2284*t3086;
  t3105 = t2347*t3100;
  t3108 = t3056 + t3090 + t3105;
  t3117 = t2422*t3051;
  t3118 = -0.340999127418*t2343*t3086;
  t3120 = t2433*t3100;
  t3121 = t3117 + t3118 + t3120;
  t3124 = -0.340999127418*t2343*t3051;
  t3128 = t2458*t3086;
  t3130 = t2462*t3100;
  t3137 = t3124 + t3128 + t3130;
  t3114 = t207*t3108;
  t3123 = -0.340999127418*t2417*t3121;
  t3138 = t2446*t3137;
  t3139 = t3114 + t3123 + t3138;
  t3148 = t2507*t3108;
  t3153 = t2523*t3121;
  t3154 = -0.340999127418*t2417*t3137;
  t3157 = t3148 + t3153 + t3154;
  t3166 = t2538*t3108;
  t3177 = t2552*t3121;
  t3178 = t2560*t3137;
  t3179 = t3166 + t3177 + t3178;
  t2595 = t2592*t2470;
  t2608 = t2606*t2527;
  t2615 = t2610*t2573;
  t2616 = t2595 + t2608 + t2615;
  t2641 = t2635*t2470;
  t2652 = t2647*t2527;
  t2656 = t2655*t2573;
  t2659 = t2641 + t2652 + t2656;
  t2933 = t2592*t2903;
  t2934 = t2606*t2908;
  t2935 = t2610*t2923;
  t2939 = t2933 + t2934 + t2935;
  t2942 = t2635*t2903;
  t2943 = t2647*t2908;
  t2944 = t2655*t2923;
  t2947 = t2942 + t2943 + t2944;
  t3189 = t2592*t3139;
  t3194 = t2606*t3157;
  t3195 = t2610*t3179;
  t3197 = t3189 + t3194 + t3195;
  t3202 = t2635*t3139;
  t3205 = t2647*t3157;
  t3206 = t2655*t3179;
  t3216 = t3202 + t3205 + t3206;
  t2472 = t167*t2470;
  t2529 = t2492*t2527;
  t2575 = t2531*t2573;
  t2578 = t2472 + t2529 + t2575;
  t2904 = t167*t2903;
  t2914 = t2492*t2908;
  t2926 = t2531*t2923;
  t2927 = t2904 + t2914 + t2926;
  t3140 = t167*t3139;
  t3160 = t2492*t3157;
  t3182 = t2531*t3179;
  t3183 = t3140 + t3160 + t3182;
  p_output1[0]=0.993567*t2578 + 0.041508*t2616 + 0.105375*t2659;
  p_output1[1]=0.993567*t2927 + 0.041508*t2939 + 0.105375*t2947;
  p_output1[2]=0.993567*t3183 + 0.041508*t3197 + 0.105375*t3216;
  p_output1[3]=0.930418*t2616 - 0.366501*t2659;
  p_output1[4]=0.930418*t2939 - 0.366501*t2947;
  p_output1[5]=0.930418*t3197 - 0.366501*t3216;
  p_output1[6]=-0.113255*t2578 + 0.364143*t2616 + 0.924432*t2659;
  p_output1[7]=-0.113255*t2927 + 0.364143*t2939 + 0.924432*t2947;
  p_output1[8]=-0.113255*t3183 + 0.364143*t3197 + 0.924432*t3216;
}



void R_RightToeBottomFront_src(double *p_output1, const double *var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
