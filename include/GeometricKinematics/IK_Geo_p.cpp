//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: IK_Geo_p.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 17-Jul-2022 20:41:51
//

// Include Files
#include "rt_nonfinite.h"
#include "FK_Geo_p.h"
#include "FK_Geo_v.h"
#include "IK_Geo_p.h"
#include "IK_Geo_v.h"
#include "J_FK_Geo_p.h"
#include "J_IK_Geo_p.h"
#include "dJ_FK_Geo_p.h"
#include "dJ_IK_Geo_p.h"

// Function Definitions

//
// IK_GEO_P
//     [QFLEXION,QKNEE] = IK_GEO_P(LA,LL)
// Arguments    : double LA
//                double LL
//                double *qFlexion
//                double *qKnee
// Return Type  : void
//
void IK_Geo_p(double LA, double LL, double *qFlexion, double *qKnee)
{
  double t2;

  //     This function was generated by the Symbolic Math Toolbox version 8.0.
  //     17-Jul-2022 20:28:12
  t2 = LL * LL;
  *qFlexion = (LA + std::acos((t2 * 0.944822373393802 + 0.02839440665154953) /
    LL)) - 0.1;
  *qKnee = -std::acos(t2 * 1.8896447467876041 - 1.0016111866969011) - 0.035;
}

//
// File trailer for IK_Geo_p.cpp
//
// [EOF]
//
