#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <SIM/iodefn.h>
#include <SIM/clocks.h>
#include <SIM/maths.h>
#include <SIM/weather.h>

#include "stab.h"
#include "aero.h"
#include "systems.h"
#include "aerolink.h"
#include "engines.h"
#include "fcs.h"
#include "model.h"

float Mu;
float Zw;
float Mw;
float MwDot;
float Mq;
float Zde;
float Mde;
float Xu;
float Xw;
float Zu;
float Xde;

float Yv;
float Lb;
float Lp;
float Lr;
float Nb;
float Np;
float Nr;
float Ydr;
float Lda;
float Ldr;
float Nda;
float Ndr;

float U;
float W;
float V;
float V2;
float rho;
float s;
float b;
float cbar;
float m;
float Ix;
float Iy;
float Iz;
float MachNo;
float g;

float a1;
float a2;
float a3;
float a4;

float b2;
float b1;
float b0;

float Cd_Alpha;
float Cl_Alpha;
float Cm_Alpha;

/* -------------------------------------------------------------------- */
void stability_derivatives()
{
    g      = 9.81;
    U      = Model_U;
    W      = Model_U * tan(Model_Alpha);
    V2     = U * U + W * W;
    V      = sqrt(V2);
    rho    = Weather_Rho;
    s      = Aero_s;
    b      = Aero_b;
    cbar   = Aero_CBar;
    m      = Aero_Mass;
    Ix     = Aero_Ixx;
    Iy     = Aero_Iyy;
    Iz     = Aero_Izz;
    MachNo = V / Weather_SpeedOfSound;

    Cl_Alpha = Aero_AeroCz1();
    Cd_Alpha = -0.12 + 2.0 * 3.0 * Model_Alpha;
    Cm_Alpha = Aero_AeroCmAlpha();

    Xu = -2.0 * Aero_AeroCd() * rho * V * s / (2.0 * m);
    Zu = -2.0 * Aero_AeroCl() * rho * V * s / (2.0 * m);
    Mu = MachNo * 0.22 * rho * V * s * cbar / (2.0 * Iy);

    Xw = -(Cd_Alpha - Aero_AeroCl()) * rho * V * s / (2.0 * m);
    Zw = -(Cl_Alpha + Aero_AeroCd()) * rho * V * s / (2.0 * m);
    Mw = Aero_AeroCmAlpha() * rho * V * s * cbar / (2.0 * Iy);

    MwDot = Aero_AeroCmAlphaDot() * rho * V * s * cbar * cbar / (4.0 * Iy);
    Mq    = Aero_AeroCmQ() * rho * V * s * cbar * cbar / (4.0 * Iy);

    Xde = 0.0;
    Mde = Aero_AeroCmDe() * rho * V2 * s * cbar / (2.0 * Iy);
    Zde = -Aero_AeroClTail() * rho * V2 * s / (2.0 * m);

    a1 = -Mq - V * Mw - Zw - Xu;
    a2 = -Zw * Mq - V * Mw - Xw * Zu + Xu * (Mq + V * Mw + Zw);
    a3 = -Xu * (Zw * Mq - V * Mw) + Zu * (Xw * Mq + g * Mw) - Mu * (V * Xw - g);
    a4 = g * (Zu * Mw - Mu * Zw);

    b2 = Mde + Zde * Mw;
    b1 = Xde * (Zu * Mw + Mu) + Zde * (Mw - Xu * Mw) - Mde * (Xu + Zw);
    b0 = Xde * (Zu * Mw - Zw * Mu) + Zde * (Mu * Xw - Mw * Xu) + Mde * (Zw * Xu - Xw * Zu);

    Yv  = Aero_AeroCyBeta() * rho * V * s / (2.0 * m);
    Lb  = Aero_AeroClBeta() * rho * V2 * s * b / (2.0 * Ix);
    Lp  = Aero_AeroClP() * rho * V * s * b * b / (4.0 * Ix);
    Lr  = Aero_AeroClR() * rho * V * s * b * b / (4.0 * Ix);
    Nb  = Aero_AeroCnBeta() * rho * V2 * s * b / (2.0 * Iz);
    Np  = Aero_AeroCnP() * rho * V * s * b * b / (4.0 * Iz);
    Nr  = Aero_AeroCnR() * rho * V * s * b * b / (4.0 * Iz);
    Ydr = Aero_AeroCyDr() * rho * V2 * s / (2.0 * m);
    Lda = Aero_AeroClDa() * rho * V2 * s * b / (2.0 * Ix);
    Ldr = Aero_AeroClDr() * rho * V2 * s * b / (2.0 * Ix);
    Nda = Aero_AeroCnDa() * rho * V2 * s * b / (2.0 * Iz);
    Ndr = Aero_AeroCnDr() * rho * V2 * s * b / (2.0 * Iz);

    printf("TAS=%5.1f m/s (%3.0f Kt) Mach=%3.2f Alt=%5.0f m (%5.0f ft)\n", V, V * 1.943844, MachNo, -Model_Pz, -Model_Pz * 3.280840);
    printf("Cxu=%f Cxw=%f\n", -2.0 * Aero_AeroCd(), -(Cd_Alpha - Aero_AeroCl()));
    printf("Czu=%f Czw=%f\n", -2.0 * Aero_AeroCl(), -(Cl_Alpha + Aero_AeroCd()));
    printf("Cmu=%f Cmw=%f CmwDot=%f Cmq=%f\n", MachNo * 0.22, Aero_AeroCmAlpha(), Aero_AeroCmAlphaDot(), Aero_AeroCmQ());

    printf("Xu=%f Xw=%f\n", Xu, Xw);
    printf("Zu=%f Zw=%f\n", Zu, Zw);
    printf("Mu=%f Mw=%f MwDot=%f Mq=%f\n", Mu, Mw, MwDot, Mq);
    printf("Xde=%f Mde=%f Zde=%f\n", Xde, Mde, Zde);

    printf("Yb=%f Lb=%f Lp=%f Lr=%f\n", Yv, Lb, Lp, Lr);
    printf("Nb=%f Np=%f Nr=%f\n", Nb, Np, Nr);
    printf("Ydr=%f Lda=%f Ldr=%f Nda=%f Ndr=%f\n", Ydr, Lda, Ldr, Nda, Ndr);

    printf("b2=%f b1=%f b0=%f\n", b2, b1, b0);
    printf("a1=%f a2=%f a3=%f a4=%f\n", a1, a2, a3, a4);
}

void BEGIN_Stab()
{
}
