#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include <SIM/maths.h>
#include <SIM/weather.h>

#include "aero.h"
#include "aerolink.h"
#include "model.h"
#include "fcs.h"
#include "iolib.h"

bool  FCS_APSpeedMode;
float FCS_ThrottlePosition;
bool  FCS_AutoTrimming;
float FCS_Kp;
float FCS_Ki;
float FCS_Kd;
int   FCS_FD_VBar;
int   FCS_FD_HBar;
float SPD_s;

#define DEG1                           (1.0 / ONERAD)
#define DEG3                           (3.0 / ONERAD)
#define DEG20                          (20.0 / ONERAD)
#define DEG25                          (25.0 / ONERAD)
#define DEG30                          (30.0 / ONERAD)
#define DEG40                          (40.0 / ONERAD)
#define DEG60                          (60.0 / ONERAD)
#define DEG85                          (85.0 / ONERAD)
#define DEG95                          (95.0 / ONERAD)

#define NumberOfEngines                4

#define GainRollRate                   (15.0 / ONERAD)
#define MinBankAngle                   (5.0 / ONERAD)
#define MarginAOA                      (3.0 / ONERAD)
#define MinPitch                       (-10.0 / ONERAD)
#define MaxPitch                       (30.0 / ONERAD)
#define MarginMinPitch                 (5.0 / ONERAD)
#define MarginMaxPitch                 (5.0 / ONERAD)
#define BankProtectionAngle            (33.0 / ONERAD)
#define BankMaxAngle                   (67.0 / ONERAD)
#define BankMaxAngleWithAOAP           (45.0 / ONERAD)
#define GainPitchProtection            1.0
#define GainPitchProtectionNegative    1.2
#define GainAlphaProtection            2.0
#define GainRollProtection             1.0
#define TimeSmoothTransition           50
#define OffsetElevators                2.0
#define SaturateRudder                 3.6
#define YawDamperEngaged               true
#define TurnCoordinatorEngaged         true

#define NumberOfEngines                4

static float        APAlt;
static bool         APAltMode;
static float        APPitch;
static bool         APPitchMode;
static float        APHdg;
static bool         APHdgMode;
static float        APSpeed;
static bool         APSpeedMode;
static float        APVS;
static bool         APVSMode;
static bool         AutolandMode;
static unsigned int OldSpeed;
static unsigned int OldHdg;
static unsigned int OldAlt;
static int          OldVS;
static bool         OldMetric;
static float        de_fcs;
static bool         AoaProtection;
static bool         PitchProtection;
static bool         RollProtection;
static float        PitchRate_s;
static float        PitchRate_eold;
static float        RollRate_s;
static float        RollRate_eold;
static float        Beta_s;
static float        Beta_eold;
static float        Beta_lastValue;
static float        Nz_s;
static float        PitchTrimRemembered;
static float        PitchRememberedValue;
static bool         FlareMode;
static unsigned int counterFlare;
static float        YawRate_t;
static float        Roll_t;
static float        RollRate_t;
static float        YawDamper_t;
static float        YawRate_d;
static float        Roll_d;
static float        RollRate_d;
static float        YawRate_a;
static float        Roll_a;
static float        RollRate_a;
static float        delta_rc;
static float        YawDamper_d;
static float        dr_a_lastValue;
static float        dr_a;
static float        dr_c;
static float        dr_t;

static float        fdhx;
static float        fdhy;

static float        ydamp    = 0.0;
static float        ydampint = 0.0;

static float SetPositionStick(float offset, float semiBand, float input);
static void  FindProtection(float Aileron, float Elevator);
static float LongitudinalLaw(float Elevator, float GainPositive, float GainNegative);
static float AngleOfAttackProtection(float Alpha, float PitchRateC, float GainPositive);

#define AlphaMaxMinusAlphaProt    MarginAOA

static float LateralLaw(float Roll, float RollRateC, float Gain, float MAXBANKANGLE, float MAXEXCESSBANKANGLE);
static float NzLaw(float Elevator);

static float RollRateHold(float RollRateC);

static float BetaHold(float BetaC);
#define Beta_Kp         30.0
#define Beta_Ki         0.15
#define Beta_Kd         700.0

//static float TurnCoordinator(float *Rudder);
#define TurnC_Kp        3.0
#define TurnC_Ki        10.0
#define TurnC_Kd        0.0

static void NormalAirbusLaw(float *Elevator, float *Aileron, float *Rudder);
static void FlareModeAirbusLaw(float *Elevator, float *Aileron, float *Rudder);
static void UpdateFlightDirector(float Elevator, float Aileron, int *fdh, int *fdv);
static void CheckFCU();
float CheckFMS();
static float PitchHold(float ThetaC);

static float BankAngleHold(float BRef);
static float FPAHold(float GammaC);

void FCS_TurnCoordinator(float *Rudder);
void FCS_YawDamper(float *Rudder);

/* --------------------------------------------------------- */
static void UpdateFlightDirector(float Elevator, float Aileron, int *fdh, int *fdv)
{
    float de;
    float da;
    float dh;
    float dv;

    FCS_Autoland(&de, &da);

    dh = (Elevator - de) * 140.0;
    dh = Maths_Limit(dh, -140.0, 140.0);
    fdhx = Maths_Integrate(fdhx, 0.5 * (dh - fdhx));
    *fdh = intround(dh);
	
    dv   = (da - Aileron) * 140.0;
    dv = Maths_Limit(dv, -140.0, 140.0);
    fdhy = Maths_Integrate(fdhy, 0.5 * (dv - fdhy));
    *fdv = intround(dv);
}

/* --------------------------------------------------------- */
static float SetPositionStick(float offset, float semiBand, float input)
{
    float SlopeCalibration1;
    float SlopeCalibration2;

    SlopeCalibration1 = 1.0 / (1.0 - offset - semiBand);
    SlopeCalibration2 = 1.0 / (1.0 + offset - semiBand);
    if (input < offset + semiBand && input > offset - semiBand)
    {
        input = 0.0;
    }
    else if (input >= offset + semiBand)
    {
        input = SlopeCalibration1 * (input - offset - semiBand);
    }
    else
    {
        input = SlopeCalibration2 * (input - offset + semiBand);
    }
    return input;
}

/* --------------------------------------------------------- */
static void FindProtection(float Aileron, float Elevator)
{
    PitchProtection = Model_Pitch <= MinPitch + MarginMinPitch || Model_Pitch >= MaxPitch - MarginMaxPitch;
    RollProtection  = Model_Roll <= -BankProtectionAngle || Model_Roll >= BankProtectionAngle;
    AoaProtection   = Model_Alpha >= Aero_AeroMaxAlpha() - MarginAOA;
    if (AoaProtection)
    {
        RollProtection  = false;
        PitchProtection = false;
    }
}

/* --------------------------------------------------------- */
static float LongitudinalLaw(float Elevator, float GainPositive, float GainNegative)
{
    if (Model_Pitch >= (MinPitch + MarginMinPitch) && Model_Pitch <= (MaxPitch - MarginMaxPitch))
    {
        return Elevator;
    }
    
	else if (Model_Pitch > (MaxPitch - MarginMaxPitch) && Model_Pitch < MaxPitch)
    {
        return Elevator - GainPositive * (Model_Pitch - (MaxPitch - MarginMaxPitch)) / (MaxPitch - (MaxPitch - MarginMaxPitch));
    }
    
	else if (Model_Pitch < (MinPitch + MarginMinPitch) && Model_Pitch > MinPitch)
    {
        return Elevator + GainNegative * (Model_Pitch - (MinPitch + MarginMinPitch)) / (MinPitch - (MinPitch + MarginMinPitch));
    }
    
	else if (Model_Pitch >= MaxPitch)
    {
        if (Elevator > 0.0)
        {
            return -GainPositive * (Model_Pitch - (MaxPitch - MarginMaxPitch)) / (MaxPitch - (MaxPitch - MarginMaxPitch));
        }
        else
        {
            return Elevator - GainPositive * (Model_Pitch - (MaxPitch - MarginMaxPitch)) / (MaxPitch - (MaxPitch - MarginMaxPitch));
        }
    }
    
	else if (Model_Pitch <= -MinPitch)
    {
        if (Elevator < 0.0)
        {
            return GainNegative * (Model_Pitch - (MinPitch + MarginMinPitch)) / (MinPitch - (MinPitch + MarginMinPitch));
        }
        else
        {
            return Elevator + GainNegative * (Model_Pitch - (MinPitch + MarginMinPitch)) / (MinPitch - (MinPitch + MarginMinPitch));
        }
    }
    return 0.0;
}

/* --------------------------------------------------------- */
static float AngleOfAttackProtection(float Alpha, float PitchRateC, float GainPositive)
{
    float AlphaMax;
    float AlphaProt;

    AlphaMax  = Aero_AeroMaxAlpha();
    AlphaProt = AlphaMax - AlphaMaxMinusAlphaProt;

    if (Alpha <= AlphaProt)
    {
        return PitchRateC;
    }
    else if (Alpha > AlphaProt && Alpha < AlphaMax)
    {
        return PitchRateC - GainPositive * (Alpha - AlphaProt) / (AlphaMax - AlphaProt);
    }
    else if (Alpha >= AlphaMax)
    {
        if (PitchRateC > 0.0)
        {
            return -GainPositive * (Alpha - AlphaProt) / (AlphaMax - AlphaProt);
        }
        else
        {
            return PitchRateC - GainPositive * (Alpha - AlphaProt) / (AlphaMax - AlphaProt);
        }
    }
    return 0.0;
}

/* --------------------------------------------------------- */
static float LateralLaw(float Roll, float RollRateC, float Gain, float MAXBANKANGLE, float MAXEXCESSBANKANGLE)
{
    if (Roll >= -MAXBANKANGLE && Roll <= MAXBANKANGLE)
    {
        return RollRateC;
    }
    else if (Roll > MAXBANKANGLE && Roll < MAXEXCESSBANKANGLE)
    {
        return RollRateC - Gain * (Roll - MAXBANKANGLE) / (MAXEXCESSBANKANGLE - MAXBANKANGLE);
    }
    else if (Roll < -MAXBANKANGLE && Roll > -MAXEXCESSBANKANGLE)
    {
        return RollRateC - Gain * (Roll + MAXBANKANGLE) / (MAXEXCESSBANKANGLE - MAXBANKANGLE);
    }
    else if (Roll >= MAXEXCESSBANKANGLE)
    {
        if (RollRateC > 0.0)
        {
            return -Gain * (Roll - MAXBANKANGLE) / (MAXEXCESSBANKANGLE - MAXBANKANGLE);
        }
        else
        {
            return RollRateC - Gain * (Roll - MAXBANKANGLE) / (MAXEXCESSBANKANGLE - MAXBANKANGLE);
        }
    }
    else if (Roll <= -MAXEXCESSBANKANGLE)
    {
        if (RollRateC < 0.0)
        {
            return -Gain * (Roll + MAXBANKANGLE) / (MAXEXCESSBANKANGLE - MAXBANKANGLE);
        }
        else
        {
            return RollRateC - Gain * (Roll + MAXBANKANGLE) / (MAXEXCESSBANKANGLE - MAXBANKANGLE);
        }
    }
    return 0.0;
}

/* --------------------------------------------------------- */
static float NzLaw(float Elevator)
{
    const float K1 = 10.0;
    const float K2 = 0.5;   // was 0.3
    const float K3 = -0.8;  // was 0.3
	const float K4 = 1.0;   // was 1.0
    const float K5 = 10.0;  // was 5.0

    float c = cos(Model_Pitch) / cos(Model_Roll);
	float dPitch = Model_Q * cos(Model_Roll) - Model_R * sin(Model_Roll);
    float Nz = (Model_Lift * cos(Model_Alpha) + Model_Drag * sin(Model_Alpha)) / (Aero_Mass * Model_G);
    float Nzc = -Elevator + c - K1 * dPitch;
    float dNz;
	
	Nzc = Maths_Limit(Nzc, -1.0, 2.5);
    dNz = Nzc - Nz;
    Nz_s = Maths_Integrate(Nz_s, dNz * K3);
    Elevator = Elevator * K2 + Nz_s + (Nz - c) * K4 + dPitch * K5;
    
    return Elevator;
}

/* --------------------------------------------------------- */
static float RollRateHold(float RollRateC)
{
    const float RollRate_Kp = 30.0;
    const float RollRate_Ki = 0.7;
    const float RollRate_Kd = -15.0;
    const float RollRate_in = 0.25;
	
    float da;
    float olds = RollRate_s;
    float e = RollRateC * RollRate_in - Model_P;

    RollRate_s = RollRate_s + e;
    da = e * RollRate_Kp + RollRate_s * RollRate_Ki  + (e - RollRate_eold) * RollRate_Kd;
    RollRate_eold = e;

    if (da >= 1.0)
    {
        da         = 1.0;
        RollRate_s = olds;
    }
    else if (da <= -1.0)
    {
        da         = -1.0;
        RollRate_s = olds;
    }
    return da;
}

/* --------------------------------------------------------- */
static float BetaHold(float BetaC)
{
    float dr;
    float e;
    float olds;

    olds      = Beta_s;
    e         = BetaC - Model_Beta;
    Beta_s    = Beta_s + e;
    dr        = Beta_Kp * e + Beta_Ki * Beta_s + Beta_Kd * (e - Beta_eold);
    Beta_eold = e;
    dr = Maths_Limit(dr, Beta_lastValue - 0.3 / (Aero_RudderGain * ONERAD), Beta_lastValue + 0.3 / (Aero_RudderGain * ONERAD));
    Beta_lastValue = dr;
    if (dr >= SaturateRudder / (Aero_RudderGain * ONERAD))
    {
        dr     = SaturateRudder / (Aero_RudderGain * ONERAD);
        Beta_s = olds;
    }
    else if (dr <= -SaturateRudder / (Aero_RudderGain * ONERAD))
    {
        dr     = -SaturateRudder / (Aero_RudderGain * ONERAD);
        Beta_s = olds;
    }
    return dr;
}

/* --------------------------------------------------------- */
void FCS_TurnCoordinator(float *Rudder)
{
    *Rudder = -Model_Beta * 20.0;
}

/* --------------------------------------------------------- */
void FCS_YawDamper(float *Rudder)
{
    ydampint = Maths_Integrate(ydampint, ydamp);
    ydamp   = Model_R * 20.0 - ydampint / 1.0;
    *Rudder = *Rudder + ydamp;
}

/* --------------------------------------------------------- */
float oldTurnCoordinator(float Rudder)
{
    YawRate_t = Maths_Integrate(YawRate_t, 1.354 * YawRate_d);
    YawRate_d = Maths_Integrate(YawRate_d, 4.044 * Model_R * ONERAD - 4.048 * YawRate_d - YawRate_t);
    Roll_t = Maths_Integrate(Roll_t, 1.354 * Roll_d);
    Roll_d = Maths_Integrate(Roll_d, 4.044 * Model_Roll * ONERAD - 4.048 * Roll_d - Roll_t);
    RollRate_t = Maths_Integrate(RollRate_t, 100.0 * RollRate_d);
    RollRate_d = Maths_Integrate(RollRate_d, 100.0 * Model_P * ONERAD - 20.0 * RollRate_d - RollRate_t);
    if (Model_Flaps > 0.001)
    {
        YawRate_a = YawRate_d * 2.5;
    }
    else
    {
        YawRate_a = YawRate_d * 1.25;
    }
    Roll_a     = Roll_d * 2.5 * 9.81 / 240.0;
    RollRate_a = RollRate_d * 0.69;
    delta_rc   = TurnC_Kp * YawRate_a - TurnC_Ki * Roll_a + TurnC_Kd * RollRate_a;
    YawDamper_t = Maths_Integrate(YawDamper_t, 0.0187 * delta_rc - 3.2 * YawDamper_d);
    YawDamper_d = Maths_Integrate(YawDamper_d, 1.56 * delta_rc - 32.1 * YawDamper_d + YawDamper_t);
    dr_a = Maths_Integrate(dr_a, 402.0 * YawDamper_d - 20.0 * dr_a);
    if (!YawDamperEngaged)
    {
        dr_a = 0.0;
    }
    if (TurnCoordinatorEngaged)
    {
        dr_a = dr_a + BetaHold(0.0) * ONERAD * Aero_RudderGain;
    }
    dr_a = Maths_Limit(dr_a, dr_a_lastValue - 0.3, dr_a_lastValue + 0.3);
    dr_a = Maths_Limit(dr_a, -SaturateRudder, SaturateRudder);
    dr_a_lastValue = dr_a;
    dr_c = Maths_Integrate(dr_c, 402.0 * Rudder * 0.05 * ONERAD * Aero_RudderGain - 20.0 * dr_c);
    dr_t = dr_c + dr_a;
    dr_t = Maths_Limit(dr_t, -ONERAD * Aero_RudderGain, ONERAD * Aero_RudderGain);
    return dr_t / (ONERAD * Aero_RudderGain);
}

/* --------------------------------------------------------- */
static void NormalAirbusLaw(float *Elevator, float *Aileron, float *Rudder)
{
    float r1 = *Rudder;
	float r2 = *Rudder;
	
	FindProtection(*Elevator, *Aileron);
    if (PitchProtection)
    {
        *Elevator = -LongitudinalLaw(-*Elevator, GainPitchProtection, GainPitchProtectionNegative);
    }
    if (RollProtection)
    {
        *Aileron = LateralLaw(Model_Roll, *Aileron, GainRollProtection, BankProtectionAngle, BankMaxAngle);
    }
    if (AoaProtection)
    {
        *Elevator = -AngleOfAttackProtection(Model_Alpha, -*Elevator, GainAlphaProtection);
        *Aileron  = LateralLaw(Model_Roll, *Aileron, GainRollProtection, BankProtectionAngle, BankMaxAngleWithAOAP);
    }
    *Elevator = NzLaw(*Elevator);
    *Aileron  = RollRateHold(*Aileron);

	if (YawDamperEngaged)
	{
	    FCS_YawDamper(&r1);
	}
	if (TurnCoordinatorEngaged)
	{
	    FCS_TurnCoordinator(&r2);
	}
	if (fabs(r1) > fabs(r2))
	{
	    *Rudder = r1;
	}
	else
	{
	    *Rudder = r2;
	}
	
    FlareMode = true;
    Model_Autotrim(&PitchTrimRemembered);
}

/* --------------------------------------------------------- */
static void FlareModeAirbusLaw(float *Elevator, float *Aileron, float *Rudder)
{
    float r;
	
    FCS_TurnCoordinator(&r);
	*Rudder = r;
	
    if (Model_Pz <= Aero_CGHeight + AeroLink_NavPkt.GroundLevel - Maths_Metres(30.0) && Model_Pz > Aero_CGHeight + AeroLink_NavPkt.GroundLevel - Maths_Metres(50.0))
    {
        *Elevator            = *Elevator + PitchTrimRemembered;
        PitchRememberedValue = Model_Pitch;
        counterFlare         = 0;
    }
    else if (!Model_OnTheGround)
    {
        if (FlareMode)
        {
            if (counterFlare < TimeSmoothTransition)
            {
                *Elevator    = *Elevator + PitchTrimRemembered + 0.5 * (PitchRememberedValue + OffsetElevators / ONERAD) / Aero_ElevatorGain / (float) TimeSmoothTransition * (float) counterFlare;
                counterFlare = counterFlare + 1;
            }
            else
            {
                *Elevator = *Elevator + PitchTrimRemembered + 0.5 * (PitchRememberedValue + OffsetElevators / ONERAD) / Aero_ElevatorGain;
            }
        }
    }
    else
    {
        FlareMode = false;
    }
}

/* --------------------------------------------------------- */
static void CheckFCU()
{
    bool NewSpeedMode;
    bool NewHdgMode;
    bool NewAltMode;
    bool NewVSMode;
    int  at_speed;

    NewSpeedMode = (AeroLink_NavPkt.FCU_AP1 | AeroLink_NavPkt.FCU_AP2) && AeroLink_NavPkt.FCU_ATHR && AeroLink_NavPkt.FCU_SPD_Hold;
    if (APSpeedMode != NewSpeedMode)
    {
        if (NewSpeedMode)
        {
            FCS_EngageSpeedHold();
        }
        else
        {
            FCS_DisengageSpeedHold();
        }
        APSpeedMode = NewSpeedMode;
		FCS_APSpeedMode = APSpeedMode;  /* needed for engines module */
    }
	if (AeroLink_NavPkt.FCU_SPD_MACH)
	{
    	at_speed = AeroLink_NavPkt.FCU_SPD;
	}
	else
	{
	    at_speed = (int) Weather_Mach_to_Kts(-Model_Pz, (float) AeroLink_NavPkt.FCU_SPD / 100.0);
	}
    if (OldSpeed != at_speed)
    {
        FCS_SetSpeedHold((float) (at_speed) / 1.944);
        OldSpeed = at_speed;
    }
	
    NewHdgMode = AeroLink_NavPkt.FCU_HDG_Hold;
    if (APHdgMode != NewHdgMode)
    {
        if (NewHdgMode)
        {
            FCS_EngageHeadingHold();
        }
        else
        {
            FCS_DisengageHeadingHold();
        }
        APHdgMode = NewHdgMode;
    }
    if (OldHdg != AeroLink_NavPkt.FCU_HDG)
    {
        FCS_SetHeadingHold(Maths_Rads((float) (AeroLink_NavPkt.FCU_HDG)));
        OldHdg = AeroLink_NavPkt.FCU_HDG;
    }
	
    NewAltMode = AeroLink_NavPkt.FCU_ALT_Hold;
    if (APAltMode != NewAltMode)
    {
        if (NewAltMode)
        {
            FCS_EngageHeightHold();
        }
        else
        {
            FCS_DisengageHeightHold();
        }
        APAltMode = NewAltMode;
    }
    if ((OldAlt != AeroLink_NavPkt.FCU_ALT) || (OldMetric != AeroLink_NavPkt.FCU_Metric_ALT))
    {
	    if (AeroLink_NavPkt.FCU_Metric_ALT)
		{
            FCS_SetHeightHold((float) (-AeroLink_NavPkt.FCU_ALT));
		}
		else
		{
            FCS_SetHeightHold(Maths_Metres((float) (-AeroLink_NavPkt.FCU_ALT)));
		}
        OldAlt = AeroLink_NavPkt.FCU_ALT;
		OldMetric = AeroLink_NavPkt.FCU_Metric_ALT;
    }
	
    NewVSMode = AeroLink_NavPkt.FCU_VS_Hold;
    if (APVSMode != NewVSMode)
    {
        if (NewVSMode)
        {
            FCS_EngageVSpeedHold();
        }
        else
        {
            FCS_DisengageVSpeedHold();
        }
        APVSMode = NewVSMode;
    }
    if (OldVS != AeroLink_NavPkt.FCU_VS)
    {
        FCS_SetVSpeedHold((float) (AeroLink_NavPkt.FCU_VS) * 0.508);  /* FPM * 100 -> m/s */
        OldVS = AeroLink_NavPkt.FCU_VS;
    }
	
    if (AutolandMode != AeroLink_NavPkt.FCU_APPR)
    {
        //printf("AutolandMode : %d AeroLink_NavPkt.APPR : %d\n", AutolandMode, AeroLink_NavPkt.APPR);
        //printf("ModelLat : %f ModelLong : %f\n", Model_Latitude, Model_Longitude);
        if (AeroLink_NavPkt.FCU_APPR)
        {
            FCS_EngageAutoland();
        }
        else
        {
            FCS_DisengageAutoland();
        }
    }
}

/* --------------------------------------------------------- */
void FCS_ResetFCS()
{
    PitchRate_s          = 0.0;
    PitchRate_eold       = 0.0;
    RollRate_s           = 0.0;
    RollRate_eold        = 0.0;
    Beta_s               = 0.0;
    Beta_eold            = 0.0;
    Nz_s                 = 0.0;
    YawRate_t            = 0.0;
    Roll_t               = 0.0;
    RollRate_t           = 0.0;
    YawDamper_t          = 0.0;
    YawRate_d            = 0.0;
    Roll_d               = 0.0;
    RollRate_d           = 0.0;
    YawDamper_d          = 0.0;
    dr_a_lastValue       = 0.0;
    dr_a                 = 0.0;
    dr_c                 = 0.0;
    dr_t                 = 0.0;
    AoaProtection        = false;
    PitchProtection      = false;
    RollProtection       = false;
    PitchTrimRemembered  = 0.0;
    PitchRememberedValue = 0.0;
    FlareMode            = false;
    counterFlare         = 0;
    if (APAltMode || APPitchMode || APVSMode || AutolandMode)
    {
        Model_Pitch = Model_Autotrim(&de_fcs);
    }
}

/* --------------------------------------------------------- */
void FCS_FCS(float *Elevator, float *Aileron, float *Rudder)
{
    if (*Elevator >= -0.005 && *Elevator <= 0.005)
    {
        *Elevator = 0.0;
    }
    if (*Aileron >= -0.005 && *Aileron <= 0.005)
    {
        *Aileron = 0.0;
    }
    *Rudder = SetPositionStick(0.005, 0.035, *Rudder);

    CheckFCU();

    if (Model_Pz <= Aero_CGHeight + AeroLink_NavPkt.GroundLevel - 0.001 - Maths_Metres(50.0))
    {
        NormalAirbusLaw(Elevator, Aileron, Rudder);
    }
    else
    {
        FlareModeAirbusLaw(Elevator, Aileron, Rudder);
    }

    if (APAltMode)
    {
        *Elevator = FCS_HeightHold(APAlt);
    }

    if (AeroLink_NavPkt.WayPoint.BeaconStatus)
    {   /* flight plan active? */
        *Aileron = CheckFMS();
    }
    else if (AeroLink_NavPkt.FCU_LOC)
    {
        *Aileron = FCS_LOCHold();
    }
    else if (APHdgMode)
    {
        *Aileron = FCS_HeadingHold(APHdg);
    }

    if (APSpeedMode)
    {
        FCS_ThrottlePosition = FCS_SpeedHold(APSpeed);
    }

    if (APVSMode)
    {
        *Elevator = FCS_VSpeedHold(APVS);
    }

    if (AutolandMode)
    {
        FCS_Autoland(Elevator, Aileron);
    }

    if (AeroLink_NavPkt.FCU_FD)
    {
        UpdateFlightDirector(*Elevator, *Aileron, &FCS_FD_HBar, &FCS_FD_VBar);
    }
}

/* --------------------------------------------------------- */
void FCS_EngagePitchHold()
{
    APPitchMode = true;
    FCS_DisengageVSpeedHold();
    FCS_DisengageAutoland();
}

/* --------------------------------------------------------- */
void FCS_DisengagePitchHold()
{
    APPitchMode = false;
}

/* --------------------------------------------------------- */
bool FCS_PitchHoldEngaged()
{
    return APPitchMode;
}

/* --------------------------------------------------------- */
static float PitchHold(float ThetaC)
{
    const float Pitch_Kp = 0.4;
    const float Pitch_Ki = 5.0;

    float Qc;
    float dedot;

    Qc    = (ThetaC - Model_Pitch) * Pitch_Kp;
    dedot = -Pitch_Ki * (Qc - Model_Q);
    de_fcs = Maths_Integrate(de_fcs, dedot);
    de_fcs = Maths_Limit(de_fcs, -1.0, 1.0);
    return de_fcs;
}

/* --------------------------------------------------------- */
void FCS_SetPitchHold(float Theta)
{
    APPitch = Theta;
}

/* --------------------------------------------------------- */
void FCS_EngageAutoland()
{
    AutolandMode = true;
    FCS_DisengageHeightHold();
    FCS_DisengageHeadingHold();
    FCS_DisengageVSpeedHold();
}

/* --------------------------------------------------------- */
void FCS_DisengageAutoland()
{
    AutolandMode = false;
}

/* --------------------------------------------------------- */
bool FCS_AutolandEngaged()
{
    return AutolandMode;
}

/* --------------------------------------------------------- */
void FCS_Autoland(float *Elevator, float *Aileron)
{
    float Qdm;
    float h;
    float fpa;
    float gserr;

    //printf("ILSBeacon : %d AL RWQDM : %f\n", AeroLink_NavPkt.ILS1.ILSBeacon, AeroLink_NavPkt.ILS1.RunwayQdm);

    if (AeroLink_NavPkt.ILS1.ILSBeacon)
    {
        Qdm = (float) AeroLink_NavPkt.ILS1.RunwayQdm;
        Qdm = Maths_Normalise(Qdm);
    }
    else
    {
        return;
    }

    h = -Model_Pz + (float) AeroLink_NavPkt.GroundLevel + Aero_CGHeight;
    if (h < 15.0)
    {
        *Aileron = FCS_HeadingHold(Qdm + 1.0 * (float) AeroLink_NavPkt.ILS1.LocaliserError);
        if (Model_OnTheGround)
        {
            *Elevator = PitchHold(Model_Pitch * 0.5);
        }
        else
        {
            *Elevator = FPAHold(-0.005);
        }
    }
    else
    {
        gserr = (float) (AeroLink_NavPkt.ILS1.GlideSlopeError);
        gserr = Maths_Limit(gserr, -DEG1, DEG1);
        fpa = -DEG3 + 5.0 * gserr;
        if (fpa > 0.0)
        {
            fpa = 0.0;
        }
        *Aileron  = FCS_HeadingHold(Qdm + 10.0 * (float) AeroLink_NavPkt.ILS1.LocaliserError);
        *Elevator = FPAHold(fpa);
    }
}

/* --------------------------------------------------------- */
void FCS_EngageHeightHold()
{
    APAltMode = true;
    FCS_DisengageVSpeedHold();
    FCS_DisengageAutoland();
}

/* --------------------------------------------------------- */
void FCS_DisengageHeightHold()
{
    APAltMode = false;
}

/* --------------------------------------------------------- */
bool FCS_HeightHoldEngaged()
{
    return APAltMode;
}

/* --------------------------------------------------------- */
void FCS_SetHeightHold(float Href)
{
    APAlt = Href;
}

/* --------------------------------------------------------- */
float FCS_HeightHold(float Href)
{
    float vs;
    float h;

    h  = Model_Pz - Aero_CGHeight;
    vs = -(Href - h) * 0.083333;
    if (vs > 5.08)
    {
        vs = 5.08;
    }
    else if (vs < -5.08)
    {
        vs = -5.08;
    }
    return FCS_VSpeedHold(vs);
}

/* --------------------------------------------------------- */
void FCS_EngageHeadingHold()
{
    APHdgMode = true;
    FCS_DisengageAutoland();
}

/* --------------------------------------------------------- */
void FCS_DisengageHeadingHold()
{
    APHdgMode = false;
}

/* --------------------------------------------------------- */
bool FCS_HeadingHoldEngaged()
{
    return APHdgMode;
}

/* --------------------------------------------------------- */
void FCS_SetHeadingHold(float Href)
{
    APHdg = Href;
}

/* --------------------------------------------------------- */
float FCS_HeadingHold(float HdgRef)
{
    float dHdg;
    float trc;

    HdgRef = HdgRef + (float) (AeroLink_NavPkt.MagneticVariation);
    dHdg   = HdgRef - Model_Yaw;
    dHdg = Maths_Normalise(dHdg);
    trc = dHdg * 0.15;  /* 20 deg -> 3 deg/s */
    if (trc > DEG3)
    {
        trc = DEG3;
    }
    else if (trc < -DEG3)
    {
        trc = -DEG3;
    }
    return BankAngleHold(trc * Model_U / 9.81);
}

/* --------------------------------------------------------- */
float CheckFMS()
{
    float h;
    float dh;
    float error;

    h     = AeroLink_NavPkt.WayPoint.RunwayQdm;
    error = AeroLink_NavPkt.WayPoint.LocaliserError;

    dh = error * 6.0;
    dh = Maths_Limit(dh, -DEG60, DEG60);
    h = h + dh;
    h = Maths_Normalise(h);
    return FCS_HeadingHold(h);
}

/* --------------------------------------------------------- */
float FCS_LOCHold()
{
    float h;
    float dh;
    float error;

    h     = Maths_Rads((float) AeroLink_NavPkt.HSI_Crs);
    error = AeroLink_NavPkt.NAV1.LocaliserError;

    if ((error >= -DEG85) && (error <= DEG85))   /* TO */
    {
        /* do nothing */
    }
    else if (error <= -DEG95)   /* FROM */
    {
        error = -M_PI - error;
    }
    else if (error >= DEG95)    /* FROM */
    {
        error = M_PI - error;
    }
    else
    {
        error = 0.0;  /* cone of confusion */
    }

    dh = error * 6.0;
    dh = Maths_Limit(dh, -DEG60, DEG60);
    h = h + dh;
    h = Maths_Normalise(h);
    return FCS_HeadingHold(h);
}

/* --------------------------------------------------------- */
void FCS_EngageSpeedHold()
{
    APSpeedMode = true;
}

/* --------------------------------------------------------- */
void FCS_DisengageSpeedHold()
{
    APSpeedMode = false;
}

/* --------------------------------------------------------- */
bool FCS_SpeedHoldEngaged()
{
    return APSpeedMode;
}

/* --------------------------------------------------------- */
void FCS_SetSpeedHold(float Vref)
{
    APSpeed = Vref;
}

/* --------------------------------------------------------- */
float FCS_SpeedHold(float Vref)
{
    const float SPD_Kp    = 10.0;
    const float SPD_Ki    = 0.0;
    const float SPD_Kudot = 2.0;

    float       tp;
    float       e;
    float       IAS;

    IAS    = Model_U * sqrt(Weather_DensityRatio);
    e      = Vref - IAS - Model_UDot * SPD_Kudot;
    SPD_s += e;
    tp     = SPD_Kp * e + SPD_Ki * SPD_s;
    tp = Maths_Limit(tp, 0.2, 1.0);
    return tp;
}

/* --------------------------------------------------------- */
void FCS_EngageVSpeedHold()
{
    APVSMode = true;
    FCS_DisengageHeightHold();
    FCS_DisengageAutoland();
}

/* --------------------------------------------------------- */
void FCS_DisengageVSpeedHold()
{
    APVSMode = false;
}

/* --------------------------------------------------------- */
bool FCS_VSpeedHoldEngaged()
{
    return APVSMode;
}

/* --------------------------------------------------------- */
void FCS_SetVSpeedHold(float Vref)
{
    APVS = Vref;
}

/* --------------------------------------------------------- */
float FCS_VSpeedHold(float Vref)
{
    return FPAHold(Vref / Model_Vc);
}

/* --------------------------------------------------------- */
static float BankAngleHold(float RollC)
{
    const float Bank_Kp = 3.0;
    const float Bank_Kd = 3.0;

    float       da;

    RollC = Maths_Limit(RollC, -DEG25, DEG25);
    da = (RollC - Model_Roll) * Bank_Kp - Model_P * Bank_Kd;
    da = Maths_Limit(da, -1.0, 1.0);
    return da;
}

/* --------------------------------------------------------- */
static float FPAHold(float GammaC)
{
    const float FPA_Kp = 0.2;
    const float FPA_Ki = 20.0;

    float       Qc;
    float       dedot;
    float       Gamma;

    Gamma = Model_Pitch - Model_Alpha;
    Qc    = (GammaC - Gamma) * FPA_Kp;
    dedot = -FPA_Ki * (Qc - (Model_Q * cos(Model_Roll) - Model_R * sin(Model_Roll)));
    de_fcs = Maths_Integrate(de_fcs, dedot);
    de_fcs = Maths_Limit(de_fcs, -1.0, 1.0);
    return de_fcs;
}

/* --------------------------------------------------------- */
void BEGIN_FCS()
{
    APAlt                = 0.0;
    APAltMode            = false;
    APHdg                = 0.0;
    APHdgMode            = false;
    APSpeed              = 0.0;
    APSpeedMode          = false;
    APVS                 = 0.0;
    APVSMode             = false;
    AutolandMode         = false;
    APPitchMode          = false;
	
    OldSpeed             = 0;
    OldHdg               = 0;
    OldAlt               = 0;
    OldVS                = 0;
    OldMetric            = false;
	
    FCS_Kp               = 1.0;
    FCS_Ki               = 0.0;
    FCS_Kd               = 0.0;
	
    de_fcs               = 0.0;

    PitchRate_s          = 0.0;
    PitchRate_eold       = 0.0;
    RollRate_s           = 0.0;
    RollRate_eold        = 0.0;
    Beta_s               = 0.0;
    Beta_eold            = 0.0;
    Beta_lastValue       = 0.0;
    Nz_s                 = 0.0;
	
    YawRate_t            = 0.0;
    Roll_t               = 0.0;
    RollRate_t           = 0.0;
    YawDamper_t          = 0.0;
    YawRate_d            = 0.0;
    Roll_d               = 0.0;
    RollRate_d           = 0.0;
    YawDamper_d          = 0.0;
	
    FCS_ThrottlePosition = 0.0;
    FCS_APSpeedMode      = false;

    fdhx                 = 0.0;
    fdhy                 = 0.0;
	
    dr_a_lastValue       = 0.0;
    dr_a                 = 0.0;
    dr_c                 = 0.0;
    dr_t                 = 0.0;
    AoaProtection        = false;
    PitchProtection      = false;
    RollProtection       = false;
    PitchTrimRemembered  = 0.0;
    PitchRememberedValue = 0.0;
    FlareMode            = false;
    counterFlare         = 0;
}
