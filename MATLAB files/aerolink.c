#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <SIM/clocks.h>
#include <SIM/iodefn.h>
#include <SIM/iosdefn.h>
#include <SIM/aerodefn.h>
#include <SIM/navdefn.h>
#include <SIM/octdefn.h>
#include <SIM/maths.h>
#include <SIM/cbool.h>
#include <SIM/weather.h>
#include <SIM/target.h>

#include "aerolink.h"
#include "fcs.h"
#include "engines.h"
#include "model.h"
#include "systems.h"
#include "aero.h"

#define DEFAULT_PORT 12345

static char BROADCAST_IP [] = "192.168.1.255";
static char GROUP_IP [] = "192.168.1.0";

AeroDefn_AeroDataPkt AeroLink_AeroPkt;
NavDefn_NavDataPkt AeroLink_NavPkt;
IODefn_IODataPkt AeroLink_IOPkt;
IosDefn_IosDataPkt AeroLink_IosPkt;
Protodefn_ProtoDataPkt AeroLink_ProtoPkt;
OctDefn_OctaveRxDataPkt AeroLink_OctRxPkt;
OctDefn_OctaveTxDataPkt AeroLink_OctTxPkt;
boolean AeroLink_OctaveMode;
unsigned int AeroLink_CmdPtr;
boolean AeroLink_ReplayMode;
unsigned int AeroLink_CameraPosition;
boolean AeroLink_Freezing;
boolean AeroLink_RemoteHold;

static int retval;
static struct sockaddr_in s_addr_in;
static struct sockaddr_in s_addr_out;
static int socket_in;
static int socket_out;
static int addr_len;
static int permission;

static boolean Pkt1Found = false;
static boolean Pkt3Found = false;
static char buff[1500];

static unsigned int group_ip;

static unsigned int FrameNumber;
static float WindSpeed;
static float WindDirection;
static IosDefn_RestoreVectorRecord LastRestore;
static boolean Restored;
static boolean OldOctaveMode;
static boolean OldReplayMode;

static unsigned int Get_Group(unsigned int x);
static unsigned int Get_Node(unsigned int x);
static unsigned char GetByte();
static float GetReal();
static unsigned int GetWord();
static int GetInt();
static boolean GetBoolean();
static void RememberState(IosDefn_RestoreVectorRecord v);
static void RestoreState(IosDefn_RestoreVectorRecord v);
static void InitialiseIOPkt(IODefn_IODataPkt *Pkt);
static void InitialiseNavPkt(NavDefn_NavDataPkt *Pkt);

/* ------------------------------------------------ */
static unsigned int Get_Group(unsigned int x)
{
  union {
      unsigned int c;
      char b[4];
  } x32;

  x32.c = htonl(x);
  x32.b[0] = x32.b[1];
  x32.b[1] = x32.b[2];
  x32.b[2] = x32.b[3];
  x32.b[3] = 0;
  return x32.c;
}

/* ------------------------------------------------ */
static unsigned int Get_Node(unsigned int x)
{
  return htonl(x) % 256;
}

/* ------------------------------------------------ */
void AeroLink_LinkTx()
{
  do {
    retval = recvfrom(socket_in, &buff, sizeof(buff), 0, 
                      (struct sockaddr *) &s_addr_in, &addr_len);
    if (retval < 0) {
      printf("recvfrom error\n");
      exit(1);
    }
    if (Get_Group(s_addr_in.sin_addr.s_addr) == group_ip) {
      switch (Get_Node(s_addr_in.sin_addr.s_addr)) {
      case 1:;
        if (retval != sizeof(AeroLink_IOPkt)) {
          printf("IOPkt: bad size: %d\n", retval);
          exit(1);
        }
        memcpy(&AeroLink_IOPkt, &buff, sizeof(AeroLink_IOPkt));
        Pkt1Found = true;
        break;
      case 2:;
        break;
      case 4:;
        if (retval != sizeof(AeroLink_IosPkt)) {
          printf("IosPkt: bad size: %d\n", retval);
          exit(1);
        }
        memcpy(&AeroLink_IosPkt, &buff, sizeof(AeroLink_IosPkt));
        AeroLink_RespondToIos();
        break;
      default :
        printf("Unknown pkt: %x8\n", htonl(s_addr_in.sin_addr.s_addr));
        break;
      }
    }
  } while (!Pkt1Found);

  Pkt1Found = false;

  AeroLink_FormPacket();
  retval = sendto(socket_out, &AeroLink_AeroPkt, sizeof(AeroLink_AeroPkt), 0, 
                  (struct sockaddr *) &s_addr_out, sizeof(s_addr_out));
  if (retval < 0) {
    printf("send error\n");
    exit(1);
  }
}

/* ------------------------------------------------ */
void AeroLink_LinkRx()
{
  do {
    retval = recvfrom(socket_in, &buff, sizeof(buff), 0, 
                      (struct sockaddr *) &s_addr_in, &addr_len);
    if (retval < 0) {
      printf("recvfrom error\n");
      exit(1);
    }
    if (Get_Group(s_addr_in.sin_addr.s_addr) == group_ip) {
      switch (Get_Node(s_addr_in.sin_addr.s_addr)) {
      case 2:;
        break;

      case 3:;
        if (retval != sizeof(AeroLink_NavPkt)) {
          printf("NavPkt: bad size: %d\n", retval);
          exit(1);
        }
        memcpy(&AeroLink_NavPkt, &buff, sizeof(AeroLink_NavPkt));
        Pkt3Found = true;
        break;
     
      case 4:;
        if (retval != sizeof(AeroLink_IosPkt)) {
          printf("IosPkt: bad size: %d\n", retval);
          exit(1);
        }
        memcpy(&AeroLink_IosPkt, &buff, sizeof(AeroLink_IosPkt));
        AeroLink_RespondToIos();
        break;

      default :
        printf("Unknown pkt: %x8\n", htonl(s_addr_in.sin_addr.s_addr));
        break;
      }
    }
  } while (!Pkt3Found);
  Pkt3Found = false;
}

/* ------------------------------------------------ */
void AeroLink_Init_Socket()
{
  permission = 1;
  addr_len = sizeof(s_addr_in);
  s_addr_in.sin_family = AF_INET;
  s_addr_in.sin_addr.s_addr = htonl(INADDR_ANY);
  s_addr_in.sin_port = htons(DEFAULT_PORT);
  socket_in = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (socket_in < 0) {
    printf("socket open error\n");
    exit(1);
  }
  if (bind(socket_in, (struct sockaddr *) &s_addr_in, sizeof(s_addr_in)) < 0) {
    printf("bind error\n");
    exit(1);
  }
  s_addr_out.sin_family = AF_INET;
  s_addr_out.sin_addr.s_addr = inet_addr(BROADCAST_IP);
  s_addr_out.sin_port = htons(DEFAULT_PORT);
  socket_out = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (socket_out < 0) {
    printf("socket open error\n");
    exit(1);
  }
  if (setsockopt(socket_out, SOL_SOCKET, SO_BROADCAST, &permission, sizeof(permission)) < 0) {
    printf("permission error\n");
    exit(1);
  }
}

/* ------------------------------------------------ */
void AeroLink_Close_Socket()
{
  close(socket_in);
  close(socket_out);
}

/* ------------------------------------------------ */
void AeroLink_RespondToIos()
{
  float t;
  boolean dummyb;
  float dummyr;
  int dummyi;

  AeroLink_CmdPtr = 0;
  for (;;) {
    switch (GetWord()) {

    case IosDefn_EndOfPkt:;
      /* ignore */
      break;

    case IosDefn_Exit:;
      /* ignore */
      break;

    case IosDefn_SetRunHoldFreeze:;
      switch (GetInt()) {
      case 0:;
        AeroLink_RemoteHold = false;
        AeroLink_Freezing = false;
        break;
      case 1:;
        AeroLink_RemoteHold = true;
        AeroLink_Freezing = false;
        break;
      case 2:;
        AeroLink_RemoteHold = false;
        AeroLink_Freezing = true;
        break;
      }
      break;
    case IosDefn_SetOctaveMode:;
      AeroLink_OctaveMode = GetBoolean();
      break;
    case IosDefn_LoadFlightPlan:;
      dummyb = GetByte();
      dummyb = GetByte();
      dummyb = GetByte();
      dummyb = GetByte();
      dummyr = GetReal();
      dummyr = GetReal();
      dummyi = GetInt();
      dummyi = GetInt();
      break;
    case IosDefn_SetFlightPlanMode:;
      dummyb = GetBoolean();
      break;
    case IosDefn_SetFlapsMode:;
      Systems_Failures[1] = GetBoolean();
      break;
    case IosDefn_SetGearMode:;
      Systems_Failures[2] = GetBoolean();
      break;
    case IosDefn_SetNav1LocaliserMode:;
      Systems_Failures[3] = GetBoolean();
      break;
    case IosDefn_SetNav1GlideSlopeMode:;
      Systems_Failures[4] = GetBoolean();
      break;
    case IosDefn_SetNav2LocaliserMode:;
      Systems_Failures[5] = GetBoolean();
      break;
    case IosDefn_SetNav2GlideSlopeMode:;
      Systems_Failures[6] = GetBoolean();
      break;
    case IosDefn_SetRMI1Mode:;
      Systems_Failures[7] = GetBoolean();
      break;
    case IosDefn_SetRMI2Mode:;
      Systems_Failures[8] = GetBoolean();
      break;
    case IosDefn_SetDMEMode:;
      Systems_Failures[9] = GetBoolean();
      break;
    case IosDefn_SetEngine1Mode:;
      Systems_Failures[10] = GetBoolean();
      break;
    case IosDefn_SetEngine2Mode:;
      Systems_Failures[11] = GetBoolean();
      break;
    case IosDefn_SetEngine3Mode:;
      Systems_Failures[12] = GetBoolean();
      break;
    case IosDefn_SetEngine4Mode:;
      Systems_Failures[13] = GetBoolean();
      break;
    case IosDefn_SetASIMode:;
      Systems_Failures[14] = GetBoolean();
      break;
    case IosDefn_SetAIMode:;
      Systems_Failures[15] = GetBoolean();
      break;
    case IosDefn_SetVSIMode:;
      Systems_Failures[16] = GetBoolean();
      break;
    case IosDefn_SetAltimeterMode:;
      Systems_Failures[17] = GetBoolean();
      break;
    case IosDefn_SetTurnMode:;
      Systems_Failures[18] = GetBoolean();
      break;
    case IosDefn_SetHSICardMode:;
      Systems_Failures[19] = GetBoolean();
      break;
    case IosDefn_SetRMICardMode:;
      Systems_Failures[20] = GetBoolean();
      break;
    case IosDefn_Restore:;
      RestoreState(AeroLink_IosPkt.RestoreVector);
      RememberState(AeroLink_IosPkt.RestoreVector);
      if (!Model_OnTheGround) {
        FCS_ResetFCS();
      }
      if (!Aero_AutoPilot) {
        FCS_DisengageSpeedHold();
        FCS_DisengageHeightHold();
        FCS_DisengageHeadingHold();
        FCS_DisengageVSpeedHold();
      }
      Weather_SetWind(WindSpeed, WindDirection, true);
      Engines_EngineModel();
      Model_FlightModel();
      break;
    case IosDefn_SetAircraftAltitude:;
      Model_Pz = GetReal();
      break;
    case IosDefn_SetAircraftHeading:;
      Model_Yaw = GetReal() + (float) AeroLink_NavPkt.MagneticVariation;
      Model_SetQuarternions();
      Model_SetDCM();
      break;
    case IosDefn_SetAircraftSpeed:;
      Model_U = GetReal() / sqrt(Weather_DensityRatio);
      Model_V = 0.0;
      Model_W = Model_U * tan(Model_Alpha);
      Model_ResetFlightModel();
      break;
    case IosDefn_SetCgPosition:;
      Aero_CgPosition = GetReal();
      break;
    case IosDefn_SetLeftFuelQuantity:;
      dummyr = GetReal();
      break;
    case IosDefn_SetRightFuelQuantity:;
      dummyr = GetReal();
      break;
    case IosDefn_SetSingleEngineMode:;
      dummyb = GetBoolean();
      break;
    case IosDefn_SetMorseMode:;
      dummyb = GetBoolean();
      break;
    case IosDefn_SetRMICardType:;
      dummyb = GetBoolean();
      break;
    case IosDefn_SetAdfDip:;
      dummyb = GetBoolean();
      break;
    case IosDefn_SetTurbulence:;
      Weather_Turbulence_Level = GetReal();
      break;
    case IosDefn_SetWindSpeed:;
      WindSpeed = GetReal();
      Weather_SetWind(WindSpeed, WindDirection, false);
      break;
    case IosDefn_SetWindDir:;
      WindDirection = GetReal();
      Weather_SetWind(WindSpeed, WindDirection, false);
      break;
    case IosDefn_SetQNH:;
      Weather_RegionalQNH = intround(GetReal());
      break;
    case IosDefn_SetMagneticVariation:;
      dummyr = GetReal();
      break;
    case IosDefn_SetTimeOfDay:;
      t = GetReal();
      Clocks_ClockHours = (unsigned int) t;
      Clocks_ClockMins = (unsigned int) ((t - (float) Clocks_ClockHours) * 60.0);
      break;
    case IosDefn_RePositionAircraft:;
      Model_Latitude = GetReal();
      Model_Longitude = GetReal();
      break;
    case IosDefn_SetGroundTemperature:;
      Weather_ISADeviation = GetReal();
      break;
    case IosDefn_SetTargetPosition:;
      Target_TLatitude = GetReal();
      Target_TLongitude = GetReal();
      break;
    case IosDefn_SetTargetDistance:;
      Target_TargPursuit(GetReal(), Model_Latitude, Model_Longitude, Model_Pz, Model_Yaw);
      break;
    case IosDefn_SetTargetSpeed:;
      Target_TU = GetReal();
      break;
    case IosDefn_SetTargetHeading:;
      Target_TYaw = GetReal() + (float) AeroLink_NavPkt.MagneticVariation;
      break;
    case IosDefn_SetTargetTurnRate:;
      Target_TYawDot = GetReal();
      break;
    case IosDefn_SetTargetAltitude:;
      Target_TPz = GetReal();
      break;
    case IosDefn_SetTargetClimbRate:;
      Target_TW = GetReal();
      break;
    case IosDefn_SetAutopilotAltitude:;
      dummyr = GetReal();
      break;
    case IosDefn_AutopilotAltitudeOn:;
      dummyb = GetBoolean();
      break;
    case IosDefn_SetAutopilotHeading:;
      dummyr = GetReal();
      break;
    case IosDefn_AutopilotHeadingOn:;
      dummyb = GetBoolean();
      break;
    case IosDefn_SetAutopilotSpeed:;
      dummyr = GetReal();
      break;
    case IosDefn_AutopilotSpeedOn:;
      dummyb = GetBoolean();
      break;
    case IosDefn_SetAutopilotVSpeed:;
      dummyr = GetReal();
      break;
    case IosDefn_AutopilotVSpeedOn:;
      dummyb = GetBoolean();
      break;
    case IosDefn_AutolandOn:;
      if (GetBoolean()) {
        FCS_EngageAutoland();
      } else {
        FCS_DisengageAutoland();
      }
      break;
    case IosDefn_SetKp:;
      FCS_Kp = GetReal();
      break;
    case IosDefn_SetKi:;
      FCS_Ki = GetReal();
      break;
    case IosDefn_SetKd:;
      FCS_Kd = GetReal();
      break;
    case IosDefn_PlaybackReplay:;
      AeroLink_ReplayMode = GetBoolean();
      if (AeroLink_ReplayMode != OldReplayMode) {
        if (AeroLink_ReplayMode) {
          OldOctaveMode = AeroLink_OctaveMode;
          AeroLink_OctaveMode = false;
        } else {
          AeroLink_OctaveMode = OldOctaveMode;
        }
        OldReplayMode = AeroLink_ReplayMode;
      }
      break;
    case IosDefn_PlaybackCamera:;
      AeroLink_CameraPosition = GetInt();
      break;
    default :
      return;
      break;
    }
  }
}

/* ------------------------------------------------ */
static unsigned char GetByte()
{
  unsigned char x;

  x = AeroLink_IosPkt.CmdBuff[AeroLink_CmdPtr];
  AeroLink_CmdPtr = AeroLink_CmdPtr + 1;
  return x;
}

/* ------------------------------------------------ */
static float GetReal()
{
union {
  float r;
  char b[4];
} x32;

  x32.b[0] = GetByte();
  x32.b[1] = GetByte();
  x32.b[2] = GetByte();
  x32.b[3] = GetByte();
  return x32.r;
}

/* ------------------------------------------------ */
static unsigned int GetWord()
{
  union {
    unsigned short int c;
    char b[2];
} x16;

  x16.b[0] = GetByte();
  x16.b[1] = GetByte();
  return (unsigned int) x16.c;
}

/* ------------------------------------------------ */
static int GetInt()
{
  union {
    short int i;
    char b[2];
} x16;

  x16.b[0] = GetByte();
  x16.b[1] = GetByte();
  return (int) x16.i;
}

/* ------------------------------------------------ */
static boolean GetBoolean()
{
  unsigned char x;

  x = GetByte();
  return (x != 0);
}

/* ------------------------------------------------ */
static void RememberState(IosDefn_RestoreVectorRecord v)
{
  memcpy(&LastRestore, &v, sizeof(IosDefn_RestoreVectorRecord));
  Restored = true;
}

/* ------------------------------------------------ */
void AeroLink_RestoreLast()
{
  if (Restored) {
    RestoreState(LastRestore);
    Weather_SetWind(WindSpeed, WindDirection, true);
  }
}

/* ------------------------------------------------ */
static void RestoreState(IosDefn_RestoreVectorRecord v)
{
  Model_Latitude = v.Latitude;
  Model_Longitude = v.Longitude;
  Model_Pz = v.Pz + Aero_CGHeight;
  Model_Pitch = v.Pitch;
  Model_Roll = v.Roll;
  Model_Yaw = v.Yaw;
  Model_U = v.U;
  Model_V = v.V;
  Model_W = v.W;
  Model_P = v.P;
  Model_Q = v.Q;
  Model_R = v.R;
  Model_Vc = sqrt(Model_U * Model_U + Model_V * Model_V + Model_W * Model_W);
  Model_SetQuarternions();
  Model_SetDCM();
  Model_ResetFlightModel();
}

/* ------------------------------------------------ */
void AeroLink_FormPacket()
{
  unsigned int i;

  AeroLink_AeroPkt.PktNumber = FrameNumber;
  FrameNumber = FrameNumber + 1;
  AeroLink_AeroPkt.Pitch = Model_Pitch;
  AeroLink_AeroPkt.Roll = Model_Roll;
  AeroLink_AeroPkt.Yaw = Model_Yaw;

  AeroLink_AeroPkt.P = Model_P;
  AeroLink_AeroPkt.Q = Model_Q;
  AeroLink_AeroPkt.R = Model_R;

  AeroLink_AeroPkt.PDot = Model_PDot;
  AeroLink_AeroPkt.QDot = Model_QDot;
  AeroLink_AeroPkt.RDot = Model_RDot;

  AeroLink_AeroPkt.Vn = Model_Vn;
  AeroLink_AeroPkt.Ve = Model_Ve;
  AeroLink_AeroPkt.Vd = Model_Vd;

  AeroLink_AeroPkt.Pz = Model_Pz - Aero_CGHeight;
  AeroLink_AeroPkt.Latitude = Model_Latitude;
  AeroLink_AeroPkt.Longitude = Model_Longitude;

  AeroLink_AeroPkt.U = Model_U;
  AeroLink_AeroPkt.V = Model_V;
  AeroLink_AeroPkt.W = Model_W;

  AeroLink_AeroPkt.UDot = Model_UDot;
  AeroLink_AeroPkt.VDot = Model_VDot;
  AeroLink_AeroPkt.WDot = Model_WDot;

  AeroLink_AeroPkt.Pmt = Model_Pmt;
  AeroLink_AeroPkt.Rmt = Model_Rmt;
  AeroLink_AeroPkt.Ymt = Model_Ymt;

  AeroLink_AeroPkt.Alpha = Model_Alpha;
  AeroLink_AeroPkt.Beta = Model_Beta;
  AeroLink_AeroPkt.AlphaDot = Model_AlphaDot;
  AeroLink_AeroPkt.BetaDot = Model_BetaDot;
  AeroLink_AeroPkt.Stalling = Model_Stalling;

  AeroLink_AeroPkt.Cl = Model_Cl;
  AeroLink_AeroPkt.Cd = Model_Cd;

  AeroLink_AeroPkt.Lift = Model_Lift;
  AeroLink_AeroPkt.Thrust = Model_Thrust;
  AeroLink_AeroPkt.Drag = Model_Drag;
  AeroLink_AeroPkt.SideForce = Model_SideForce;

  AeroLink_AeroPkt.XForce = Model_XForce;
  AeroLink_AeroPkt.YForce = Model_YForce;
  AeroLink_AeroPkt.ZForce = Model_ZForce;

  AeroLink_AeroPkt.Vc = Model_Vc;
  AeroLink_AeroPkt.MachNumber = Model_MachNumber;
  AeroLink_AeroPkt.WindSpeed = WindSpeed;
  AeroLink_AeroPkt.WindDir = WindDirection;

  AeroLink_AeroPkt.Ixx = Aero_Ixx;
  AeroLink_AeroPkt.Iyy = Aero_Iyy;
  AeroLink_AeroPkt.Izz = Aero_Izz;
  AeroLink_AeroPkt.Ixz = Aero_Ixz;
  AeroLink_AeroPkt.Iyz = Aero_Iyz;
  AeroLink_AeroPkt.Ixy = Aero_Ixy;
  AeroLink_AeroPkt.Mass = Aero_Mass;

  AeroLink_AeroPkt.Ex = Model_Ex;
  AeroLink_AeroPkt.Ey = Model_Ey;
  AeroLink_AeroPkt.Ez = Model_Ez;

  AeroLink_AeroPkt.Rho = Weather_Rho;
  AeroLink_AeroPkt.OAT = Weather_GroundTemperature;

  AeroLink_AeroPkt.EngineType = Aero_EngineType;
  AeroLink_AeroPkt.NumberOfEngines = (short unsigned int) Aero_NumberOfEngines;
  for (i = 0; i <= 3; i += 1) {
    AeroLink_AeroPkt.Engines[i].Thrust = Engines_Engines[i].Thrust;
    AeroLink_AeroPkt.Engines[i].Epr = Engines_Engines[i].Epr;
    AeroLink_AeroPkt.Engines[i].Rpm = Engines_Engines[i].Rpm;
    AeroLink_AeroPkt.Engines[i].FuelFlow = Engines_Engines[i].FuelFlow;
    AeroLink_AeroPkt.Engines[i].Egt = Engines_Engines[i].Egt;
    AeroLink_AeroPkt.Engines[i].Beta = Engines_Engines[i].Beta;
    AeroLink_AeroPkt.Engines[i].ManifoldPressure = Engines_Engines[i].ManifoldPressure;
    AeroLink_AeroPkt.EngineLevers[i] = Engines_ThrottleLever[i];
  }
  AeroLink_AeroPkt.FuelQuantityLeft = Engines_FuelQuantityLeft;
  AeroLink_AeroPkt.FuelQuantityRight = Engines_FuelQuantityRight;

  AeroLink_AeroPkt.EngineThrust = Engines_EngineThrustX;
  AeroLink_AeroPkt.EngineMoment = Engines_EngineYMT;

  AeroLink_AeroPkt.EngineFireSound = Engines_EngineFireSound;
  AeroLink_AeroPkt.ConfigWarning = Systems_ConfigWarning;

  AeroLink_AeroPkt.FlapPosition = Systems_FlapPosition;
  AeroLink_AeroPkt.GearPosition = Systems_GearPosition;

  AeroLink_AeroPkt.Elevator = Model_Elevator * Aero_ElevatorGain;
  AeroLink_AeroPkt.Aileron = Model_Aileron * Aero_AileronGain;
  AeroLink_AeroPkt.Rudder = Model_Rudder * Aero_RudderGain;

  AeroLink_AeroPkt.ElevatorTrim = Systems_GetElevatorTrim() * Aero_ElevatorTrimGain;
  AeroLink_AeroPkt.AileronTrim = Systems_GetAileronTrim() * Aero_AileronTrimGain;
  AeroLink_AeroPkt.RudderTrim = Systems_GetRudderTrim() * Aero_RudderTrimGain;

  AeroLink_AeroPkt.OctaveMode = AeroLink_OctaveMode;
  AeroLink_AeroPkt.OnTheGround = Model_OnTheGround;
  AeroLink_AeroPkt.TimeStamp = ((Clocks_ClockHours * 60 + Clocks_ClockMins) * 60 +
                                 Clocks_ClockSecs) * Clocks_FrameRate + Clocks_ClockTicks;

  AeroLink_AeroPkt.TPitch = Target_TPitch;
  AeroLink_AeroPkt.TRoll = Target_TRoll;
  AeroLink_AeroPkt.TYaw = Target_TYaw;
  AeroLink_AeroPkt.TPz = Model_Pz - Aero_CGHeight;
  AeroLink_AeroPkt.TLatitude = Target_TLatitude;
  AeroLink_AeroPkt.TLongitude = Target_TLongitude;

  AeroLink_AeroPkt.TimeOfDay = (unsigned short int) Clocks_ClockHours * 60 + Clocks_ClockMins;
}

/* ------------------------------------------------ */
void AeroLink_AddFlightData(unsigned int n, float x)
{
  AeroLink_AeroPkt.FlightData[n-1] = x;
}

/* ------------------------------------------------ */
void AeroLink_Replay()
{
  Model_Pitch = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Pitch;
  Model_Roll = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Roll;
  Model_Yaw = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Yaw;
  Model_P = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.P;
  Model_Q = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Q;
  Model_R = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.R;
  Model_PDot = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.PDot;
  Model_QDot = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.QDot;
  Model_RDot = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.RDot;
  Model_Vn = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Vn;
  Model_Ve = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Ve;
  Model_Vd = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Vd;
  Model_Pz = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Pz + Aero_CGHeight;
  Model_Latitude = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Latitude;
  Model_Longitude = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Longitude;
  Model_U = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.U;
  Model_V = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.V;
  Model_W = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.W;
  Model_UDot = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.UDot;
  Model_VDot = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.VDot;
  Model_WDot = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.WDot;
  Model_Pmt = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Pmt;
  Model_Rmt = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Rmt;
  Model_Ymt = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Ymt;
  Model_Alpha = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Alpha;
  Model_Beta = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Beta;
  Model_AlphaDot = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.AlphaDot;
  Model_BetaDot = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.BetaDot;
  Model_Stalling = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Stalling;
  Model_Cl = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Cl;
  Model_Cd = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Cd;
  Model_Lift = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Lift;
  Model_Thrust = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Thrust;
  Model_Drag = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Drag;
  Model_SideForce = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.SideForce;
  Model_XForce = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.XForce;
  Model_YForce = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.YForce;
  Model_ZForce = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.ZForce;
  Model_Vc = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Vc;
  Model_MachNumber = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.MachNumber;
  WindSpeed = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.WindSpeed;
  WindDirection = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.WindDir;
  Weather_Rho = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Rho;
  Weather_GroundTemperature = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.OAT;
  if (AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.EngineType == AeroDefn_Jet && 
      AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.NumberOfEngines == 4) {
    Engines_EngineThrustX = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.EngineThrust;
    Engines_EngineYMT = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.EngineMoment;
    Engines_EngineFireSound = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.EngineFireSound;
  }
  Engines_FuelQuantityLeft = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.FuelQuantityLeft;
  Engines_FuelQuantityRight = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.FuelQuantityRight;
  Systems_ConfigWarning = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.ConfigWarning;
  Systems_FlapPosition = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.FlapPosition;
  Systems_GearPosition = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.GearPosition;
  Model_OnTheGround = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.OnTheGround;
}

/* ------------------------------------------------ */
static void InitialiseIOPkt(IODefn_IODataPkt *Pkt)
{
  {
    Pkt->PktNumber = 0;
    Pkt->Elevator = 0.0;
    Pkt->Aileron = 0.0;
    Pkt->Rudder = 0.0;
    Model_ElevatorTrim = 0.0;
    Model_AileronTrim = 0.0;
    Model_RudderTrim = 0.0;
    Pkt->FlapSelector = 0.0;
    Pkt->EngineLever = 0.0;
    Pkt->ReverseLever = 0.0;
    Pkt->LeftBrake = 0.0;
    Pkt->RightBrake = 0.0;
    Pkt->LampsTestButton = IODefn_Off;
    Pkt->WarningCancelButton = IODefn_Off;
    Pkt->GearSelector = IODefn_GearOff;
    Pkt->RudderTrimSwitch = IODefn_TrimOff;
    Pkt->ThrottlePushButton = IODefn_Off;
    Pkt->ThrottleSwitch = IODefn_Off;
    Pkt->TriggerSwitch = IODefn_Off;
    Pkt->AileronTrimSwitch = IODefn_TrimOff;
    Pkt->ElevatorTrimSwitch = IODefn_TrimOff;
    Pkt->HoldButtonPressed = false;
    Pkt->RestoreButtonPressed = false;
    Pkt->FreezeButtonPressed = false;
    Pkt->ClockButtonPressed = false;
    Pkt->AutomaticResetSwitch = IODefn_Off;
    Pkt->RightBoostPumpSwitch = IODefn_Off;
    Pkt->LeftBoostPumpSwitch = IODefn_Off;
    Pkt->StarterSwitch = IODefn_Off;
    Pkt->IgnitionSwitch = IODefn_Off;
    Pkt->MasterSwitch = IODefn_Off;
    Pkt->ColumnButton = IODefn_Off;
    Pkt->ParkBrake = IODefn_Off;
    Pkt->KeySwitch = IODefn_On;
    Pkt->TimeStamp = 0;
  }
}

/* ------------------------------------------------ */
static void InitialiseNavPkt(NavDefn_NavDataPkt *Pkt)
{
  unsigned int i;
  unsigned int j;

  {
    Pkt->PktNumber = 0;
    Pkt->BaroPressure1 = 1013;
    Pkt->BaroPressure2 = 1013;
    Pkt->MagneticVariation = 0.0;
    Pkt->CurrentRunway = 0;
    Pkt->RMI_Dir1 = 0.0;
    Pkt->RMI_Dir2 = 0.0;
    Pkt->RunwayLatitude = 0.0;
    Pkt->RunwayLongitude = 0.0;
    Pkt->RunwayQDM = 0;
    Pkt->GroundLevel = 0.0;
    Pkt->OuterMarker = false;
    Pkt->MiddleMarker = false;
    Pkt->InnerMarker = false;
    Pkt->MarkerTest = false;
    Pkt->Track = 0.0;
    Pkt->TrackRange = 0;
    {
      Pkt->ILS1.LocaliserError = 0.0;
      Pkt->ILS1.GlideSlopeError = 0.0;
      Pkt->ILS1.SlantDistance = 0.0;
      Pkt->ILS1.GroundSpeed = 0.0;
      Pkt->ILS1.GroundDistance = 0.0;
      Pkt->ILS1.BearingToStation = 0.0;
      Pkt->ILS1.ILSBeacon = false;
      Pkt->ILS1.RunwayQdm = 0.0;
      Pkt->ILS1.BeaconStatus = false;
    }
    {
      Pkt->NAV1.LocaliserError = 0.0;
      Pkt->NAV1.GlideSlopeError = 0.0;
      Pkt->NAV1.SlantDistance = 0.0;
      Pkt->NAV1.GroundSpeed = 0.0;
      Pkt->NAV1.GroundDistance = 0.0;
      Pkt->NAV1.BearingToStation = 0.0;
      Pkt->NAV1.ILSBeacon = false;
      Pkt->NAV1.RunwayQdm = 0.0;
      Pkt->NAV1.BeaconStatus = false;
    }
    {
      Pkt->NAV2.LocaliserError = 0.0;
      Pkt->NAV2.GlideSlopeError = 0.0;
      Pkt->NAV2.SlantDistance = 0.0;
      Pkt->NAV2.GroundSpeed = 0.0;
      Pkt->NAV2.GroundDistance = 0.0;
      Pkt->NAV2.BearingToStation = 0.0;
      Pkt->NAV2.ILSBeacon = false;
      Pkt->NAV2.RunwayQdm = 0.0;
      Pkt->NAV2.BeaconStatus = false;
    }
    {
      Pkt->ADF1.LocaliserError = 0.0;
      Pkt->ADF1.GlideSlopeError = 0.0;
      Pkt->ADF1.SlantDistance = 0.0;
      Pkt->ADF1.GroundSpeed = 0.0;
      Pkt->ADF1.GroundDistance = 0.0;
      Pkt->ADF1.BearingToStation = 0.0;
      Pkt->ADF1.ILSBeacon = false;
      Pkt->ADF1.RunwayQdm = 0.0;
      Pkt->ADF1.BeaconStatus = false;
    }
    {
      Pkt->ADF2.LocaliserError = 0.0;
      Pkt->ADF2.GlideSlopeError = 0.0;
      Pkt->ADF2.SlantDistance = 0.0;
      Pkt->ADF2.GroundSpeed = 0.0;
      Pkt->ADF2.GroundDistance = 0.0;
      Pkt->ADF2.BearingToStation = 0.0;
      Pkt->ADF2.ILSBeacon = false;
      Pkt->ADF2.RunwayQdm = 0.0;
      Pkt->ADF2.BeaconStatus = false;
    }
    Pkt->HSI_Crs = 0;
    Pkt->HSI_Hdg = 0;
    Pkt->VOR_Obs = 0;

    for (i = 0; i <= 1; i += 1) {
      Pkt->SavedRadios[i].ComVHF1.Active = 0;
      Pkt->SavedRadios[i].ComVHF1.Stby = 0;
      Pkt->SavedRadios[i].ComVHF2.Active = 0;
      Pkt->SavedRadios[i].ComVHF2.Stby = 0;
      Pkt->SavedRadios[i].ComVHF3.Active = 0;
      Pkt->SavedRadios[i].ComVHF3.Stby = 0;
      Pkt->SavedRadios[i].ComHF1.Active = 0;
      Pkt->SavedRadios[i].ComHF1.Stby = 0;
      Pkt->SavedRadios[i].ComHF2.Active = 0;
      Pkt->SavedRadios[i].ComHF2.Stby = 0;
      Pkt->SavedRadios[i].ComAM.Active = 0;
      Pkt->SavedRadios[i].ComAM.Stby = 0;
      Pkt->SavedRadios[i].NavADF.Active = 0;
      Pkt->SavedRadios[i].NavADF.Stby = 0;
      Pkt->SavedRadios[i].NavVOR.Active = 0;
      Pkt->SavedRadios[i].NavVOR.Stby = 0;
      Pkt->SavedRadios[i].NavILS.Active = 0;
      Pkt->SavedRadios[i].NavILS.Stby = 0;
      Pkt->SavedRadios[i].CrsKnob = 0;
      Pkt->SavedRadios[i].NavGuard = false;
      Pkt->SavedRadios[i].PowerSwitch = false;
      for (j = 0; j <= 13; j += 1) {
        Pkt->SavedRadios[i].PushSwitches[j] = false;
      }
    }
    Pkt->FD = false;
    Pkt->LS = false;
    Pkt->LOC = false;
    Pkt->AP1 = false;
    Pkt->AP2 = false;
    Pkt->ATHR = false;
    Pkt->EXPED = false;
    Pkt->APPR = false;
    Pkt->SPD_MACH = false;
    Pkt->HDG_TRK = false;
    Pkt->FCUHdg = 0;
    Pkt->FCUAlt = 0;
    Pkt->FCUSpeed = 0;
    Pkt->FCUVS = 0;
    Pkt->FCUHdgHold = false;
    Pkt->FCUAltHold = false;
    Pkt->FCUSpeedHold = false;
    Pkt->FCUVSHold = false;
    Pkt->FCUBaroPressure = 1013;
    Pkt->FCUBaroHg = false;
    Pkt->FCUBaroMode = 0;
    Pkt->NavAid1 = NavDefn_NavOFF;
    Pkt->NavAid2 = NavDefn_NavOFF;
    Pkt->MapMode = NavDefn_MapILS;
    Pkt->DispMode = NavDefn_DispOFF;
    Pkt->MorseMode = false;
  }
}

/* ------------------------------------------------ */
void BEGIN_AeroLink()
{
    AeroLink_CmdPtr = 0;
    FrameNumber = 0;
    Pkt1Found = false;
    Pkt3Found = false;
    group_ip = Get_Group(inet_addr(GROUP_IP));
    WindSpeed = 0.0;
    WindDirection = 0.0;
    AeroLink_Freezing = false;
    AeroLink_RemoteHold = false;
    Restored = false;
    AeroLink_ReplayMode = false;
    AeroLink_CameraPosition = 1;
    AeroLink_OctaveMode = false;
    OldOctaveMode = AeroLink_OctaveMode;
    OldReplayMode = false;
    InitialiseIOPkt(&AeroLink_IOPkt);
    InitialiseNavPkt(&AeroLink_NavPkt);
}
