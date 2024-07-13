%{
	simctrl-constants.m
	
	Script containing definitions for constants used in the flight simulator / MATLAB interface.

	Copyright (C) 2015 Aerosoft - All Rights Reserved
%}


% Port used for UDP packets
fslink_defaultport = 54321;

% MEX file function control. MEX files typically provide a single function, so parameterising
% the MEX function provides a multifunction MEX file.
fslink_open = 1;
fslink_close = 2;
fslink_send = 3;
fslink_recv = 4;
fslink_dataset = 5;
fslink_dataget = 6;

% Extracted flight simulator data and arranged for MEX file access via a MATLAB array.
fslink_Roll = 1;
fslink_Pitch = 2;
fslink_Yaw = 3;
fslink_P = 4;
fslink_Q = 5;
fslink_R = 6;
fslink_Altitude = 7;
fslink_Alpha = 8;
fslink_Beta = 9;
fslink_U = 10;
fslink_V = 11;
fslink_W = 12;
fslink_Udot = 13;
fslink_Vdot = 14;
fslink_Wdot = 15;
fslink_Vn = 16;
fslink_Ve = 17;
fslink_Vd = 18;
fslink_Elevator = 19;
fslink_Aileron = 20;
fslink_Rudder = 21;
fslink_ElevatorTrim = 22;
fslink_AileronTrim = 23;
fslink_RudderTrim = 24;
fslink_Throttle = 25;
fslink_Active = 26;
fslink_KeySwitch = 27;
