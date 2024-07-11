%{
    Flight Simulator - MATLAB interface M-FILE script for altitude hold test.
	
	Copyright (C) 2015 Aerosoft - All Rights Reserved
%}

% Define the constants used in the mex file interface
simctrl_constants;

% Main loop control
done = 0;

% Gets set to 1 when the IOS activates MATLAB mode 
matlabMode = 0; 

% Set to 1 when matlabMode 1 detected. Used to detect IOS has deactivated MATLAB mode
matlabRunning = 0;

%
% CUSTOM FLIGHT CONTROL GLOBAL VARIABLES START HERE
%

Href   = -2500.0 * 0.3048;   % 2500 ft -> m
de     = 0.0;
demin  = -0.4;
demax  = 0.4;

%
% CUSTOM FLIGHT CONTROL GLOBAL VARIABLES END HERE
%


% Open the UDP port for the flight simulator connection
fslink(fslink_open,fslink_defaultport);

%
% Main Loop - It is advised to not execute a Ctrl-C at the MATLAB prompt.
% The script will exit in an orderly way via the IOS. Only in the event that
% the flight simulator has unexpectedly stopped, then Ctrl-C can be used.
%
while done == 0
    
    % Wait for the current flight simulation iteration packets to arrive.
    fslink(fslink_recv);
    
    % Extract the received data from the MEX file into a Matlab array - DIN (Data IN)
    DIN = fslink(fslink_dataget);
    matlabMode = DIN(fslink_Active); % Active
    
    % Only compute and send data to the flight simulator if mode is active
    if matlabMode == 1
    
		% Flag that MATLAB mode on IOS has been set. This IS duplication, but this varibale
		% remains 1 even if the IOS switches MATLAB mode off. Thus, provides a way to signal exit.
        matlabRunning = 1;
        
		
		
		
		%
		% CUSTOM FLIGHT CONTROL CODE STARTS HERE
		%
		
		% Access and compute altitude hold using simulation variables.
		% See script simctrl-constants.m for a list of available variables.
		
		U     = DIN(fslink_U);  % U
		Udot  = DIN(fslink_Udot);  % Udot
		H     = DIN(fslink_Altitude);  % Altitude
		pitch = DIN(fslink_Pitch);  % Pitch
		alpha = DIN(fslink_Alpha);  % Alpha
		q     = DIN(fslink_Q);  % Q
		Vd    = DIN(fslink_Vd);  % Vd

        if ( U < 0.1 )
            U = 0.1;
        end

        VSref = -0.08333*(Href - H);
		if VSref > 5.08
            VSref = 5.08;
        elseif VSref < -5.08
            VSref = -5.08;
        end

        % Vertical speed controller
        FPAngC = VSref / U;

        % Flight path angle controller
        pitchC = FPAngC + alpha;

        % Pitch angle controller (converts pitch angle error into pitch rate demand)
        qC = 0.2*(pitchC - pitch); % Pitch rate commanded

        % Pitch rate controller (converts pitch rate error into elevator command)
        dedot = -2.0*(qC - q); % Elevator change from pitch rate error
        de = de + 0.02*dedot; % Forward Euler integration

        % Apply the elevator input saturation limits
        % Limit: demin <= de <= demax
		if de > demax
            de = demax;
        elseif de < demin
            de = demin;
        end
		
        % Finally, prepare the control data array for the MEX file - DOUT (Data OUT)
		% This should be the last line in the custom code
        DOUT = [0.0, de, 0.0, DIN(fslink_Throttle)];

        %
		% CUSTOM FLIGHT CONTROL CODE ENDS HERE
		%




        % Pass the control data to the mex file
        fslink(fslink_dataset,DOUT);

        % Send the control data to the flight simulator
        fslink(fslink_send);
    
	else
	
        % When not in matlab mode, check if matlab mode was running previously.
        % This flags that the user has exited matlab mode, so break from the main loop
        if matlabRunning == 1
          done = 1; 
        end
    
	end
    
end % main loop


% Close UDP connection
fslink(fslink_close);

% Clean up the memory used by the MEX file
clear fslink
