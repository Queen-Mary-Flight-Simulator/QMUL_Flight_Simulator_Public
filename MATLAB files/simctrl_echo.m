%{
    Flight Simulator - MATLAB interface M-FILE script for data echo test.
	
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
    matlabMode = DIN(fslink_Active); % Flight simulator MATLAB mode state
    
    % Access simulator data
    de = DIN(fslink_Elevator);
    da = DIN(fslink_Aileron);
    dr = DIN(fslink_Rudder);
    dt = DIN(fslink_Throttle);
    
    % Only compute and send data to the flight simulator if mode is active
    if matlabMode == 1
    
		% Flag that MATLAB mode on IOS has been set. This IS duplication, but this varibale
		% remains 1 even if the IOS switches MATLAB mode off. Thus, provides a way to signal exit.
        matlabRunning = 1;
        
		
		
		
		%
		% CUSTOM FLIGHT CONTROL CODE STARTS HERE
		%


		
        % Finally, prepare the control data array for the MEX file - DOUT (Data OUT)
        % This should be the last line in the custom code
		DOUT = [da, de, dr, dt];

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
clear fslink;
