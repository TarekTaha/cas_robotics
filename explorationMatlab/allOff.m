%% allOff
%
% *Description:* This function turns off all the platform including the air
% suply, the warning lights and the warning noise

%% Function Call
%
% *Inputs:* NULL
%
% *Returns:* NULL

function allOff()

%% Turn off everything
hPlatform = actxserver('EyeInHand.PlatformState');
pause(0.1);
hPlatform.AirSupply = 0;
hPlatform.Brake = 0;
hPlatform.WarningLight = 0;
hPlatform.WarningNoise = 0;
pause(0.1);
hPlatform.release