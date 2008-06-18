function [acsor_data] =  GetScanFromACSOR(acsor_data)

number_of_readings_per_freq = 100;
% 'B' - Burst

% Init
temp_acsor_data_dec=[];

%-------------Gets Reading--------------------------
acsor_data.command='B';% this is what is to be sent
fprintf(acsor_data.acsorport_obj,acsor_data.command);%issue command to scan 
pause(.3);

%------------------ Get Data ------SLOW---------
%acsor_data.acsor_data_dec = [acsor_data.acsor_data_dec, fread(acsor_data.acsorport_obj,get(acsor_data.acsorport_obj,'BytesAvailable'),'uint16')];
%temp_acsor_data_dec = acsor_data.acsor_data_dec;
%-----------------------------------------------

%------------------ Get Data -------------------
acsor_data.acsor_data_hex = fread(acsor_data.acsorport_obj,get(acsor_data.acsorport_obj,'BytesAvailable'));
% for i = 1:2:length(acsor_data.acsor_data_hex)-1
%     temp_acsor_data_dec = [temp_acsor_data_dec, hex2dec([dec2hex(acsor_data.acsor_data_hex(i+1,1)) dec2hex(acsor_data.acsor_data_hex(i,1))])];
% end
%-----------------------------------------------
temp_acsor_data_dec=[hex2dec([dec2hex(acsor_data.acsor_data_hex(2:2:end,1)),dec2hex(acsor_data.acsor_data_hex(1:2:end,1))])]';

for i = 1:length(temp_acsor_data_dec)/(number_of_readings_per_freq)
    temp_acsor_data_dec2(i,1:number_of_readings_per_freq) = temp_acsor_data_dec(1,number_of_readings_per_freq*(i-1)+1:number_of_readings_per_freq*(i-1)+number_of_readings_per_freq);
end

acsor_data.acsor_data_dec = temp_acsor_data_dec2;
acsor_data.acsor_data_all_readings{end+1,1} = acsor_data.acsor_data_dec;


