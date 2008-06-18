function [acsor_data] =  ACSOR_init()

%-----------------------------Initalise-------------------------------
%     clear all;
clear acsor_data
%     close all;
    close_all_coms();

%     global acsor_data
    acsor_data.acsor_data_hex = [];
    acsor_data.acsor_data_dec = [];
    acsor_data.acsor_data_all_readings = {};

    %create new port object if needed change com port if needed
    if ~isfield(acsor_data,'acsorport_obj')
        acsor_data.acsorport_obj=serial('COM1',...
            'BaudRate',115200,...
            'DataBits',8,...
            'Parity','none',...
            'FlowControl' ,'none',...
            'StopBits',1,...
            'Timeout',2,...
            'InputBufferSize', 4096,...
            'OutputBufferSize', 4096);
    end

    %open a new object if needed
    if strcmp(get(acsor_data.acsorport_obj,'status'),'closed')
        try fopen(acsor_data.acsorport_obj);
        catch error('Couldnt open com1 for the acsor - plug it in')
        end
    end


%this funciton closes all come ports
function close_all_coms()

inst_array=instrfind;
for i=1:length(inst_array)
    if ~strcmp(inst_array(i).Status,'closed')
        warning(strcat('Com',num2str(i),' is not closed, closing it now'))
        fclose(inst_array(i))
    end
    delete(inst_array(i))
    disp(strcat(sprintf('\n'),'All COM ports have been closed'));
end
