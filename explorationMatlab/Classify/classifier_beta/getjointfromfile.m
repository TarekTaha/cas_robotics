%% getjointfromfile
% *Description*: Given a ply filename from Steve this can get the joint
% angle from the top of file. This function is made redundant by the scan
% position being included in the ply file

function Q=getjointfromfile(fileName)
    %read just the 4th line where the pose of the robot is
    fid=fopen(fileName);for i=1:4; lineoftext = fgetl(fid);end;fclose(fid)
    [nothing,str]=strtok(lineoftext, '(');
    str=str(2:end-1);
    for i=1:6
        [temp,str]=strtok(str(2:end), ',');
        Q(i)=str2double(temp);
    end
    Q=deg2rad(Q);
end