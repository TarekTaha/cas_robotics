%% update_jointconfig_obs
%
% *Description*: This file goes through a discrete set of all possible
% configurations for the first 3 joints and determines if there is a
% collision here or if it is unknownthen stores this in a big matrix  

%% Function Call
%
% *Inputs:* NULL
%
% *Outputs:* NULL
%

function []=update_jointconfig_obs(matsize,obsticle_points,unknown_points)

%% Variables
global graf_obs r;

%graf_obs code
% col1 = x, col2=y, col3=z,col4=reason (-1 impossible, 0 obstacle, 1 unknown)

qlimits=r.qlim;
n=r.n;
L=r.link;

for piece=1:n
    linkvals(piece).val=[L{piece}.alpha L{piece}.A L{piece}.D L{piece}.offset];
end

%% Functions
%if we dont have graf_obs object then try and load one or on last resort build one
if isempty(graf_obs)
    %try and load this from file if possible make sure file is correct
    if exist('graf_obs.mat','file')==2
        try load graf_obs.mat
            display('Loading graf_obs.mat, make sure if this is not new that you delete it and make a new one');
        catch
            error('update_jointconfig_obs::Error loading file');
        end
    else
        graf_obs=zeros([matsize(1)*matsize(2)*matsize(3),4]);
        for i=1:matsize(1)
            for j=1:matsize(2)
                for k=1:matsize(3)
                    [J1,J2,J3]=mapindextojoints(i,j,k,qlimits,matsize);
                    if ~joint_softlimit_check([J1,J2,J3,0,0,0])
                        %means it is an impossible pose due to the soft limits
                        graf_obs(matsize(3)*matsize(2)*(i-1)+matsize(3)*(j-1)+k,:)=[i,j,k,-1];
                    else
                        [results,unknown_points_result]=check_path_for_col([J1,J2,J3,0,0,0],obsticle_points,unknown_points,linkvals);
                        if ~results %means that it is impossible (due to obstacle) and will always be so
                            graf_obs(matsize(3)*matsize(2)*(i-1)+matsize(3)*(j-1)+k,:)=[i,j,k,0];
                        elseif ~unknown_points_result %means that it is unknown space and may become possible
                            graf_obs(matsize(3)*matsize(2)*(i-1)+matsize(3)*(j-1)+k,:)=[i,j,k,1];
                        end
                    end
                end
            end
        end
        %make size correct again
        graf_obs=graf_obs((graf_obs(:,1)>0 & graf_obs(:,2)>0 & graf_obs(:,3)>0),:);
        save('graf_obs.mat','graf_obs');
    end
else %else remove points that are now free from the old one 
    graf_obs_2Bupdated=graf_obs(:,4)==1;
    for i=1:matsize(1)
        for j=1:matsize(2)
            for k=1:matsize(3)
                %if there is a point which was previously unknown but may be able to be updated
                if ~isempty(find(graf_obs(graf_obs_2Bupdated,1)==i & graf_obs(graf_obs_2Bupdated,2)==j & graf_obs(graf_obs_2Bupdated,3)==k,1))
                    [J1,J2,J3]=mapindextojoints(i,j,k,qlimits,matsize);
                    [results,unknown_points_result]=check_path_for_col([J1,J2,J3,0,0,0],obsticle_points,unknown_points,linkvals);
                    if results && unknown_points_result
                        nowfreeindx=find(graf_obs(:,1)==i & graf_obs(:,2)==j & graf_obs(:,3)==k,1);
                        graf_obs=graf_obs([1:nowfreeindx-1,nowfreeindx+1:end],:);
                        graf_obs_2Bupdated=graf_obs_2Bupdated([1:nowfreeindx-1,nowfreeindx+1:end],:);
                    elseif ~results 
                        %it is in collision due to an obstacle and we
                        %shouldn't check it again
                        graf_obs(find(graf_obs(:,1)==i & graf_obs(:,2)==j & graf_obs(:,3)==k,1),4)=0;                        
                    end
                end
            end
        end
    end
end