function explorationMain(handles)

set(handles.stopflag_checkbox,'Value',0)

set(handles.stopflag_checkbox,'Visible','on')

show_new_info_details=false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global robot_maxreach workspace G_scan Q

%start data collection
state_data=collectExploreData([]);
useNBV=false;
set(gcf,'CurrentAxes',handles.axes3)

%% intial safe pose default scans
%since we wish to choose our own default start points
for stepcount=2:size(robot_maxreach.default_Q,1)+1
    want_to_continue=~get(handles.stopflag_checkbox,'Value');
    while want_to_continue && G_scan.tries<size(robot_maxreach.default_Q,1)
        try explore(handles,useNBV); 
            want_to_continue=~get(handles.stopflag_checkbox,'Value');
            if want_to_continue==0; error('User chose to exit');end
            break;             
        catch
            display(lasterr);
            want_to_continue=input('Type (1) to continue, (2) for keyboard command, (0) to exit\n');            
            if want_to_continue==2; keyboard; end
            if want_to_continue==0; error('User chose to exit');end
            if get(handles.useRealRobot_checkbox,'Value')==1;  
                set(handles.stopflag_checkbox,'Value',0);
                set(handles.stopflag_checkbox,'Visible','on');
            end   
        end; 
    end
    state_data=collectExploreData(state_data,handles,stepcount);   
end

want_to_continue=~get(handles.stopflag_checkbox,'Value');

%% NBV based exploration 
if want_to_continue
    %now go through and get NBV and then use them to explore
    for stepcount=stepcount+1:15;
        if ~get(handles.stopflag_checkbox,'Value')
            %do next best view exploration search
          try NBV_beta2();
          catch
            lasterr
            display('No views found or some error, removing self scanning and trying again');
            workspace.indexedobsticles=remove_self_scanning(workspace.indexedobsticles);
            try NBV_beta2();catch; lasterr;display('Still no best views fond or some error: handing over control to you master');keyboard;end
          end
          
            current_bestview=stepcount-1;
            global bestviews;        

            useNBV=true;
            
            want_to_continue=~get(handles.stopflag_checkbox,'Value');   
                
            while want_to_continue; 
                try %if we have already planned a path, use this one otherwise try and get another, otherwise go to next possible one
                    if movetonewQ(handles,rad2deg(bestviews(1).Q),bestviews(1).all_steps);
                        G_scan.done_bestviews_orfailed=[G_scan.done_bestviews_orfailed;bestviews(1).Q];explore(handles,useNBV,1);validpathfound=true;break;
                    else %can't get to the desired best view
                        display('User has control');
                        keyboard

                        %tac on the actual position here just in case it isn't exactly where it was supposed to finish
                        robot_maxreach.path(end).all_steps(end+1,:)=Q;
                        %move back along the path taken to get here
                        if ~movetonewQ(handles,rad2deg(robot_maxreach.path(end).all_steps(1,:)),robot_maxreach.path(end).all_steps(end:-1:1,:));
                            display('some major problem if we cant follow the same path back');
                            keyboard                                
                        end
                        %try once again to move to the actual desired newQ for exploration
                        if movetonewQ(handles,rad2deg(bestviews(1).Q),bestviews(1).all_steps);
                            G_scan.done_bestviews_orfailed=[G_scan.done_bestviews_orfailed;bestviews(1).Q];explore(handles,useNBV,1);validpathfound=true;break;
                        else % last resort is to remove surrounding obstacle points remove indexed and normal obsticle points within robot FF since not valid
                            display(['No Valid path available or found, on #',num2str(current_bestview)]);
                            validpathfound=false;
                        end
                    end

                    want_to_continue=0;                        
                catch; display(lasterr);
                    want_to_continue=input(' Type (0) to go to new best view, (1) to continue trying for a path, (2) for keyboard command, (3) to exit\n');            
                    if want_to_continue==2; keyboard; end
                    if want_to_continue==3; error('User chose to exit');end
                    if get(handles.useRealRobot_checkbox,'Value')==1;  use_real_robot_GETJs();end 
                    display(['No Valid path available or found, on #',num2str(current_bestview)]);
                    validpathfound=false;
                end;
            end                

            if show_new_info_details            
            %Plotting and dispalying what we expected compared to what we got
            display(strcat('The size of the expected infor was:',num2str(size(bestviews(1).expectedaddinfo)),', While the actual size was:',num2str(size(workspace.newestknownledge)),...
                ', The set difference was:',num2str(size(setdiff(bestviews(1).expectedaddinfo,workspace.newestknownledge,'rows'))),...
                ', The weighted addinfo is:',num2str(bestviews(1).addinfo),'. And the overall weight was:',num2str(bestviews(1).overall)));
                temp=plot3(bestviews(1).expectedaddinfo(:,1),bestviews(1).expectedaddinfo(:,2),bestviews(1).expectedaddinfo(:,3),'r.');
                temp2=plot3(workspace.newestknownledge(:,1),workspace.newestknownledge(:,2),workspace.newestknownledge(:,3),'g.');
                temp3=setdiff(bestviews(1).expectedaddinfo,workspace.newestknownledge,'rows');
                temp4=plot3(temp3(:,1),temp3(:,2),temp3(:,3),'b.');
                pause(2);
                delete(temp); delete(temp4);
                pause(2);delete(temp2); 
            end

            %termination conditions
            changeinweight=diff(state_data.knownweight);
            if length(changeinweight)>3
                if sum(changeinweight(end-3:end))<100
                    %set to stop ASAP
                    set(handles.stopflag_checkbox,'Value',1);
                    display('Termination condition reached');
                    break;
                end
            end                   

          %save the state and save testing values
          state_data=collectExploreData(state_data,handles,stepcount); 

        end
    end
  
end


%Plotting the results
figure;
subplot(3,1,1);plot(state_data.knownweight);grid on; title('Known Weight');
subplot(3,1,2);plot(state_data.size_known);grid on; title('Total Known Points');
subplot(3,1,3);plot(state_data.time);grid on; title('Time Taken');

% Saving for journal results
% save('state_data.mat','state_data');
% testnum=input('test num');
% saveresultstofile(testnum);

% profile off; profile viewer;