%% Pose Selection To Classify Unknown - Implementation Ver
% Description: This function is used to identiy areas which have unknown
% classification and attempt to look at them in such a way that we can do a
% classification

function poseclassunknown_Imp_newest(numofintplanes)

%% Setup and Variables
% close all
global workspace classPlanePlotHa AXBAMnCtestdata alldirectedpoints Q

% figure(1)
% plot_planes(plane,mew);
% axis([-1,1,-1,1,0,1.5]);
% axis equal
% camlight

% if ~exist('alldirectedpoints','var')    
display('Not currently clearning alldirectedpoints');
%   alldirectedpoints=[];
% end


%% Load data or classify a scan

NOhandleOPTIONS.useRealRobot=true;
NOhandleOPTIONS.show_robot=true;
NOhandleOPTIONS.animate_move=false;
NOhandleOPTIONS.remv_unkn_in_mv=false;    

%must start from a reasonable pose
 try movetonewQ(0,rad2deg([Q(1:3),0,0,0]),[],NOhandleOPTIONS); 
 catch %#ok<CTCH>
   error('The start must have joint 4-6 =0')
 end

 
%this is how many we go after at once
maxindexsize=30;

%take one laser sweep from an initial pose
% newQ=[0,-88*pi/180,140*pi/180,0,0*pi/180,0];
%move to the next place
% movetonewQ(0,rad2deg(newQ),[],NOhandleOPTIONS);
% Q=newQ;

% % and collect data to classify
% if NOhandleOPTIONS.useRealRobot
%     try use_real_robot_SCAN(90); catch; display('Couldnt scan');end
% end
%             
           

%% More Vars
% the classifier reterns a classification of this number is unknown
% unknownclass=8;
% size of the surfaces which is 1/2 of 20' scan at ideal distance 0.5*tan(20*pi/180)
mew=workspace.classifyMew;
% This is how many sets of data we will classify
% numofintplanes=20;

%how many solutions and failures
solsfound=0;
noposefound=0;



%optimisataion variables
global classunkn_optimise 
classunkn_optimise.minSurfToEF=0.25;
classunkn_optimise.maxSurfToEF=0.8;
classunkn_optimise.distAwayfromTarget=0.47;
classunkn_optimise.maxAngle=42*pi/180;
% classunkn_optimise.iLimit=500;
% classunkn_optimise.stol=1e-6;

%% determine regions of uncertainty
[index,UNclassifiedvoxels]=get_unknown_identification();    
       

%% set the blasting variables
try [planeSet,pose,pathval]=determinePathsNposes(index,maxindexsize);
  if size(planeSet,2)==0; error('do it again');end
catch
  lasterr;
  display('No poses found, trying again with 4* maxindexsize, and removing self scanning')
  workspace.indexedobsticles=remove_self_scanning(workspace.indexedobsticles,Q);

  try [planeSet,pose,pathval]=determinePathsNposes(index,4*maxindexsize);
    if size(planeSet,2)==0; error('Still no pose');end
  catch
    lasterr;
    display('No poses found, trying again with all index')

    try [planeSet,pose,pathval]=determinePathsNposes(index,size(index,1));
      if size(planeSet,2)==0; error('No really...Still no pose');end
    catch
        lasterr
      display('Still no luck so handing over to you');
      keyboard
    end
  end
end
  

%% go through until we have found enough solutions as required

indextoblast=1;

while solsfound<numofintplanes
    try
    %Shouldn't happen but if we dont have a valid pose go to next pose
    if isempty(pose)
      display('for some reason pose variable is empty')
      keyboard
    elseif ~pose(indextoblast).validPose            
        indextoblast=indextoblast+1;
        continue
    else %we have a valid pose
        
        % plot plane and points
        for j=1:length(classPlanePlotHa);  try delete(classPlanePlotHa(j));end; end
        classPlanePlotHa=plot_planes(planeSet(indextoblast),mew);        
        hold on;
        classPlanePlotHa(2)=plot3(planeSet(indextoblast).home_point(1),planeSet(indextoblast).home_point(2),planeSet(indextoblast).home_point(3),'b*');
        classPlanePlotHa(3)=plot3([planeSet(indextoblast).home_point(1),planeSet(indextoblast).home_point(1)+planeSet(indextoblast).normal_by_eigenval(1)/10],...
                                  [planeSet(indextoblast).home_point(2),planeSet(indextoblast).home_point(2)+planeSet(indextoblast).normal_by_eigenval(2)/10],...
                                  [planeSet(indextoblast).home_point(3),planeSet(indextoblast).home_point(3)+planeSet(indextoblast).normal_by_eigenval(3)/10],'b');    

                     
        %make sure vector is correct
         newQ=pose(indextoblast).Q;

        %move to the next place if possible, else continue with the
        %next plane        
        if ~movetonewQ(0,rad2deg(newQ),pathval(indextoblast).all_steps,NOhandleOPTIONS);            
            %go to next one since we can't get to this one
            indextoblast=indextoblast+1;
            continue;
        end
            % and do a scan to classify
%% If we are doing with real robot then move the 5th joint
        if NOhandleOPTIONS.useRealRobot
            %try and move through a complete sensing sweep so we have to make newQ standard
            if newQ(5)>-45*pi/180 
                if newQ(5)<60*pi/180
                    newQ=[newQ(1:4),newQ(5)+30*pi/180,0];
                else
                    newQ=[newQ(1:4),90*pi/180,0];
                end
            else
                newQ=[newQ(1:4),-45*pi/180,0];
            end

            try 
              movetonewQ(0,rad2deg(newQ),[],NOhandleOPTIONS);
            catch  %#ok<CTCH>
              error('Could move to correct position');
            end
            
%% for testing/plotting purposes
            AXBAMnCtestdata.planeSet=planeSet(indextoblast);
            AXBAMnCtestdata.newQ=newQ;
            AXBAMnCtestdata.Pworkspace=workspace;
            AXBAMnCtestdata.allE=pose(indextoblast).allE;
%% end fortesting         
            
            try use_real_robot_SCAN(-60); organise_data();catch; error('Could sense anything');end
            use_real_robot_GETJs();
            % and sense and classify           
            global G_scan;
            try [ClassifiedData] = Block_Classifier(G_scan.PointData, G_scan.IntensityData); 
            catch  %#ok<CTCH>
                error('Couldnt classify');
            end                                
            
            %update oc grid status
            update_ocstatus(ClassifiedData);
            
%             move back to zeros for last 4
            try movetonewQ(0,rad2deg([Q(1:3),0,0,0]),[],NOhandleOPTIONS);  %try moving to all zeros
            catch %#ok<CTCH>
                movetonewQ(0,[0,-85,90,0,0,0],[],NOhandleOPTIONS); %try moving back to the assumed safe pose starting guess
            end 
        end
        
        
        solsfound=solsfound+1;
        display(['Solution found and found a path successfully there = ',num2str(solsfound)]);
        %if we have enough then break
        if solsfound==numofintplanes; 
            break;
        end
            
%% Testing
%          uiwait(msgbox('press OK button to continue'));
        try AXBAMnCtesting(false);
        catch %#ok<CTCH>
            display('Some error when saving testing data');
          keyboard
        end
%% End testing 

        %so we dont redo the same point again or within 2*Mew of any other center point
        pt=planeSet(indextoblast).home_point;            
        alldirectedpoints=[alldirectedpoints;pt];
        
        [index,UNclassifiedvoxels]=get_unknown_identification();    
                      
        %% Determine poses and paths
        try [planeSet,pose,pathval]=determinePathsNposes(index,maxindexsize,alldirectedpoints);
          if size(planeSet,1)==0; error('do it again');end
        catch %#ok<CTCH>
          display('No poses found, trying again with 4* maxindexsize, and removing self scanning')
          workspace.indexedobsticles=remove_self_scanning(workspace.indexedobsticles);
          
        try [planeSet,pose,pathval]=determinePathsNposes(index,4*maxindexsize);
          if size(planeSet,1)==0; error('Still no pose');end
        catch %#ok<CTCH>
          display('No poses found, trying again with all index')
          try [planeSet,pose,pathval]=determinePathsNposes(index,size(index,1));
          catch %#ok<CTCH>
            display('Still no luck so handing over to you');
            keyboard
          end
        end
        end

        
        %reset indextoblast
        indextoblast=1;
        
    end
    
    catch %#ok<CTCH>
        lasterr
        display('some problem in the while loop, you have control')
        keyboard
    end
end


  

    
       
        
