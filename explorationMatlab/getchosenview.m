%% getchosenview
%
% *Description:*  This function goes through and finds out the chosen value
% from the radio buttons and sets it in scan.chosenview. Pass in the
% handles from the exGUI. This may be a redundant function since only
% needed if the user must select a pose otherwise it isn't required for
% autonmous exploration

%% Function Call
%
% *Inputs:* 
%
% _h_ (array double) the handles to the GUI
%
% *Returns:* NULL

function getchosenview(h)

%% Variables
global scan
thetrueone=0;
currentvalue=false;

%% Find which value has been selected
while currentvalue==false
    thetrueone=thetrueone+1;
    currentvalue=get(eval(strcat('h.v',num2str(thetrueone),'_radiobutton')),'value');  
end

%% Draw up the correct vectors based on selected orientation
switch thetrueone
    case 1 
        vector=unit([1,1,1]);
    case 2
        vector=unit([-1,1,1]);
    case 3 
        vector=unit([1,-1,1]);
    case 4 
        vector=unit([-1,-1,1]);
    case 5 
        vector=unit([1,1,-1]);
    case 6 
        vector=unit([-1,1,-1]);
    case 7 
        vector=unit([1,-1,-1]);
    case 8 
        vector=unit([-1,-1,-1]);
end

scan.chosenview=vector;