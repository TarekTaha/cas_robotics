%% template_make_m_file
%
% *Description*: This file is setout how files must be to compile with the
% documentation engine. The descrition at top should be succinct and will
% be shown on the main index html page. Note the space (no %) after the
% description. This function can be called to make m files in the correct
% format

%% Function Call
%
% *Inputs:* 
%
% _file_name_ (size, type string) what is its purpose?
%
% *Returns:* NULL

function template_make_m_file(file_name)

%% Variables

% global someglobal;
allinputvars=[];
alloutputvars=[];

%% Input Checks
if nargin<1
    error('Please pass in the name of the new file');
end

%% Display Input

display(['Making filename for function: ',file_name]);

%% Gather quick description of file
filedescription=input('Please describe the file in 30 words or less... ','s');

%% Describe inputs
current_val=1;
inputvars{current_val}=[];
while ~strcmp(inputvars{end},'-1')
    inputvars{current_val}=input('Enter INPUT vars or -1 if none/ finish then colon(:) then quick descrip, eg x:number of apples... ','s');
    current_val=current_val+1;
end

%% Describe Outputs
current_val=1;
outputvars{current_val}=[];
while ~strcmp(outputvars{end},'-1')
    outputvars{current_val}=input('Enter OUTPUT vars or -1 if none/ finish then colon(:) then quick descrip, eg x:number of apples... ','s');
    current_val=current_val+1;
end

%% Create new m file with this template

outputdata{1}{1}={['%% ', file_name]};
outputdata{1}{2}={'%'};
outputdata{1}{3}={['% *Description*: ',filedescription]};
outputdata{1}{4}={''};
outputdata{1}{5}={'%% Function Call'};
outputdata{1}{6}={'%'};
%print inputs
current_val=1;
if ~strcmp(inputvars{current_val},'-1')
    while ~strcmp(inputvars{current_val},'-1')
        [name,descript]=strtok(inputvars{current_val},':');
        if current_val==1
            outputdata{1}{7}={['% *Inputs:* ',name,' (size?, type?) ', descript]};
        else
            outputdata{1}{end+1}={'%'};
            outputdata{1}{end+1}={['% _',name,'_ (size?, type?) ', descript]};
        end
        allinputvars=[char(allinputvars),name];
        current_val=current_val+1;
    end
else
    outputdata{1}{7}={'% *Inputs:* NULL'};
end

outputdata{1}{end+1}={'%'};

%print outputs
current_val=1;
if ~strcmp(outputvars{current_val},'-1')
    while ~strcmp(outputvars{current_val},'-1')
        [name,descript]=strtok(outputvars{current_val},':');
        if current_val==1
            outputdata{1}{end+1}={['% *Outputs:* ',name,' (size?, type?) ', descript]};
        else
            outputdata{1}{end+1}={'%'};
            outputdata{1}{end+1}={['% _',name,'_ (size?, type?) ', descript]};
        end
        alloutputvars=[char(alloutputvars),name];
        current_val=current_val+1;
    end
else
    outputdata{1}{end+1}={'% *Outputs:* NULL'};
end  
 
outputdata{1}{end+1}={'%'};
outputdata{1}{end+1}={''};
outputdata{1}{end+1}={['function [',char(alloutputvars),']=',file_name,'(',char(allinputvars), ')']};
outputdata{1}{end+1}={''};
outputdata{1}{end+1}={'%% Variables'};
outputdata{1}{end+1}={''};
outputdata{1}{end+1}={'%% Input Checks'};
outputdata{1}{end+1}={['if nargin<',num2str(size(inputvars,1))]};
outputdata{1}{end+1}={'  error(''need to pass in correct num of vars'')'};  
outputdata{1}{end+1}={'end'};
outputdata{1}{end+1}={''};
outputdata{1}{end+1}={'%% Functions'};

%finally print output to file 
fid = fopen([file_name,'.m'], 'wt');
for i=1:size(outputdata{1},2)
    try fprintf(fid, '%s\n', char(outputdata{1}{i}));
    catch
        fclose(fid);
        error('Printnig to file');
    end
end
fclose(fid);