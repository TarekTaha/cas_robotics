%% Function makeindex
%
% *Description*: Developed so to enable uses to quickly publish directories
% of code and details of the cross function and global variable dependancies. 
% 
%% Details
%  
% * Using the freely avalable grep tool this function parses m files in the
% current directory:
% * It will determine all the global variables used by the various files so
% it is easy to check for overlaps of possible conflicts. 
% * It will also show which functions (scripts) are called by others and
% which which function in this directory are called by your code. 
% * It then makes a html folder and documents subsiquently published code in an index.html file along with links to the published files.
%
%% Requirements/Usage
%  
% * Linux users: have grep. Windows users: download gnu grep for win32 setups, 
% <http://gnuwin32.sourceforge.net/packages/grep.htm>
% make sure setup directory is added to the windows PATH environment variable. 
% * Place "makeindex.m" with other matlab tools or in directory added to
% the matlab path so it can be accessed in matlab from anyway
% * Navigate to your directory
% * type "makeindex"
% * You wll now have a html directory with your docuemented code in it and
% an index.html file which shows brief details of each
%
% *Note:* For best results and a valid desciption setup your comment in
% your files like this one so you will have a valid description on the
% index.html page. Also name functions the same as the m file they are in

function makeindex()

%% User Selectable Publish Options
username='username';% Please Change Me

publish_options.format='html';
publish_options.evalCode=false;
publish_options.outputDir='./html';


current_dir=dir;
% m file index start number
currMfile=1;

todaysdate=date;
directoryName=pwd;
if ispc
    slashPos=findstr(pwd,'\');
else
    slashPos=findstr(pwd,'/');
end
directoryName=directoryName(slashPos(end)+1:end);

tic;try
%% Go through each m file, extract info and publish
for i=1:size(current_dir,1)
    [b,c]=strtok(current_dir(i).name,'.');
    if strcmp(c(2:end),'m')
        mfilenames(currMfile).val=b;
        mfilenames(currMfile).description=[];
        mfilenames(currMfile).calledby=[];
        mfilenames(currMfile).calls=[];
        mfilenames(currMfile).globals=[];
        
        [result,output]=system(strcat('grep -H "',char(mfilenames(currMfile).val),'(" *.m'));
        if result==1 && ~isempty(output)
            error('Check grep is properly installed and env var is added to PATH! Go to a shell and type "grep" and make sure');
        end

%% Read description from comment headers of file (You should setup header like this file)
        tempfn=fopen(char([mfilenames(currMfile).val,'.m']));
        linebyline=textscan(tempfn,'%s','delimiter', '\n');
        current_line=3;
        while ~strcmp(char(linebyline{1}{current_line}),'')    
            commentmarkpos=findstr(char(linebyline{1}{current_line}),'%');
            [remaining,nothing]=strtok(char(linebyline{1}{current_line}),'%');
            if isempty(mfilenames(currMfile).description) && (strcmp(remaining,'') ||...
                    strcmp(remaining,' ')) ||... 
                    isempty(mfilenames(currMfile).description) && isempty(commentmarkpos) ||...
                    current_line==size(linebyline{1},1)
                
                mfilenames(currMfile).description='<b> None Valid </b>';
                break;
            else
                if isempty(mfilenames(currMfile).description)
                    mfilenames(currMfile).description=remaining;
                else
                    mfilenames(currMfile).description=[mfilenames(currMfile).description,'<br>',remaining];
                end
            end      
            current_line=current_line+1;
        end

        fclose(tempfn);

%% Determine functions which calls this function
        while (result==0 && size(output,2)>0)
            [templine,output]=strtok(output,char(10));
            calledbyfile =strtrim(strtok(templine,'.'));
            if isempty(strfind(mfilenames(currMfile).calledby,calledbyfile )) &&...
                    ~strcmp(calledbyfile,mfilenames(currMfile).val)
                mfilenames(currMfile).calledby=[mfilenames(currMfile).calledby,' ',calledbyfile];
            end
        end
        
        [result,output]=system(['grep -H -w "global" ',mfilenames(currMfile).val,'.m']);

%% Determine globals used in this function
        while (result==0 && size(output,2)>0)
            [templine,output]=strtok(output,char(10));            
            [preceeding,globalvars]=strtok(templine,char(32));
            if ~isempty(preceeding) && ~strcmp(preceeding(end),'%') && strcmp(preceeding(end-5:end),'global')
                while size(globalvars,2)>0
                    [currentvar,globalvars]=strtok(globalvars,char(32));
                    if size(currentvar,2)>0 &&...
                            isempty(strfind(mfilenames(currMfile).globals,currentvar)) &&...
                            ~strcmp(currentvar,'global')
                      
                        try if strcmp(currentvar(end),';') && size(currentvar,2)>1;  currentvar=currentvar(1:end-1); end; end;
                        if isempty(strfind(mfilenames(currMfile).globals,strcat(currentvar,';')))
                            mfilenames(currMfile).globals=[mfilenames(currMfile).globals,currentvar,'; '];
                        end
                    end
                end
            end
        end

%% Publish file to HTML directory
        publish([mfilenames(currMfile).val,'.m'],publish_options);
        
        currMfile=currMfile+1;
    end
end
catch; keyboard;end
%check if we have found anything to publish
if ~exist('mfilenames','var')
    our_pwd=pwd;
    error(['No m files found, please check current directory: ',our_pwd]);
end
%% Gather published filenames and generate links from the index page

for i=1:size(mfilenames,2)
    for j=1:size(mfilenames,2)
        %since we dont need to search through function which call the
        %present to see if they are called by the present one
        if i~=j && ~isempty(strfind(mfilenames(j).calledby,mfilenames(i).val))
            mfilenames(i).calls=[mfilenames(i).calls,' ',mfilenames(j).val];
        end
    end
end

%% Make index.html file
cd html
% Print Header
fid=fopen('index.html','w');
fprintf(fid,'%s\n\r%s\n\r%s\n\r%s\n\r%s\n\r','<html>','<head>',['<H1>Matlab Code: ',directoryName,'</H1>'],['<H2> &copy ',username,' ',todaysdate,'</h2>'],'</head>');
fprintf(fid,'%s\n\r','<table border="1" width="100%">');
fprintf(fid,'%s\n\r','<tr><th width="15%">File (function)</th><th width="26%">Description</th><th width="24%">Calls</th><th width="24%">Called By</th><th width="11%">Globals Used</th></tr>');


%% Go through and print out each line out
for i=1:size(mfilenames,2)
    fprintf(fid,'%s','<tr>');
    %Check that there is a filename otherwise don't make it a link
    if exist(strcat(mfilenames(i).val,'.html'),'file')==2
        fprintf(fid,'%s\n\r',strcat('<td><a href="',mfilenames(i).val,'.html">',mfilenames(i).val,'</a></td>'));
    else
        fprintf(fid,'%s\n\r',strcat('<td>',mfilenames(i).val,' (No HTML file)</td>'));
    end

    fprintf(fid,'%s',strcat('<td>',mfilenames(i).description,'&nbsp</td>'));
    fprintf(fid,'%s',strcat('<td>',mfilenames(i).calls,'&nbsp</td>'));    
    fprintf(fid,'%s',strcat('<td>',mfilenames(i).calledby,'&nbsp</td>'));
    fprintf(fid,'%s',strcat('<td>',mfilenames(i).globals,'&nbsp</td>'));

    fprintf(fid,'%s','</tr>');
end

fprintf(fid,'%s','</table> <br> Created by makeindex, &copy;  Gavin Paul 2008</body></html>');
fclose(fid);

toc
display('Publishing complete');
% open up the main index page 
open index.html;