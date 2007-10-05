
function makeindex()
clear all;
a=dir;
numMfiles=1;

%publish options
publish_options.format='html';
publish_options.evalCode=false;
publish_options.outputDir='./html';

for i=1:size(a,1)
    [b,c]=strtok(a(i).name,'.');
    if strcmp(c(2:end),'m')
        mfilenames(numMfiles).val=b;
        mfilenames(numMfiles).calledby=[];
        mfilenames(numMfiles).calls=[];
        mfilenames(numMfiles).globals=[];
        
        [result,output]=system(strcat('grep -H "',char(mfilenames(numMfiles).val),'(" *.m'));
%         if result~=0
%             [result,output]=system(strcat('f:\bin\GnuWin32\bin\grep -H "',char(mfilenames(numMfiles).val),'(" *.m'));
%         end
        
        while (result==0 && size(output,2)>0)
            [templine,output]=strtok(output,char(10));
            calledbyfile =strtrim(strtok(templine,'.'));
            if isempty(strfind(mfilenames(numMfiles).calledby,calledbyfile )) &&...
                    ~strcmp(calledbyfile,mfilenames(numMfiles).val)
                mfilenames(numMfiles).calledby=[mfilenames(numMfiles).calledby,' ',calledbyfile];
            end
        end
        
        [result,output]=system(['grep -H -w "global" ',mfilenames(numMfiles).val,'.m']);
%         if result~=0
%             [result,output]=system(['f:\bin\GnuWin32\bin\grep -H -w "global" ',mfilenames(numMfiles).val,'.m']);
%         end

        while (result==0 && size(output,2)>0)
            [templine,output]=strtok(output,char(10));            
            [preceeding,globalvars]=strtok(templine,char(32));
            if ~isempty(preceeding) && ~strcmp(preceeding(end),'%') && strcmp(preceeding(end-5:end),'global')
                while size(globalvars,2)>0
                    [currentvar,globalvars]=strtok(globalvars,char(32));
                    if size(currentvar,2)>0 &&...
                            isempty(strfind(mfilenames(numMfiles).globals,currentvar)) &&...
                            ~strcmp(currentvar,'global')
                      
                        try if strcmp(currentvar(end),';') && size(currentvar,2)>1;  currentvar=currentvar(1:end-1); end
                        catch; keyboard; end
                        if isempty(strfind(mfilenames(numMfiles).globals,strcat(currentvar,';')))
                            mfilenames(numMfiles).globals=[mfilenames(numMfiles).globals,currentvar,'; '];
                        end
                    end
                end
            end
        end

        %Publish file to HTML directory
        publish([mfilenames(numMfiles).val,'.m'],publish_options);
        
        numMfiles=numMfiles+1;
    end
end

for i=1:size(mfilenames,2)
    for j=1:size(mfilenames,2)
        %since we dont need to search through function which call the
        %present to see if they are called by the present one
        if i~=j && ~isempty(strfind(mfilenames(j).calledby,mfilenames(i).val))
            mfilenames(i).calls=[mfilenames(i).calls,' ',mfilenames(j).val];
        end
    end
end

cd html
% Print Header
fid=fopen('index.html','w');
fprintf(fid,'%s\n\r%s\n\r%s\n\r%s\n\r%s\n\r','<html>','<head>','<H1>Exploration Matlab Code</H1>','<H2> &copy Gavin Paul 2007</h2>','</head>');
fprintf(fid,'%s\n\r','<table border="1" width="100%">');
fprintf(fid,'%s\n\r','<tr><th width="15%">File (function)</th><th width="30%">Calls</th><th width="30%">Called By</th><th width="25%">Globals Used</th></tr>');


%go through and print out each line out
for i=1:size(mfilenames,2)
    fprintf(fid,'%s','<tr>');
    %Check that there is a filename otherwise don't make it a link
    if exist(strcat(mfilenames(i).val,'.html'),'file')==2
        fprintf(fid,'%s\n\r',strcat('<td><a href="',mfilenames(i).val,'.html">',mfilenames(i).val,'</a></td>'));
    else
        fprintf(fid,'%s\n\r',strcat('<td>',mfilenames(i).val,' (No HTML file)</td>'));
    end

    fprintf(fid,'%s',strcat('<td>',mfilenames(i).calls,'</td>'));
    fprintf(fid,'%s',strcat('<td>',mfilenames(i).calledby,'</td>'));
    fprintf(fid,'%s',strcat('<td>',mfilenames(i).globals,'</td>'));

    fprintf(fid,'%s','</tr>');
end

fprintf(fid,'%s','</body></html>');
fclose(fid);

