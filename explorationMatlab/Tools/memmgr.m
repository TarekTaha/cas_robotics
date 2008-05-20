%This enables you to see what is in memory and clear it
display('1) These are the vars in the workspace')
whos
varsinmem=whos;

display('2) These are the GLOBALS in the workspace')
whos('global')

globvarsinmem=whos('global');


a=input('which var to delete? {-1=end} Type 1 (norm) or 2 (glob)'); 

while a>0
    b=input('now type var numb from top or (0) to delete all, (-1) to quit\n');
    
    if b==-1; return; end
    
    if a>=1 && a<2
        if b>0
            todelete=varsinmem(round(b));
            reply = input(['Are you sure you want to delete ',todelete.name,'? y/n [n]: '], 's');
            if strcmp(reply,'y') || strcmp(reply,'Y')
                thewords=['clear ', eval('todelete.name')];
                eval(thewords)           
            end
        else
            reply = input('Are you sure you want to delete ALL? y/n [n]: ', 's');
            if strcmp(reply,'y') || strcmp(reply,'Y')
                thewords='clear all';
                eval(thewords)           
            end            
        end        
    elseif  a>=2 && a<3
        if b>0
            todelete=globvarsinmem(round(b));
            reply = input(['Are you sure you want to delete ',todelete.name,'? y/n [n]: '], 's');
            if strcmp(reply,'y') || strcmp(reply,'Y')
                thewords=['clear global ', eval('todelete.name')];
                eval(thewords)
            end
        else
            reply = input('Are you sure you want to delete ALL globals? y/n [n]: ', 's');
            if strcmp(reply,'y') || strcmp(reply,'Y')
                thewords='clear all global';
                eval(thewords)           
            end            
        end
           
    end
    a=input('which var to delete? {-1=end} Type 1 (norm) or 2 (glob)'); 
end

display('1) These are the vars in the workspace')
whos
display('2) These are the GLOBALS in the workspace')
whos('global')

