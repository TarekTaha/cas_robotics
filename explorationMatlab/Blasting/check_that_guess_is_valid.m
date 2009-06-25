%% check_that_guess_is_valid
function q=check_that_guess_is_valid(q,qlimits,pt,t,Links,numlinks,plane_equ,displayon,maxdistguess,linkvals)
%check to see we are not starting parallel with plane normal a temp variable to hold the joint fix
    newQ=q;
    
    [valid,dist_val,targetdist(1).val,correctway]=classunkcheck_newQ(newQ,qlimits,pt,t,Links,numlinks,plane_equ,displayon,linkvals); 
    if dist_val>maxdistguess        
        if newQ(5)>0; newQ(5)=q(5)-pi/2;            
        else newQ(5)=q(5)+pi/2;          
        end   
        if displayon; display('changing joint 5 only');end
        [valid,dist_val,targetdist(1).val,correctway]=classunkcheck_newQ(newQ,qlimits,pt,t,Links,numlinks,plane_equ,displayon,linkvals); 
        
        if dist_val>maxdistguess            
            newQ=q;
            if newQ(4)>0; newQ(4)=q(4)-pi/2;                              
            else newQ(4)=q(4)+pi/2;               
            end
            if displayon; display('changing joint 4 only');end
            [valid,dist_val,targetdist(1).val,correctway]=classunkcheck_newQ(newQ,qlimits,pt,t,Links,numlinks,plane_equ,displayon,linkvals); 
            
            if dist_val>maxdistguess
                newQ=q;
                if newQ(4)>0; newQ(4)=newQ(4)-pi/2;
                else newQ(4)=newQ(4)+pi/2;
                end                
                if newQ(5)>0; newQ(5)=newQ(5)-pi/2;
                else newQ(5)=newQ(5)+pi/2;
                end
                if displayon; display('changing joint 4 and 5');end
                [valid,dist_val,targetdist(1).val,correctway]=classunkcheck_newQ(newQ,qlimits,pt,t,Links,numlinks,plane_equ,displayon,linkvals); 
                
                if dist_val>maxdistguess
                    if displayon; display('Still not good enough even after different starts of 4 and 5');end
                    %probably could make a better guess
                end
            else if displayon; display('close enough without moving 4 and 5 together');end
            end
        else if displayon; display('close enough without moving 4');end
        end
    else if displayon; display('close enough without moving 4 or 5');end
    end
    
    %if we are aligned ok but we are around the opposite way then fix this
    if ~correctway         
        if newQ(4)>0; newQ(4)=newQ(4)-pi;                              
        else newQ(4)=newQ(4)+pi;               
        end
        if displayon; display('Switching directing by changing joint 4 only'); end
    end 
    
    q=newQ;
end