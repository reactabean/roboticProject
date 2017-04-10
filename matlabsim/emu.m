function [ pos, vel] = emu( tau,posstart,velstart)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

 M1 =1.7;
 M2 =1.0;
 M3 =1.7;
 M4 = 1.0;


%calculate M ========================
 M = Mfun( posstart,velstart )
 V = Vfun
 G(1:4) = 0;
 G(3) = -gravity*(M3 + M4);
 F = 0.5*velstart ;
for index = 1:10
   
    temp1 =  tau - V - G - F;
    
    
    
    
    

    
    
    
    
    
    
    
    
end





end

