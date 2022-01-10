%close and clear everything on command window, figure ...
clear all
clc
close all

%generate symbol for degrees and links
syms ('L1', 'L2', 'L3', 'the1', 'the2', 'the3', 'real') 


% The angle range that each joint can work was determined in degrees and converted to radian. 
 the1_range =deg2rad([30:1:90]'); %180 degrees 1235MG servo motor 
 the2_range =deg2rad([-90:1:90]'); %180 degrees FT5335M servo motor
% just give information about range of servo motor 3 
% the3_range =deg2rad([-90:1:90]'); %270 degrees LDX 227 servo motor 


%link lengths in metric
 L1=0.14; %link1 lengths
 L2=0.14; %link2 lengths
%just give information
% L3=0.05; %link3 lengths

index=1; %iteration variable for P_endeff matrix

%creating P_endeff matrix to keep results from calculations that forward
%kinematics equation
%generating zero matrix, number of row is (range of the1) * (range of
%the2) and 9 columns 
%In column, it contains 3 elements (the1, the2 and the3) in radian and degrees,respectively, 3 positions coordinate (x,y and z position) 
P_endeff = zeros (size(the1_range,1)*size(the2_range,1),9); 



for iter1=1:1:size(the1_range,1)  %the number of loops in the range of the1 is defined 
    the1_c = cos(the1_range(iter1)); % cosine value of the1 at iter1 is assigned to the1_c
    the1_s = sin(the1_range(iter1)); % sine value of the1 at iter1 is assigned to the1_s
        for iter2=1:1:size(the2_range,1) %the number of loops in the range of the2 is defined
            the2_c = cos(the2_range(iter2)); % cosine value of the2 at iter2 is assigned to the2_c
            the2_s = sin(the2_range(iter2)); % sine value of the2 at iter2 is assigned to the2_s
                    
                   % L3 is not added to equation to take inverse kinematics of robot arm. 
                   %x position is calculated
                    P_endeff_x = L2*cos(the1_range(iter1) + the2_range(iter2)) + L1*cos(the1_range(iter1));
                   %y position is calculated
                    P_endeff_y = L2*sin(the1_range(iter1) + the2_range(iter2)) + L1*sin(the1_range(iter1));
               %z position is assign to zero because, we do not work z plane
                    P_endeff_z = 0;
                    %Calculated values is placed to P_endeff matrix the1(in
                    %radian), the2 (in radian), the3 (in radian), the1(in
                    %degree), the2(in degree), the3 (in degree), x, y and z
                    %position, respectively                    
                    P_endeff (index,1:9 ) = [the1_range(iter1) the2_range(iter2) -the1_range(iter1)-the2_range(iter2) ((the1_range(iter1))*180/pi) ((the2_range(iter2))*180/pi) ((-the1_range(iter1)-the2_range(iter2))*180/pi) P_endeff_x P_endeff_y P_endeff_z];
                  
                    %next index
                    index=index+1;
       end   
end

 %write P_endeff values on excel
xlswrite('workspace.xlsx',P_endeff);

%drawing x,y and z points in 3 dimension plane
scatter3(P_endeff(:,7),P_endeff(:,8),P_endeff(:,9))