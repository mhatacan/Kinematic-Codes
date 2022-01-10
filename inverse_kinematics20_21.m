%close and clear everything on command window, figure ...
clear all
clc
close all
                
%generate symbol for degrees and links 
syms ('L1', 'L2', 'L3' ,'the1', 'the2', 'the3','real') 

the1_c = cos(the1); % cosine value of the1 is assigned to the1_c
the1_s = sin(the1); % sine value of the1 is assigned to the1_s

the2_c = cos(the2); % cosine value of the1 is assigned to the2_c
the2_s = sin(the2); % sine value of the1 is assigned to the2_s

the3_c = cos(the3); % cosine value of the1 is assigned to the3_c
the3_s = sin(the3);% sine value of the1 is assigned to the3_s

%The values in the DH parameters matrix are placed in order for T_01
T_01 = [the1_c -the1_s 0 0;
        the1_s the1_c 0 0;
        0 0 1 0;
        0 0 0 1];

%The values in the DH parameters matrix are placed in order for T_12
T_12 = [the2_c -the2_s 0 L1;
        the2_s the2_c 0 0;
        0 0 1 0;
        0 0 0 1];

%The values in the DH parameters matrix are placed in order for T_23
T_23 = [the3_c -the3_s 0 L2;
        the3_s the3_c 0 0;
        0 0 1 0;
        0 0 0 1];
    
    
%The values in the DH parameters matrix are placed in order for T_34 (end-effector parameters)
T_34 = [1 0 0 L3;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

%Transformation matrix
%Connecting parts from base to end effector with each other 
T_04=T_01*T_12*T_23*T_34;

%Simplifying T_04
P_endeff = simplify(T_04(1:3,end));

%Expected Result
%P_endeff = [L2*cos(the1 + the2) + L1*cos(the1) + L3*cos(the1 + the2 + the3)
%            L2*sin(the1 + the2) + L1*sin(the1) + L3*sin(the1 + the2 + the3)
%            0];


% The angle range that each joint can work was determined in degrees and converted to radian. 
the1_range =deg2rad([30:1:90]');   %180 degrees 1235MG servo motor
the2_range =deg2rad([-90:2:90]');  %180 degrees FT5335M servo motor
% just give information about range of servo motor 3 
% the3_range =deg2rad([-90:2:90]');  %270 degrees LDX 227 servo motor


%link lengths in metric
 L1=0.14; %link1 lengths
 L2=0.14; %link2 lengths
%just give information
% L3=0.05; %link3 lengths

index=1; %iteration variable for Endeff_Kin_inverse matrix

%creating Endeff_Kin_inverse matrix to keep results from calculations that forward
%kinematics equation
%generating zero matrix, number of row is (range of the1) * (range of
%the2) and 6 columns 
%In column, it contains 3 elements (the1, the2 and the3) in radian 3 positions coordinate (x,y and z position)
Endeff_Kin_inverse = zeros (size(the1_range,1)*size(the2_range,1),6);

for iter1=1:1:size(the1_range,1) %the number of loops in the range of the1 is defined 
    the1_c = cos(the1_range(iter1)); % cosine value of the1 at iter1 is assigned to the1_c
    the1_s = sin(the1_range(iter1)); % sine value of the1 at iter1 is assigned to the1_s
        for iter2=1:1:size(the2_range,1) %the number of loops in the range of the2 is defined
            the2_c = cos(the2_range(iter2)); % cosine value of the2 at iter2 is assigned to the2_c
            the2_s = sin(the2_range(iter2)); % sine value of the2 at iter2 is assigned to the2_s
            
                    % L3 is not added to equation to take inverse kinematics of robot arm. 
                    %x position is calculated
                    P_endeff_x_inverse = L2*cos(the1_range(iter1) + the2_range(iter2)) + L1*cos(the1_range(iter1));
                    %y position is calculated
                    P_endeff_y_inverse = L2*sin(the1_range(iter1) + the2_range(iter2)) + L1*sin(the1_range(iter1));
                    %z position is assign to zero because, we do not work z plane
                    P_endeff_z = 0;
                    %Calculated values is placed to Endeff_Kin_inverse matrix the1(in
                    %radian), the2 (in radian), the3 (in radian) x, y and z
                    %position, respectively
                    Endeff_Kin_inverse (index,1:6) = [the1_range(iter1) the2_range(iter2) -the1_range(iter1)-the2_range(iter2) P_endeff_x_inverse P_endeff_y_inverse P_endeff_z];
                    index=index+1;  %next index
       end   
end

%maximum points of Endeff_Kin_inverse
disp('Maximum points') %write output to command window
maxx=max(Endeff_Kin_inverse(:,4));      %max Px value
disp(maxx) %write output to command window
maxy=max(Endeff_Kin_inverse(:,5));      %max Py value
disp(maxy) %write output to command window
maxz=max(Endeff_Kin_inverse(:,6));      %max Pz value
disp(maxz) %write output to command window

%minimum points of Endeff_Kin_inverse
disp('Minimum points') %write output to command window
minx=min(Endeff_Kin_inverse(:,4));      %min Px value
disp(minx) %write output to command window
miny=min(Endeff_Kin_inverse(:,5));      %min Py value
disp(miny) %write output to command window
minz=min(Endeff_Kin_inverse(:,6));      %min Pz value
disp(minz) %write output to command window


%Angles the1, the2 and the3 merged and other remainders are reset 
Thetas = zeros(size(Endeff_Kin_inverse(:,1),1),9);


pass_point                  = zeros (1,7); % to store all passing points 
pass_point_first_section    = zeros (1,7); %to store passing points in zone 1 
pass_point_second_section   = zeros (1,7); %to store passing points in zone 2 
pass_point_third_section    = zeros (1,7); %to store passing points in zone 3 
pass_point_fourth_section   = zeros (1,7); %to store passing points in zone 4 
pass_point_else             = zeros (1,7); %if there is a point where the pass is passed except from 4 zones to store(guaranteed) 

sign = 1; % to define sign 
ctr=1; ctr_1=1; ctr_2=1; ctr_3=1; ctr_4=1; ctr_5=1; %count iterations
for i=1:1:size(Endeff_Kin_inverse(:,1),1)
    
    the1 = Endeff_Kin_inverse(i,1); %the1 value at i iteration is assign to the1 (in radian)
    the2 = Endeff_Kin_inverse(i,2); %the2 value at i iteration is assign to the2 (in radian)
    the3 = Endeff_Kin_inverse(i,3); %the3 value at i iteration is assign to the3 (in radian)
    Px   = Endeff_Kin_inverse(i,4); %P_endeff_x_inverse value at i iteration is assign to Px 
    Py   = Endeff_Kin_inverse(i,5); %P_endeff_y_inverse value at i iteration is assign to Py
    Pz   = Endeff_Kin_inverse(i,6); %P_endeff_z_inverse value at i iteration is assign to Pz
    the1_degree = the1 * 180 / pi; % convert to degree the1
    the2_degree = the2 * 180 / pi; % convert to degree the2
    the3_degree = the3 * 180 / pi; % convert to degree the3
    
    %To find the inverse kinematics, the2, the1 and the3 will be found respectively. 
    %For the2
    
    if((Px>=0 && Py>=0) || (Px<=0 && Py>=0)) %define zone of the2 because MATLAB works atan2, we work atan
        sign = sign * 1;
    else 
        sign = sign * (-1);
    end
    
    c2 = (Px^2+Py^2-L1^2-L2^2)/(2*L1*L2) ;  %cosine the2 equation
    s2 = sign * sqrt(1-c2^2) ;              %sine the2 equation
    
    sign = 1; % for next iteration, sign is reset
    
        
    %to find pass points
    if( (abs(imag(c2))>0) || (abs(imag(s2))>0) ) %The condition that atan2 cannot define. If there is such a situation, these points are skipped. 
       pass_point (ctr,1:7) = [ c2 the1_degree the2_degree the3_degree Px Py Pz ]; % In that iteration cosine value (radians), the 1 degree, the 2 degree, the 3 degree, x, y and z points are transferred to the pass point, respectively.
       ctr = ctr+1; % counter up
       if( (Px>0) && (Py>=0) ) % For zone 1
          pass_point_first_section (ctr_1,1:7) = [ c2 the1_degree the2_degree the3_degree Px Py Pz ]; % In that iteration cosine value (radians), the 1 degree, the 2 degree, the 3 degree, x, y and z points are transferred to the pass point, respectively.
          ctr_1 = ctr_1+1; % counter up
       elseif( (Px<0) && (Py>0) ) % For zone 2
          pass_point_second_section (ctr_2,1:7) = [ c2 the1_degree the2_degree the3_degree Px Py Pz ]; % In that iteration cosine value (radians), the 1 degree, the 2 degree, the 3 degree, x, y and z points are transferred to the pass point, respectively.
          ctr_2 = ctr_2+1; % counter up
       elseif( (Px<0) && (Py<0) ) % For zone 3
          pass_point_third_section (ctr_3,1:7) = [ c2 the1_degree the2_degree the3_degree Px Py Pz ]; % In that iteration cosine value (radians), the 1 degree, the 2 degree, the 3 degree, x, y and z points are transferred to the pass point, respectively.
          ctr_3 = ctr_3+1; % counter up
       elseif( (Px>0) && (Py<0) ) % For zone 4
          pass_point_fourth_section (ctr_4,1:7) = [ c2 the1_degree the2_degree the3_degree Px Py Pz ]; % In that iteration cosine value (radians), the 1 degree, the 2 degree, the 3 degree, x, y and z points are transferred to the pass point, respectively.
          ctr_4 = ctr_4+1; % counter up
       else %If there is a point outside these 4 regions (theoretically not) 
          pass_point_else (ctr_5,1:7) = [ c2 the1_degree the2_degree the3_degree Px Py Pz ]; % In that iteration cosine value (radians), the 1 degree, the 2 degree, the 3 degree, x, y and z points are transferred to the pass point, respectively.
          ctr_5 = ctr_5+1; % counter up
       end
      continue;
    end
    acc(i,1:4)=[s2 c2 atan2(s2,c2) atan(s2/c2)]; %to compare atan2 values and atan
    the2_IK = atan2(s2,c2)  ;               %the2 inverse kinematic equation
    
    
    
    %For the1
    k1 = L1+L2*cos(the2_IK);                %The distance of the k1 base and the joint at the far end of the robot arm in the X axis   
    k2 = L2*sin(the2_IK) ;                  %The distance of the k2 base and the joint at the far end of the robot arm in the Y axis 

    the1_IK = atan2(Py,Px)-atan2(k2,k1) ;   %the1 inverse kinematic equation

    %For the3
    the3_IK = -the1_IK-the2_IK ;            %the3 inverse kinematic equation
    
    Thetas(i,1:9)=[the1 the2 the3 the1_IK the2_IK the3_IK Px Py Pz]; %The1, the2, the3, the1 inverse, the2 inverse, the3 inverse, x, y and z points are assigned to the Thetas matrix, respectively. 
    diff(i,1:10) = [i the1-the1_IK the2-the2_IK the3-the3_IK the1 the2 the3 the1_IK the2_IK the3_IK]; %The differences between the angles in each iteration are found and transferred to diff matrix. 
    fprintf('Iteration --> %d :\n', i);     %write screen iteration numbers
    disp(Thetas(i,1:9))                     % #i row of thetas matrix  is pushing to screen.
end


% write all datas to excel
xlswrite('data_inverse.xlsx',Endeff_Kin_inverse);
%write pass points to excel
xlswrite('pass.xlsx',pass_point);
xlswrite('pass_first.xlsx',pass_point_first_section);
xlswrite('pass_second.xlsx',pass_point_second_section);
xlswrite('pass_third.xlsx',pass_point_third_section);
xlswrite('pass_fourth.xlsx',pass_point_fourth_section);
xlswrite('pass_else.xlsx',pass_point_else);


%compare degree values 
Theta_Comparison = [Thetas(:,1:6) (Thetas(:,1:3)-Thetas(:,4:6))];
rad2deg(Theta_Comparison);


disp('Maximum angle difference  :') %write determined values to screen
disp('Maximum x degree :') %write determined values to screen
rad2deg(max(Theta_Comparison(:,7))) %convert from radian to degree
disp('Maximum y degree :') %write determined values to screen
rad2deg(max(Theta_Comparison(:,8))) %convert from radian to degree
disp('Maximum z degree :') %write determined values to screen
rad2deg(max(Theta_Comparison(:,9))) %convert from radian to degree
disp('Minimum angle difference :') %write determined values to screen
disp('Minimum x degree :') %write determined values to screen
rad2deg(min(Theta_Comparison(:,7))) %convert from radian to degree
disp('Minimum y degree :') %write determined values to screen
rad2deg(min(Theta_Comparison(:,8))) %convert from radian to degree
disp('Minimum z degree :') %write determined values to screen
rad2deg(min(Theta_Comparison(:,9))) %convert from radian to degree

%%%%plots%%%%
 
% plot pass_point_first_section
figure(1)
scatter3(pass_point_first_section(:,5),pass_point_first_section(:,6),pass_point_first_section(:,7))
xlabel('x')
ylabel('y')
zlabel('z')
title('pass point first section')

% plot pass_point_second_section
figure(2)
scatter3(pass_point_second_section(:,5),pass_point_second_section(:,6),pass_point_second_section(:,7))
xlabel('x')
ylabel('y')
zlabel('z')
title('pass point second section')
 
% plot pass_point_third_section
figure(3)
scatter3(pass_point_third_section(:,5),pass_point_third_section(:,6),pass_point_third_section(:,7))
xlabel('x')
ylabel('y')
zlabel('z')
title('pass point third section')
 
% plot pass_point_fourth_section
figure(4)
scatter3(pass_point_fourth_section(:,5),pass_point_fourth_section(:,6),pass_point_fourth_section(:,7))
xlabel('x')
ylabel('y')
zlabel('z')
title('pass point fourth section')
 
% plot  pass_point_else
figure(5)
scatter3(pass_point_else(:,5),pass_point_else(:,6),pass_point_else(:,7))
xlabel('x')
ylabel('y')
zlabel('z')
title('pass point else')

%  plot  positions
figure (6)
scatter3(Endeff_Kin_inverse(:,4),Endeff_Kin_inverse(:,5),Endeff_Kin_inverse(:,6))
xlabel('X')
ylabel('Y')
zlabel('Z')


