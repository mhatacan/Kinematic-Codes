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
the3_s = sin(the3); % sine value of the1 is assigned to the3_s


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
% P_endeff = [L2*cos(the1 + the2) + L1*cos(the1) + L3*cos(the1 + the2 + the3)
%             L2*sin(the1 + the2) + L1*sin(the1) + L3*sin(the1 + the2 + the3)
%              0 ];


