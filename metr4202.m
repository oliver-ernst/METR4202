function metr4202
% GUI kinematic demo for the METR4202 Robot. Code has been edited and 
% adapted from multiple sources under the given licence.

% Arm Geometry uses CAD2MATDEMO code found on MATLAB File Exchange
% https://au.mathworks.com/matlabcentral/fileexchange/3642-cad2matdemo-m

% Original puma3d.m code by Don Riley can be found in at:
% https://au.mathworks.com/matlabcentral/fileexchange/14932-3d-puma-robot-demo

% Original code was edited to Puma_Simulation.m by Paschalis Pelitaris. 

% It also contains maximize.m function by Oliver Woodford 
% https://www.mathworks.com/matlabcentral/fileexchange/25471-maximize

% For Licencing see licence.txt
%% GUI Init
clear all
close all 
clearvars
clc
% Load in the Geometry Data
loaddata

% Set Calibration Data

% Load DH Params from dh_params.m
setappdata(0, 'DH', dh_params);
DH = getappdata(0, 'DH');

% Set t1 home angle
setappdata(0, 't1_home', 32);
t1_home = getappdata(0,'t1_home');

% Set t1 home angle offset
setappdata(0, 't1_offset', 90);
t1_offset = getappdata(0,'t1_offset');

% Set t1 minimum angle
setappdata(0, 't1_min_set', -110);
t1_min_set = getappdata(0, 't1_min_set');

% Set t1 maximum angle
setappdata(0, 't1_max_set', 110);
t1_max_set = getappdata(0, 't1_max_set');

% Set t2 home angle
setappdata(0, 't2_home', 6);
t2_home = getappdata(0,'t2_home');

% Set t2 minimum angle
setappdata(0, 't2_min_set', -175);
t2_min_set = getappdata(0, 't2_min_set');

% Set t2 maximum angle
setappdata(0, 't2_max_set', 175);
t2_max_set = getappdata(0, 't2_max_set');

% Set EE home, min and max height
setappdata(0, 'EE_home', 10);
EE_home = getappdata(0,'EE_home');
EE_min_set = 10;
EE_max_set = 80;

% Set px home, min and max point
setappdata(0, 'px_home', -210);
px_home = getappdata(0,'px_home');
px_min_set = -210;
px_max_set = 210;

% Set py home, min and max point
setappdata(0, 'py_home', 300);
py_home = getappdata(0,'py_home');
py_min_set = 0;
py_max_set = 367; %Equal a2+a3 lengths %might change to be max of workspace

% Set pz home, min and max point
setappdata(0, 'pz_home', 80);
pz_home = getappdata(0,'pz_home');
pz_min_set = EE_min_set;
pz_max_set = EE_max_set;

%{
setappdata(0, 'r11_home', 0);
r11_home = getappdata(0,'r11_home');
setappdata(0, 'r21_home', 0);
r21_home = getappdata(0,'r21_home');
setappdata(0, 'r31_home', 0);
r31_home = getappdata(0,'r31_home');
setappdata(0, 'r13_home', 0);
r13_home = getappdata(0,'r13_home');
setappdata(0, 'r23_home', 0);
r23_home = getappdata(0,'r23_home');
setappdata(0, 'r33_home', 0);
r33_home = getappdata(0,'r33_home');
%}

% Initialise the home GUI screen
InitHome 

%
% Create the push buttons: pos is: [left bottom width height]
rnd_demo = uicontrol(fig_1,'String','Random Move','callback',@rnd_demo_button_press,...
    'Position',[25 5 80 20]);

clr_trail = uicontrol(fig_1,'String','Clr Trail','callback',@clr_trail_button_press,...
    'Position',[125 5 60 20]);
%
home = uicontrol(fig_1,'String','Home','callback',@home_button_press,...
    'Position',[200 5 70 20]);
%
DH_Tm = uicontrol(fig_1,'String','DH & T matrix','callback',@DH_Tm_button_press,...
    'Position',[285 5 100 20]);
%
InverseKin = uicontrol(fig_1, 'units','normalized', 'String','Inverse Kinematics','callback',@InverseKin_button_press,...
    'Position',[0.0781 0.385 0.0781 0.029]);

%
%% Kinematics Panel
%
K_p = uipanel(fig_1, 'units','normalized', 'Position',[0.0156 0.05625 0.207 ,0.31], ...
    'Title','Forward Kinematics','FontSize',11);

%% Inverse Kinematics Panel
%
IK_p = uipanel(fig_1, 'units','normalized', 'Position',[0.0156 0.415 0.207 0.495],...
    'Title','Inverse Kinematics','FontSize',11);

%% D-H Table Panel
%
DH_p = uipanel(fig_1, 'units','normalized', 'Position',[0.7813 0.05625 0.207 0.365],...
    'Visible', 'On', 'Title','Denavit–Hartenberg','FontSize',11);

%% Transformation matrix Panel
%
Tm_p = uipanel(fig_1, 'units','normalized', 'Position',[0.75 0.43 0.25 0.3],...
    'Visible', 'On', 'Title','Transformation Matrix','FontSize',11);

%
%     Angle    Range                Default Name
%     Theta 1: 200 (-110 to 110)    90       Base Joint 
%     Theta 2: 340 (-175 to 175)    0        Middle Joint
%     EE:      90  (10 to 100)      10       End Joint    
%{ 
WONT NEED THESE NOW
setappdata(0, 't1_home', 32);
t1_home = getappdata(0,'t1_home');
setappdata(0, 't1_offset', 90);
t1_offset = getappdata(0,'t1_offset');
setappdata(0, 't1_min_set', -110);
t1_min_set = getappdata(0, 't1_min_set');
setappdata(0, 't1_max_set', 110);
t1_max_set = getappdata(0, 't1_max_set');
setappdata(0, 't2_home', 6);
t2_home = getappdata(0,'t2_home');
setappdata(0, 't2_min_set', -175);
t2_min_set = getappdata(0, 't2_min_set');
setappdata(0, 't2_max_set', 175);
t2_max_set = getappdata(0, 't2_max_set');
setappdata(0, 'EE_home', 10);
EE_home = getappdata(0,'EE_home');
EE_min_set = 10;
EE_max_set = 80;
setappdata(0, 'px_home', -210);
px_home = getappdata(0,'px_home');
px_min_set = -210;
px_max_set = 210;
setappdata(0, 'py_home', 300);
py_home = getappdata(0,'py_home');
py_min_set = 0;
py_max_set = 367;
setappdata(0, 'pz_home', 80);
pz_home = getappdata(0,'pz_home');
pz_min_set = EE_min_set;
pz_max_set = EE_max_set;
setappdata(0, 'r11_home', 0);
r11_home = getappdata(0,'r11_home');
setappdata(0, 'r21_home', 0);
r21_home = getappdata(0,'r21_home');
setappdata(0, 'r31_home', 0);
r31_home = getappdata(0,'r31_home');
setappdata(0, 'r13_home', 0);
r13_home = getappdata(0,'r13_home');
setappdata(0, 'r23_home', 0);
r23_home = getappdata(0,'r23_home');
setappdata(0, 'r33_home', 0);
r33_home = getappdata(0,'r33_home');
%}

LD = 105; % Left, used to set the GUI.
HT = 18;  % Height
BT = 156; % Bottom

%%  GUI buttons for Theta 1.  pos is: [left bottom width height]
t1_slider = uicontrol(K_p,'style','slider',...
    'Max',t1_max_set,'Min',t1_min_set,'Value',t1_home,...
    'SliderStep',[0.05 0.2],...       % (160*2*0,05)  
    'callback',@t1_slider_button_press,...
    'Position',[LD BT 120 HT]);
t1_min = uicontrol(K_p,'style','text',...
    'String',int2str(t1_min_set),...
    'Position',[LD-30 BT+1 26 HT-4]); % L, from bottom, W, H
t1_max = uicontrol(K_p,'style','text',...
    'String',int2str(t1_max_set),...
    'Position',[LD+125 BT+1 26 HT-4]); % L, B, W, H
t1_text = uibutton(K_p,'style','text',...  % Nice program Doug. Need this
    'String','\theta_1',...                % due to no TeX in uicontrols. 
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
t1_edit = uicontrol(K_p,'style','edit',...
    'String',int2str(t1_home),...
    'callback',@t1_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%
%%  GUI buttons for Theta 2.
BT = 126;   % Bottom
t2_slider = uicontrol(K_p,'style','slider',...
    'Max',t2_max_set,'Min',t2_min_set,'Value',t2_home,...        % Mech. stop limits ! diorthosa kai to 115 se 110
    'SliderStep',[0.05 0.2],...
    'callback',@t2_slider_button_press,...
    'Position',[LD BT 120 HT]);
t2_min = uicontrol(K_p,'style','text',...
    'String',int2str(t2_min_set),...
    'Position',[LD-30 BT+1 26 HT-4]); % L, from bottom, W, H
t2_max = uicontrol(K_p,'style','text',...
    'String',int2str(t2_max_set),...
    'Position',[LD+125 BT+1 26 HT-4]); % L, B, W, H
t2_text = uibutton(K_p,'style','text',...
    'String','\theta_2',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
t2_edit = uicontrol(K_p,'style','edit',...
    'String',int2str(t2_home),...
    'callback',@t2_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%
%%  GUI buttons for End Effector 3.
BT = 96;   % Bottom
EE_slider = uicontrol(K_p,'style','slider',...
    'Max',EE_max_set,'Min',EE_min_set,'Value',EE_home,...
    'SliderStep',[0.05 0.2],...
    'callback',@EE_slider_button_press,...
    'Position',[LD BT 120 HT]);
EE_min = uicontrol(K_p,'style','text',...
    'String',int2str(EE_min_set),...
    'Position',[LD-30 BT+1 26 HT-4]); % L, from bottom, W, H
EE_max = uicontrol(K_p,'style','text',...
    'String',int2str(EE_max_set),...
    'Position',[LD+125 BT+1 26 HT-4]); % L, B, W, H
EE_txt = uibutton(K_p,'style','text',...
    'String','EE',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
EE_edit = uicontrol(K_p,'style','edit',...
    'String',int2str(EE_home),...
    'callback',@EE_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%
BT = 275;

%%  GUI buttons for Px axis.

Px_slider = uicontrol(IK_p,'style','slider',...
    'Max',px_max_set,'Min',px_min_set,'Value',px_home,...        % to 1250 isws den einai swsto
    'SliderStep',[0.05 0.2],...
    'callback',@Px_slider_button_press,...
    'Position',[LD BT 120 HT]);
Px_min = uicontrol(IK_p,'style','text',...
    'String',int2str(px_min_set),...
    'Position',[LD-33 BT+1 32 HT-4]); % L, from bottom, W, H
Px_max = uicontrol(IK_p,'style','text',...
    'String',int2str(px_max_set),...
    'Position',[LD+122 BT+1 32 HT-4]); % L, B, W, H
Px_text = uibutton(IK_p,'style','text',...
    'String','P_x',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
Px_edit = uicontrol(IK_p,'style','edit',...
    'String',int2str(px_home),...
    'callback',@Px_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H 
%%  GUI buttons for Py axis.
BT = BT-30;
Py_slider = uicontrol(IK_p,'style','slider',...
    'Max',py_max_set,'Min',py_min_set,'Value',py_home,...        % to 1250 isws den einai swsto
    'SliderStep',[0.05 0.2],...
    'callback',@Py_slider_button_press,...
    'Position',[LD BT 120 HT]);
Py_min = uicontrol(IK_p,'style','text',...
    'String',int2str(py_min_set),...
    'Position',[LD-33 BT+1 32 HT-4]); % L, from bottom, W, H
Py_max = uicontrol(IK_p,'style','text',...
    'String',int2str(py_max_set),...
    'Position',[LD+122 BT+1 32 HT-4]); % L, B, W, H
Py_text = uibutton(IK_p,'style','text',...
    'String','P_y',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
Py_edit = uicontrol(IK_p,'style','edit',...
    'String',int2str(py_home),...
    'callback',@Py_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%%  GUI buttons for Pz axis.
BT = BT-30;
Pz_slider = uicontrol(IK_p,'style','slider',...
    'Max',pz_max_set,'Min',pz_min_set,'Value',pz_home,...        % to 1250 den einai swsto
    'SliderStep',[0.05 0.2],...
    'callback',@Pz_slider_button_press,...
    'Position',[LD BT 120 HT]);
Pz_min = uicontrol(IK_p,'style','text',...
    'String',int2str(pz_min_set),...
    'Position',[LD-33 BT+1 32 HT-4]); % L, from bottom, W, H
Pz_max = uicontrol(IK_p,'style','text',...
    'String',int2str(pz_max_set),...
    'Position',[LD+122 BT+1 32 HT-4]); % L, B, W, H
Pz_text = uibutton(IK_p,'style','text',...
    'String','P_z',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
Pz_edit = uicontrol(IK_p,'style','edit',...
    'String',int2str(pz_home),...
    'callback',@Pz_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%{
    WONT NEED THESE EITHER
%%  GUI buttons for R11 axis.
BT = BT-30;
R11_slider = uicontrol(IK_p,'style','slider',...
    'Max',1,'Min',-1,'Value',0,...        
    'SliderStep',[0.05 0.2],...
    'callback',@R11_slider_button_press,...
    'Position',[LD BT 120 HT]);
R11_min = uicontrol(IK_p,'style','text',...
    'String','-1',...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
R11_max = uicontrol(IK_p,'style','text',...
    'String','+1',...
    'Position',[LD+125 BT+1 25 HT-4]); % L, B, W, H
R11_text = uibutton(IK_p,'style','text',...
    'String','r_1_1',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
R11_edit = uicontrol(IK_p,'style','edit',...
    'String',0,...
    'callback',@R11_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%%  GUI buttons for R21 axis.
BT = BT-30;
R21_slider = uicontrol(IK_p,'style','slider',...
    'Max',1,'Min',-1,'Value',0,...
    'SliderStep',[0.05 0.2],...
    'callback',@R21_slider_button_press,...
    'Position',[LD BT 120 HT]);
R21_min = uicontrol(IK_p,'style','text',...
    'String','-1',...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
R21_max = uicontrol(IK_p,'style','text',...
    'String','+1',...
    'Position',[LD+125 BT+1 25 HT-4]); % L, B, W, H
R21_text = uibutton(IK_p,'style','text',...
    'String','r_2_1',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
R21_edit = uicontrol(IK_p,'style','edit',...
    'String',0,...
    'callback',@R21_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%%  GUI buttons for R31 axis.
BT = BT-30;
R31_slider = uicontrol(IK_p,'style','slider',...
    'Max',1,'Min',-1,'Value',0,...
    'SliderStep',[0.05 0.2],...
    'callback',@R31_slider_button_press,...
    'Position',[LD BT 120 HT]);
R31_min = uicontrol(IK_p,'style','text',...
    'String','-1',...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
R31_max = uicontrol(IK_p,'style','text',...
    'String','+1',...
    'Position',[LD+125 BT+1 25 HT-4]); % L, B, W, H
R31_text = uibutton(IK_p,'style','text',...
    'String','r_3_1',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
R31_edit = uicontrol(IK_p,'style','edit',...
    'String',0,...
    'callback',@R31_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%%  GUI buttons for R13 axis.
BT = BT-30;
R13_slider = uicontrol(IK_p,'style','slider',...
    'Max',1,'Min',-1,'Value',0,...
    'SliderStep',[0.05 0.2],...
    'callback',@R13_slider_button_press,...
    'Position',[LD BT 120 HT]);
R13_min = uicontrol(IK_p,'style','text',...
    'String','-1',...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
R13_max = uicontrol(IK_p,'style','text',...
    'String','+1',...
    'Position',[LD+125 BT+1 25 HT-4]); % L, B, W, H
R13_text = uibutton(IK_p,'style','text',...
    'String','r_1_3',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
R13_edit = uicontrol(IK_p,'style','edit',...
    'String',0,...
    'callback',@R13_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%%  GUI buttons for R23 axis.
BT = BT-30;
R23_slider = uicontrol(IK_p,'style','slider',...
    'Max',1,'Min',-1,'Value',0,...
    'SliderStep',[0.05 0.2],...
    'callback',@R23_slider_button_press,...
    'Position',[LD BT 120 HT]);
R23_min = uicontrol(IK_p,'style','text',...
    'String','-1',...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
R23_max = uicontrol(IK_p,'style','text',...
    'String','+1',...
    'Position',[LD+125 BT+1 25 HT-4]); % L, B, W, H
R23_text = uibutton(IK_p,'style','text',...
    'String','r_2_3',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
R23_edit = uicontrol(IK_p,'style','edit',...
    'String',0,...
    'callback',@R23_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%%  GUI buttons for R33 axis.
BT = BT-30;
R33_slider = uicontrol(IK_p,'style','slider',...
    'Max',1,'Min',-1,'Value',0,...
    'SliderStep',[0.05 0.2],...
    'callback',@R33_slider_button_press,...
    'Position',[LD BT 120 HT]);
R33_min = uicontrol(IK_p,'style','text',...
    'String','-1',...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
R33_max = uicontrol(IK_p,'style','text',...
    'String','+1',...
    'Position',[LD+125 BT+1 25 HT-4]); % L, B, W, H
R33_text = uibutton(IK_p,'style','text',...
    'String','r_3_3',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
R33_edit = uicontrol(IK_p,'style','edit',...
    'String',0,...
    'callback',@R33_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%}
%% Elbow & flip sign
 Elbow_Sign_button = uicontrol(IK_p,'style','radiobutton',...
     'String','Elbow Up / Down',...
     'callback',@elbow_sign_func,...
     'Position',[LD-100 BT-30 130 HT]);
 Flip_Sign_button = uicontrol(IK_p,'style','radiobutton',...
     'String','No Flip',...
     'callback',@flip_sign_func,...
     'Position',[LD+40 BT-30 130 HT]);
 
%% D-H Parameters Table
% from The Development of a Genetic Programming Method for Kinematic Robot
% Calibration by Dolinsky Jens-Uwe.

BT = 195;
%{
WONT NEED THESE EITHER
%Load in D-H Parameters
setappdata(0,'DH',dh_params);
DH = getappdata(0, 'DH');
d1 = DH.d1;
d2 = DH.d2;
a2 = DH.a2;
th3 = DH.th3;
a3 = DH.a3;
al3 = DH.al3;
a4 = DH.a4;
%}
link_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 11, 'String','Link',...
    'Position',[LD-92 BT 20 HT]); % L, B, W, H
    
 link1_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 11, 'String','1',...
    'Position',[LD-92 BT-30 20 HT]); % L, B, W, H
 link2_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 11, 'String','2',...
    'Position',[LD-92 BT-60 20 HT]); % L, B, W, H
 link3_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 11, 'String','3',...
    'Position',[LD-92 BT-90 20 HT]); % L, B, W, H
 link4_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 11, 'String','4',...
    'Position',[LD-92 BT-120 20 HT]); % L, B, W, H
 
    
 Joint_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 11, 'String','Joint Type',...
    'Position',[LD-32 BT 20 HT]); % L, B, W, H

 Joint1_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 9, 'String','Revolute',...
    'Position',[LD-32 BT-30 20 HT]); % L, B, W, H
 Joint2_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 9, 'String','Revolute',...
    'Position',[LD-32 BT-60 20 HT]); % L, B, W, H
 Joint3_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 9, 'String','Revolute',...
    'Position',[LD-32 BT-90 20 HT]); % L, B, W, H
 Joint4_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 9, 'String','Prismatic',...
    'Position',[LD-32 BT-120 20 HT]); % L, B, W, H
 

 Theta_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 12, 'String','\theta *',...
    'FontWeight', 'bold',...
    'Position',[LD+28.5 BT 20 HT]); % L, B, W, H

 Th1_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 9, 'String','\theta_1',...
    'Position',[LD+28.5 BT-30 20 HT]); % L, B, W, H
 Th2_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 9, 'String','\theta_2',...
    'Position',[LD+28.5 BT-60 20 HT]); % L, B, W, H
 Th3_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 9, 'String',int2str(th3),...
    'Position',[LD+28.5 BT-90 20 HT]); % L, B, W, H
 Th4_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 9, 'String','0',...
    'Position',[LD+28.5 BT-120 20 HT]); % L, B, W, H
 

 d_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 12, 'String','d',...
    'Position',[LD+60 BT 20 HT]); % L, B, W, H
 d1_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 9, 'String',int2str(d1),...
    'Position',[LD+60 BT-30 20 HT]); % L, B, W, H
 d2_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 9, 'String',int2str(d2),...
    'Position',[LD+60 BT-60 20 HT]); % L, B, W, H
 d3_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 9, 'String','0',...
    'Position',[LD+60 BT-90 20 HT]); % L, B, W, H
 d4_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 9, 'String','0',...
    'Position',[LD+60 BT-120 20 HT]); % L, B, W, H


 a_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 12, 'String','a',...
    'Position',[LD+92.5 BT 20 HT]); % L, B, W, H   
 a1_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 9, 'String','0',...
    'Position',[LD+92.5 BT-30 20 HT]); % L, B, W, H
 a2_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 9, 'String',int2str(a2),...
    'Position',[LD+92.5 BT-60 20 HT]); % L, B, W, H
 a3_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 9, 'String',int2str(a3),...
    'Position',[LD+92.5 BT-90 20 HT]); % L, B, W, H
 a4_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 9, 'String',int2str(a4),...
    'Position',[LD+92.5 BT-120 20 HT]); % L, B, W, H

    
 alpha_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 12, 'String','\alpha',...
    'FontWeight', 'bold',...
    'Position',[LD+125 BT 20 HT]); % L, B, W, H
 alp1_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 9, 'String','0',...
    'Position',[LD+125 BT-30 20 HT]); % L, B, W, H
 alp2_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 9, 'String','0',...
    'Position',[LD+125 BT-60 20 HT]); % L, B, W, H
 alp3_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 9, 'String',int2str(al3),...
    'Position',[LD+125 BT-90 20 HT]); % L, B, W, H
 alp4_txt  = uibutton(DH_p,'style','text',...
    'FontSize', 9, 'String','0',...
    'Position',[LD+125 BT-120 20 HT]); % L, B, W, H
 

%% Transformation matrix Table

 nx_txt  = uibutton(Tm_p,'style','text',...
    'FontSize', 11, 'String','n_x=',...
    'Position',[LD-75 BT-60 20 HT]); % L, B, W, H 
 ny_txt  = uibutton(Tm_p,'style','text',...
    'FontSize', 11, 'String','n_y=',...
    'Position',[LD-75 BT-100 20 HT]); % L, B, W, H
 nz_txt  = uibutton(Tm_p,'style','text',...
    'FontSize', 11, 'String','n_z=',...
    'Position',[LD-75 BT-140 20 HT]); % L, B, W, H
 lr1_txt  = uibutton(Tm_p,'style','text',...
    'FontSize', 11, 'String','0',...
    'Position',[LD-75 BT-180 20 HT]); % L, B, W, H

 ox_txt  = uibutton(Tm_p,'style','text',...
    'FontSize', 11, 'String','o_x= ',...
    'Position',[LD BT-60 20 HT]); % L, B, W, H 
 oy_txt  = uibutton(Tm_p,'style','text',...
    'FontSize', 11, 'String','o_y= ',...
    'Position',[LD BT-100 20 HT]); % L, B, W, H
 oz_txt  = uibutton(Tm_p,'style','text',...
    'FontSize', 11, 'String','o_z= ',...
    'Position',[LD BT-140 20 HT]); % L, B, W, H
 lr2_txt  = uibutton(Tm_p,'style','text',...
    'FontSize', 11, 'String','0',...
    'Position',[LD BT-180 20 HT]); % L, B, W, H

 ax_txt  = uibutton(Tm_p,'style','text',...
    'FontSize', 11, 'String','a_x=',...
    'Position',[LD+85 BT-60 20 HT]); % L, B, W, H 
 ay_txt  = uibutton(Tm_p,'style','text',...
    'FontSize', 11, 'String','a_y=',...
    'Position',[LD+85 BT-100 20 HT]); % L, B, W, H
 az_txt  = uibutton(Tm_p,'style','text',...
    'FontSize', 11, 'String','a_z=',...
    'Position',[LD+85 BT-140 20 HT]); % L, B, W, H
 lr3_txt  = uibutton(Tm_p,'style','text',...
    'FontSize', 11, 'String','0',...
    'Position',[LD+85 BT-180 20 HT]); % L, B, W, H
 
 px_txt  = uibutton(Tm_p,'style','text',...
    'FontSize', 11, 'String','p_x=',...
    'Position',[LD+165 BT-60 20 HT]); % L, B, W, H 
 py_txt  = uibutton(Tm_p,'style','text',...
    'FontSize', 11, 'String','p_y=',...
    'Position',[LD+165 BT-100 20 HT]); % L, B, W, H
 pz_txt  = uibutton(Tm_p,'style','text',...
    'FontSize', 11, 'String','p_z= ',...
    'Position',[LD+165 BT-140 20 HT]); % L, B, W, H
 lr4_txt  = uibutton(Tm_p,'style','text',...
    'FontSize', 11, 'String','1',...
    'Position',[LD+165 BT-180 20 HT]); % L, B, W, H

%% Slider for Theta 1 motion.
% 
    function t1_slider_button_press(h,dummy)
        slider_value = round(get(h,'Value'));
        set(t1_edit,'string',slider_value);
        T_Old = getappdata(0,'ThetaOld');
        t2old = T_Old(2); EEold = T_Old(3);
        metr_ani(slider_value+t1_offset,t2old,EEold,10,'n')
    end
%
%% Slider for Theta 2 motion.
    function t2_slider_button_press(h,dummy)
        slider_value = round(get(h,'Value'));
        set(t2_edit,'string',slider_value);
        T_Old = getappdata(0,'ThetaOld');
        t1old = T_Old(1); EEold = T_Old(3);
        metr_ani(t1old,slider_value,EEold,10,'n')
    end
%
%% Slider for End Effector position.
    function EE_slider_button_press(h,dummy)
        slider_value = round(get(h,'Value'));
        set(EE_edit,'string',slider_value);
        T_Old = getappdata(0,'ThetaOld');
        t1old = T_Old(1); t2old = T_Old(2);
        metr_ani(t1old,t2old,slider_value,10,'n')
    end
%
%% Slider for Px motion.
%
    function Px_slider_button_press(h,dummy)
        slider_value = get(h,'Value');
        set(Px_edit,'string',slider_value);
    end
%
%% Slider for Py motion.
%
    function Py_slider_button_press(h,dummy)
        slider_value = get(h,'Value');
        set(Py_edit,'string',slider_value);
    end
%
%% Slider for Pz motion.
%
    function Pz_slider_button_press(h,dummy)
        slider_value = get(h,'Value');
        set(Pz_edit,'string',slider_value);
    end
%
%{ 
WONT NEED THESE EITHER
%% Slider for R11 motion.
%
    function R11_slider_button_press(h,dummy)
        slider_value = get(h,'Value');
        set(R11_edit,'string',slider_value);
    end
%
%% Slider for R21 motion.
%
    function R21_slider_button_press(h,dummy)
        slider_value = get(h,'Value');
        set(R21_edit,'string',slider_value);
    end
%
%% Slider for R31 motion.
%
    function R31_slider_button_press(h,dummy)
        slider_value = get(h,'Value');
        set(R31_edit,'string',slider_value);
    end
%
%% Slider for R13 motion.
%
    function R13_slider_button_press(h,dummy)
        slider_value = get(h,'Value');
        set(R13_edit,'string',slider_value);
    end
    %
%% Slider for R23 motion.
%
    function R23_slider_button_press(h,dummy)
        slider_value = get(h,'Value');
        set(R23_edit,'string',slider_value);
    end
%
%% Slider for R33 motion.
%
    function R33_slider_button_press(h,dummy)
        slider_value = get(h,'Value');
        set(R33_edit,'string',slider_value);
    end
%
%}
%% Edit box for Theta 1 motion.
%
     function t1_edit_button_press(h,dummy)
        user_entry = check_edit(h,t1_min_set,t1_max_set,t1_home,t1_edit);
        set(t1_slider,'Value',user_entry);  
        T_Old = getappdata(0,'ThetaOld');   % Current pose    
        %
        t2old = T_Old(2); EEold = T_Old(3);
        %
        metr_ani(user_entry+t1_offset,t2old,EEold,10,'n')
    end
%
%% Edit box for Theta 2 motion.
%
    function t2_edit_button_press(h,dummy)
        user_entry = check_edit(h,t2_min_set,t2_max_set,t2_home,t2_edit);
        set(t2_slider,'Value',user_entry);  
        T_Old = getappdata(0,'ThetaOld');   % Current pose    
        %
        t1old = T_Old(1); EEold = T_Old(3);
        %
        metr_ani(t1old,user_entry,EEold,10,'n')
    end
%% Edit box for End Effector position.
%
    function EE_edit_button_press(h,dummy)
        user_entry = check_edit(h,EE_min_set,EE_max_set,EE_home,EE_edit);
        set(EE_slider,'Value',user_entry);  
        T_Old = getappdata(0,'ThetaOld');   % Current pose    
        %
        t1old = T_Old(1); t2old = T_Old(2);
        %
        metr_ani(t1old,t2old,user_entry,10,'n')
    end
%% Edit box for Px motion.
%
    function Px_edit_button_press(h,dummy)
        user_entry = check_edit(h,px_min_set,px_max_set,px_home,Px_edit);
        set(Px_slider,'Value',user_entry);  
        setappdata (0,'Pxold',user_entry);
    end
%% Edit box for Py motion.
%
    function Py_edit_button_press(h,dummy)
        user_entry = check_edit(h,py_min_set,py_max_set,py_home,Py_edit);
        set(Py_slider,'Value',user_entry);  
        setappdata (0,'Pyold',user_entry);
    end
%% Edit box for Pz motion. 
%
    function Pz_edit_button_press(h,dummy)
        user_entry = check_edit(h,pz_min_set,pz_max_set,pz_home,Pz_edit);
        set(Pz_slider,'Value',user_entry);  
        setappdata (0,'Pzold',user_entry);
    end
%{ 
WONT NEED THESE
%% Edit box for R11 motion.
%
    function R11_edit_button_press(h,dummy)
        user_entry = check_edit(h,-1,1,r11_home,R11_edit);
        set(R11_slider,'Value',user_entry);  
        setappdata (0,'R11old',user_entry);
    end
%% Edit box for R21 motion.
%
    function R21_edit_button_press(h,dummy)
        user_entry = check_edit(h,-1,1,r21_home,R21_edit);
        set(R21_slider,'Value',user_entry);  
        setappdata (0,'R21old',user_entry);
    end
%% Edit box for R31 motion.
%
    function R31_edit_button_press(h,dummy)
        user_entry = check_edit(h,-1,1,r31_home,R31_edit);
        set(R31_slider,'Value',user_entry);  
        setappdata (0,'R31old',user_entry);
    end
%% Edit box for R13 motion.
%
    function R13_edit_button_press(h,dummy)
        user_entry = check_edit(h,-1,1,r13_home,R13_edit);
        set(R13_slider,'Value',user_entry);  
        setappdata (0,'R13old',user_entry);
    end
%% Edit box for R23 motion.
%
    function R23_edit_button_press(h,dummy)
        user_entry = check_edit(h,-1,1,r23_home,R23_edit);
        set(R23_slider,'Value',user_entry);  
        setappdata (0,'R23old',user_entry);
    end
%% Edit box for R33 motion.
%
    function R33_edit_button_press(h,dummy)
        user_entry = check_edit(h,-1,1,r33_home,R33_edit);
        set(R33_slider,'Value',user_entry);  
        setappdata (0,'R33old',user_entry);
    end
%}
%% UIControl funactions
    function elbow_sign_func (h,dummy)
        if get(Elbow_Sign_button,'Value')== 1
            set(Elbow_Sign_button, 'String', 'Elbow Down'); % Change button's String 
        else
            set(Elbow_Sign_button, 'String', 'Elbow Up');   % Change button's String
        end
    end

    function flip_sign_func (h,dummy)
        if get(Flip_Sign_button,'Value')== 1
            set(Flip_Sign_button, 'String', 'Flip');     % Change button's String
        else
            set(Flip_Sign_button, 'String', 'No Flip');  % Change button's String
        end
    end

    function user_entry = check_edit(h,min_v,max_v,default,h_edit)
        % This function will check the value typed in the text input box
        % against min and max values, and correct errors.
        %
        % h: handle of gui
        % min_v min value to check
        % max_v max value to check
        % default is the default value if user enters non number
        % h_edit is the edit value to update.
        %
        user_entry = str2double(get(h,'string'));
        if isnan(user_entry)
            errordlg(['You must enter a numeric value, defaulting to ',num2str(default),'.'],'Bad Input','modal')
            set(h_edit,'string',default);
            user_entry = default;
        end
        %
        if user_entry < min_v
            errordlg(['Minimum limit is ',num2str(min_v),', using ',num2str(min_v),'.'],'Bad Input','modal')
            user_entry = min_v;
            set(h_edit,'string',user_entry);
        end
        if user_entry > max_v
            errordlg(['Maximum limit is ',num2str(max_v),', using ',num2str(max_v),'.'],'Bad Input','modal')
            user_entry = max_v;
            set(h_edit,'string',user_entry);
        end
    end

    function home_button_press(h,dummy)
        gohome
    end

    function clr_trail_button_press(h,dummy)
        handles = getappdata(0,'patch_h'); 
        Tr = handles(length(handles));
        %
        setappdata(0,'xtrail',0); % used for trail tracking.
        setappdata(0,'ytrail',0); % used for trail tracking.
        setappdata(0,'ztrail',0); % used for trail tracking.
        %
        set(Tr,'xdata',0,'ydata',0,'zdata',0);
    end

    function rnd_demo_button_press(h, dummy)
        %disp('pushed random demo bottom');
        % a = 10; b = 50; x = a + (b-a) * rand(5)
        %     Angle    Range                Default Name
        %     Theta 1: 220 (-110 to 110)    90       Base Joint 
        %     Theta 2: 350 (-175 to 175)    0        Middle Joint
        %     EE:      90  (10 to 100)      10       End Joint  
        t1_home = getappdata(0, 't1_home'); % offsets to define the "home" postition as UP.
        t2_home = getappdata(0, 't2_home');
        EE_home = getappdata(0, 'EE_home');
        t1_offset = getappdata(0, 't1_offset');
        rnd = 0.1*(randi(11)-1);
        th1 = -110 + 220*rnd; % offset for home
        rnd = 0.1*(randi(11)-1);
        th2 = -175 + 350*rnd; % in the UP pos.
        rnd = 0.1*(randi(11)-1);
        EE = 10 + 90*rnd;
        n = 10;
        metr_ani(th1,th2,EE,'y',n)
        set(t1_edit,'string',round(th1)); % Update slider and text.
        set(t1_slider,'Value',round(th1));
        set(Th1_txt,'string',round(th1));
        set(t2_edit,'string',round(th2));
        set(t2_slider,'Value',round(th2));
        set(Th2_txt,'string',round(th2));
        set(EE_edit,'string',round(EE));
        set(EE_slider,'Value',round(EE));
    end
        
    function DH_Tm_button_press(h,dummy)
        if strcmp(DH_p.Visible,'off')
            DH_p.Visible = 'on';
            Tm_p.Visible = 'on';
        else
            DH_p.Visible = 'off';
            Tm_p.Visible = 'off';
        end
    end

    function InverseKin_button_press(h,dummy)
     
          Px=str2double(get(Px_edit,'string')); 
          Py=str2double(get(Py_edit,'string'));
          Pz=str2double(get(Pz_edit,'string'));
          %{ 
          WONT NEED THESE
          R11=str2double(get(R11_edit,'string')); 
          R21=str2double(get(R21_edit,'string'));
          R31=str2double(get(R31_edit,'string'));
          R13=str2double(get(R13_edit,'string')); 
          R23=str2double(get(R23_edit,'string'));
          R33=str2double(get(R33_edit,'string'));
          [th1,th2,EE,noplot] = metrIK(Px,Py,Pz,R11,R21,R31,R13,R23,R33);
          NOTE IK FUNC CHANGE
          %}
          [th1,th2,EE,noplot] = metrIK(Px,Py,Pz);

        if noplot == 0
            metr_ani(th1,th2,EE,10,'n')
            set(t1_edit,'string',round((th1)*100)/100); % Update slider and text.  
            set(t1_slider,'Value',round(th1));
            set(Th1_txt,'string',round(th1));
            set(t2_edit,'string',round((th2)*100)/100);   % *100)/100 to round 2 digits
            set(t2_slider,'Value',round(th2));
            set(Th2_txt,'string',round((th2)*100)/100);
            set(EE_edit,'string',round(EE));
            set(EE_slider,'Value',round(EE));
        else
            h = errordlg('Point unreachable due to joint angle constraints. noplot = 1.','JOINT ERROR');
            waitfor(h);
        end
       
    end


%% Other GUI functions

% This function will initialize the GUI figure
    function InitHome
        % Use forward kinematics to place the robot in a specified
        % configuration.
        % Figure setup data, create a new figure for the GUI
        set(0,'Units','pixels')
        dim = get(0,'ScreenSize');
        fig_1 = figure('doublebuffer','on','Position',[0,35,dim(3)-200,dim(4)-100],...
            'Name',' 3D METR4202 Robot Graphical Demo',...
            'NumberTitle','off');
        hold on;
        light                               % add a default light
        daspect([1 1 1])                    % Setting the aspect ratio
        view(105,25)
        xlabel('X'),ylabel('Y'),zlabel('Z');
        title ('METR4202 Simulation');
        x_lim = 380;
        y_lim = x_lim;
        z_lim = 200;
        axis([-x_lim x_lim -50 y_lim 0 z_lim]);
        grid on;

        s1 = getappdata(0,'Link1_data');
        s2 = getappdata(0,'Link2_data');
        s3 = getappdata(0,'Link3_data');
        s4 = getappdata(0,'Link4_data');
        %
        
        DH = getappdata(0, 'DH');
        d1 = DH.d1;
        d2 = DH.d2;
        a2 = DH.a2;
        th3 = DH.th3;
        a3 = DH.a3;
        al3 = DH.al3;
        a4 = DH.a4;
        
        %The 'home' position, for init.
        %set so that range is within x and y lims
        t1_offset = getappdata(0, 't1_offset')
        t1_home = getappdata(0, 't1_home') %+ t1_offset %+... to adjust
        t2_home = getappdata(0, 't2_home')
        EE_home = getappdata(0, 'EE_home')
        px_home = getappdata(0, 'px_home')
        py_home = getappdata(0, 'py_home')
        pz_home = getappdata(0, 'pz_home')
        
        % Forward Kinematics
        T_01 = tmat(t1_home+t1_offset, d1, 0, 0);
        T_12 = tmat(t2_home, d2, a2, 0);
        T_23 = tmat(th3, 0, a3, al3);
        T_34 = tmat(0, 0, a4, 0);
        
        % Each link frame to base frame transformation
        T_02 = T_01*T_12;
        T_03 = T_02*T_23;
        T_04 = T_03*T_34;
        
        % Actual vertex data of robot links
        Link1 = s1.V1;
        Link2 = (T_01*s2.V2')';
        Link3 = (T_02*s3.V3')';
        Link4 = (T_03*s4.V4')';
        
        % points are no fun to watch, make it look 3d.
        L1 = patch('faces', s1.F1, 'vertices' ,Link1(:,1:3));
        L2 = patch('faces', s2.F2, 'vertices' ,Link2(:,1:3));
        L3 = patch('faces', s3.F3, 'vertices' ,Link3(:,1:3));
        L4 = patch('faces', s4.F4, 'vertices' ,Link4(:,1:3));
        Tr = plot3(0,0,0,'b.'); % holder for trail paths
        
        %{
        COULD PUT THESE BACK??
        format bank
        set (nx_txt,'String',['n_x= ',num2str(round(T_03(1,1), 2))]);
        set (ny_txt,'String',['n_y= ',num2str(round(T_03(2,1), 2))]);
        set (nz_txt,'String',['n_z= ',num2str(round(T_03(3,1), 2))]);
        set (ox_txt,'String',['o_x= ',num2str(round(T_03(1,2), 2))]);
        set (oy_txt,'String',['o_y= ',num2str(round(T_03(2,2), 2))]);
        set (oz_txt,'String',['o_z= ',num2str(round(T_03(3,2), 2))]);
        set (ax_txt,'String',['a_x= ',num2str(round(T_03(1,3), 2))]);
        set (ay_txt,'String',['a_y= ',num2str(round(T_03(2,3), 2))]);
        set (az_txt,'String',['a_z= ',num2str(round(T_03(3,3), 2))]);
        set (px_txt,'String',['p_x= ',num2str(round(T_03(1,4)))]);
        set (py_txt,'String',['p_y= ',num2str(round(T_03(2,4)))]);
        set (pz_txt,'String',['p_z= ',num2str(round(T_03(3,4)))]);
        %}
        setappdata(0,'patch_h',[L1,L2,L3,L4,Tr])
        %
        setappdata(0,'xtrail',0); % used for trail tracking.
        setappdata(0,'ytrail',0); % used for trail tracking.
        setappdata(0,'ztrail',0); % used for trail tracking.
        %
        % Links Colors [R,G,B] (between 0 and 1)
        set(L1, 'facec', [1,0,0]); 
        set(L1, 'EdgeColor','none');
        set(L2, 'facec', [0,1,0]);
        set(L2, 'EdgeColor','none');
        set(L3, 'facec', [0,0,1]);
        set(L3, 'EdgeColor','none');
        set(L4, 'facec', [1,0,1]);
        set(L4, 'EdgeColor','none');
        %
        setappdata(0,'ThetaOld',[t1_home,t2_home,EE_home]);
        %
        setappdata(0,'XYZOld', [px_home,py_home,pz_home]);
        
        maximize;
    end

% This function maximizes the figure
    function maximize(hFig)

    %MAXIMIZE  Maximize a figure window to fill the entire screen
    %
    % Examples:
    %   maximize
    %   maximize(hFig)
    %
    % Maximizes the current or input figure so that it fills the whole of the
    % screen that the figure is currently on. This function is platform
    % independent.
    %
    %IN:
    %   hFig - Handle of figure to maximize. Default: gcf.
        
        if nargin < 1
            hFig = gcf;
        end
        drawnow % Required to avoid Java errors
        jFig = get(handle(hFig), 'JavaFrame'); 
        jFig.setMaximized(true);
    end
    %{
    I DONT USE THIS FUNCTION
    function del_app(varargin)
        %This is the main figure window close function, to remove any
        % app data that may be left due to using it for geometry.
        %CloseRequestFcn
        % here is the data to remove:
        %     Link1_data: [1x1 struct]
        %     Link2_data: [1x1 struct]
        %     Link3_data: [1x1 struct]
        %      Area_data: [1x1 struct]
        %        patch_h: [1x9 double]
        %       ThetaOld: [90 -182 -90]
        %         xtrail: 0
        %         ytrail: 0
        %         ztrail: 0
        % Now remove them.
        rmappdata(0,'Link1_data');
        rmappdata(0,'Link2_data');
        rmappdata(0,'Link3_data');
        rmappdata(0,'Link4_data');
        rmappdata(0,'ThetaOld');
        rmappdata(0,'XYZOld'); 
        rmappdata(0,'Area_data');
        rmappdata(0,'patch_h');
        rmappdata(0,'xtrail');
        rmappdata(0,'ytrail');
        rmappdata(0,'ztrail');
        delete(fig_1);
    end
    %}

%% Here are the functions used to solve the Kinematic Problems for this robot example:
%
% When called this function will simply initialize a plot of the Puma 762
% robot by plotting it in it's home orientation and setting the current
% angles accordingly.
    function gohome()
        t1_offset = getappdata(0, 't1_offset');
        metr_ani(t1_home+t1_offset,t2_home,EE_home,20,'y') % show it animate home
        set(t1_edit,'string',t1_home);
        set(t1_slider,'Value',t1_home);  %At the home position, so all
        set(t2_edit,'string',t2_home);   %sliders and input boxes at home 
        set(t2_slider,'Value',t2_home);
        set(EE_edit,'string',EE_home);
        set(EE_slider,'Value', EE_home);
        
        
        set(Px_edit,'string',px_home);
        set(Px_slider,'Value',px_home);  %Same for Inv Kinematics
        set(Py_edit,'string',py_home);   
        set(Py_slider,'Value',py_home);
        set(Pz_edit,'string',pz_home);
        set(Pz_slider,'Value',pz_home);
        %{
        DONT NEED THESE NOW
        set(R11_edit,'string',r11_home);
        set(R11_slider,'Value',r11_home);
        set(R21_edit,'string',r21_home);
        set(R21_slider,'Value',r21_home);
        set(R31_edit,'string',r31_home);
        set(R31_slider,'Value',r31_home);
        set(R13_edit,'string',r13_home);
        set(R13_slider,'Value',r13_home);  
        set(R23_edit,'string',r23_home);   
        set(R23_slider,'Value',r23_home);
        set(R33_edit,'string',r33_home);
        set(R33_slider,'Value',r33_home);
        %}      
        setappdata(0,'ThetaOld',[t1_home,t2_home,EE_home]);
    end

% This function computes the Forward Kinematics for the Puma 762 robot (with animation capabilities)
    function metr_ani(th1,th2,EE,n,trail)
        % This function will animate the Puma 762 robot given joint angles.
        % n is number of steps for the animation
        % trail is 'y' or 'n' (n = anything else) for leaving a trail.
        %
        DH = dh_params;
        d1 = DH.d1;
        d2 = DH.d2;
        a2 = DH.a2;
        th3 = DH.th3;
        a3 = DH.a3;
        al3 = DH.al3;
        a4 = DH.a4;
        
        t1_offset = getappdata(0, 't1_offset');
        t1_corrected = th1 - t1_offset;
        EE_offset = 40; % Changes simulation appearance and counteracted elsewhere
        ThetaOld = getappdata(0,'ThetaOld');
        %
        th1old = ThetaOld(1);
        th2old = ThetaOld(2);
        EEold = ThetaOld(3);
        out_of_bounds = 0;
        %still working on boundary control here
        if t1_corrected > 0 
            ty = 180 - th2;
            a4 = sqrt(a2^2+a3^2 - 2*a2*a3*cos(ty));
            th_thresh = asind((a3*sin(ty))/a4);
            assignin('base','poo',struct('t1',t1_corrected, 'th_thresh', th_thresh));
            if (t1_corrected+th_thresh) > t1_max_set
                %error
                out_of_bounds = 1;
            end
        elseif t1_corrected < 0
            ty = 180 - th2;
            a4 = sqrt(a2^2+a3^2 - 2*a2*a3*cos(ty));
            th_thresh = asind((a3*sin(ty))/a4);
            assignin('base','poo',struct('t1',t1_corrected, 'th_thresh', th_thresh));
            if (t1_corrected-th_thresh) < t1_min_set
                
                %error
                out_of_bounds = 1;
            end
        end
        
        %
        t1 = linspace(th1old,th1,n); 
        t2 = linspace(th2old,th2,n); 
        ee = linspace(EEold, EE, n);

        n = length(t1);
        if out_of_bounds == 0
            
            for i = 2:1:n
               % Forward Kinematics
                T_01 = tmat(t1(i), d1, 0, 0);
                T_12 = tmat(t2(i), d2, a2, 0);
                T_23 = tmat(th3, 0, a3, al3);
                T_34 = tmat(0, 0, ee(i)-EE_offset, 0); %counteracted here
                
                % Each link frame to base frame transformation
                T_02 = T_01*T_12;
                T_03 = T_02*T_23;
                T_04 = T_03*T_34;
                %
                s1 = getappdata(0,'Link1_data');
                s2 = getappdata(0,'Link2_data');
                s3 = getappdata(0,'Link3_data');
                s4 = getappdata(0,'Link4_data');

                Link1 = s1.V1;
                Link2 = (T_01*s2.V2')';
                Link3 = (T_02*s3.V3')';
                Link4 = (T_04*s4.V4')';

                handles = getappdata(0,'patch_h');           %
                L1 = handles(1);
                L2 = handles(2);
                L3 = handles(3);
                L4 = handles(4);
                Tr = handles(length(handles));
                %
                set(L1,'vertices',Link1(:,1:3));
                set(L1, 'EdgeColor','none');
                set(L2,'vertices',Link2(:,1:3));
                set(L2, 'EdgeColor','none');
                set(L3,'vertices',Link3(:,1:3));
                set(L3, 'EdgeColor','none');
                set(L4,'vertices',Link4(:,1:3));
                set(L4, 'EdgeColor','none');
                
                % store trail in appdata 
                if trail == 'y'
                    x_trail = getappdata(0,'xtrail');
                    y_trail = getappdata(0,'ytrail');
                    z_trail = getappdata(0,'ztrail');
                    %
                    xdata = [x_trail T_04(1,4)];
                    ydata = [y_trail T_04(2,4)];
                    zdata = [z_trail T_04(3,4)-EE_offset]; %counteracted here
                    %
                    setappdata(0,'xtrail',xdata); % used for trail tracking.
                    setappdata(0,'ytrail',ydata); % used for trail tracking.
                    setappdata(0,'ztrail',zdata); % used for trail tracking.
                    %
                    set(Tr,'xdata',xdata,'ydata',ydata,'zdata',zdata);
                end
                drawnow
            %end
            set(Px_edit,'string',num2str(round(T_04(1,4))));
            set(Px_slider,'Value',round(T_04(1,4)));
            set(Py_edit,'string',num2str(round(T_04(2,4))));
            set(Py_slider,'Value',round(T_04(2,4)));
            set(Pz_edit,'string',num2str(round(T_04(3,4))-EE_offset)); 
            set(Pz_slider,'Value',round(T_04(3,4))-EE_offset); %counteracted for PZ here and above 1 line
            set(a4_txt,'String',num2str(round(ee(i))));

            format bank
            set (nx_txt,'String',['n_x= ',num2str(round(T_04(1,1), 2))]);
            set (ny_txt,'String',['n_y= ',num2str(round(T_04(2,1), 2))]);
            set (nz_txt,'String',['n_z= ',num2str(round(T_04(3,1), 2))]);
            set (ox_txt,'String',['o_x= ',num2str(round(T_04(1,2), 2))]);
            set (oy_txt,'String',['o_y= ',num2str(round(T_04(2,2), 2))]);
            set (oz_txt,'String',['o_z= ',num2str(round(T_04(3,2), 2))]);
            set (ax_txt,'String',['a_x= ',num2str(round(T_04(1,3), 2))]);
            set (ay_txt,'String',['a_y= ',num2str(round(T_04(2,3), 2))]);
            set (az_txt,'String',['a_z= ',num2str(round(T_04(3,3), 2))]);
            set (px_txt,'String',['p_x= ',num2str(round(T_04(1,4)))]);
            set (py_txt,'String',['p_y= ',num2str(round(T_04(2,4)))]);
            set (pz_txt,'String',['p_z= ',num2str(round(T_04(3,4))-EE_offset)]); %and counteracted here
            end
            format long
            setappdata(0,'ThetaOld',[th1,th2,EE]);
        else
            %error
            h = errordlg('Solution is off workspace','Out of Bounds!');
            waitfor(h);
            % THIS MIGHT NEED TO BE CHANGED FOR OLD VALUES
            format long
            setappdata(0,'ThetaOld',[th1old,th2old,EEold]);
        end
    end

% This function computes the Inverse Kinematics for the Puma 762 robot
    %function [th1,th2,EE,noplot] = metrIK(Px,Py,Pz,R11,R21,R31,R13,R23,R33)
    function [th1,th2,EE,noplot] = metrIK(Px,Py,Pz)
        EE = Pz;
        %Might not need angles either
        angles= [];
        DH = dh_params;
        d1 = DH.d1;
        d2 = DH.d2;
        a2 = DH.a2;
        th3 = DH.th3;
        a3 = DH.a3;
        al3 = DH.al3;
        a4 = DH.a4;
                
        t1_min_set = getappdata(0, 't1_min_set');
        t1_max_set = getappdata(0, 't1_max_set');
        t2_min_set = getappdata(0, 't2_min_set');
        t2_max_set = getappdata(0, 't2_max_set');
        
        sign1 = 1;
        sign2 = 1;
        nogo = 0;
        noplot = 0;
        elbowSign = get(Elbow_Sign_button,'Value');
        flipSign = get(Flip_Sign_button,'Value');
        
        if elbowSign == 0
            % set elbowSign according to elbow configuration 
            elbowSign=-1;
        end
        % Because the sqrt term in theta1 & theta2 can be + or - we run through
        % all possible combinations (i = 4) and take the first combination that
        % satisfies the joint angle constraints.
        while nogo == 0;
            for i = 1:1:4
                if i == 1
                    sign1 = -elbowSign;
                    sign2 = -elbowSign;
                elseif i == 2
                    sign1 = -elbowSign;
                    sign2 = elbowSign;
                elseif i == 3
                    sign1 = elbowSign;
                    sign2 = -elbowSign;
                else
                    sign1 = elbowSign;
                    sign2 = elbowSign;
                end
                k = (Px^2 + Py^2 - a2^2 - a3^2)/(2*a2*a3)
                if abs(k) > 1
                    h = errordlg('No real solution exists','Imaginary!');
                    waitfor(h);
                    nogo = 1;
                    noplot = 1;
                    break
                end
                
                th2 = sign1*acos(k);
                %{               
                if flipSign == 1
                   th2 = -th2;
                end
                %}
                delta = a2^2 + a3^2 + 2*a2*a3*cos(th2);
                cth1 = (Px*(a2+a3*cos(th2)) + Py*a3*sin(th2))/delta;
                sth1 = (-Px*a3*sin(th2) + Py*(a2+a3*cos(th2)))/delta;
                th1 = sign2*atan2(sth1, cth1);
                
                % From rad to deg
                th1 = th1*180/pi;
                th2 = th2*180/pi;
                
                %Check if all angles are within the mech. limitation. If yes then the solution is accepted
                if th1<=t1_max_set && th1>=t1_min_set && th2<=t2_max_set && th2>=t2_min_set
                    nogo = 1;
                    break
                end
                
                
                if i == 4 && nogo == 0
                    nogo = 1;
                    noplot = 1;
                        break
                end
                
            end
        end
    end



%% uibutton: Create pushbutton with more flexible labeling than uicontrol.
    function [hout,ax_out] = uibutton(varargin)
        %uibutton: Create pushbutton with more flexible labeling than uicontrol.
        % Usage:
        %   uibutton accepts all the same arguments as uicontrol except for the
        %   following property changes:
        %
        %     Property      Values
        %     -----------   ------------------------------------------------------
        %     Style         'pushbutton', 'togglebutton' or 'text', default =
        %                   'pushbutton'.
        %     String        Same as for text() including cell array of strings and
        %                   TeX or LaTeX interpretation.
        %     Interpreter   'tex', 'latex' or 'none', default = default for text()
        %
        % Syntax:
        %   handle = uibutton('PropertyName',PropertyValue,...)
        %   handle = uibutton(parent,'PropertyName',PropertyValue,...)
        %   [text_obj,axes_handle] = uibutton('Style','text',...
        %       'PropertyName',PropertyValue,...)
        %
        % uibutton creates a temporary axes and text object containing the text to
        % be displayed, captures the axes as an image, deletes the axes and then
        % displays the image on the uicontrol.  The handle to the uicontrol is
        % returned.  If you pass in a handle to an existing uicontol as the first
        % argument then uibutton will use that uicontrol and not create a new one.
        %
        % If the Style is set to 'text' then the axes object is not deleted and the
        % text object handle is returned (as well as the handle to the axes in a
        % second output argument).
        %
        % See also UICONTROL.

        % Version: 1.6, 20 April 2006
        % Author:  Douglas M. Schwarz
        % Email:   dmschwarz=ieee*org, dmschwarz=urgrad*rochester*edu
        % Real_email = regexprep(Email,{'=','*'},{'@','.'})


        % Detect if first argument is a uicontrol handle.
        keep_handle = false;
        if nargin > 0
            h = varargin{1};
            if isscalar(h) && ishandle(h) && strcmp(get(h,'Type'),'uicontrol')
                keep_handle = true;
                varargin(1) = [];
            end
        end

        % Parse arguments looking for 'Interpreter' property.  If found, note its
        % value and then remove it from where it was found.
        interp_value = get(0,'DefaultTextInterpreter');
        arg = 1;
        remove = [];
        while arg <= length(varargin)
            v = varargin{arg};
            if isstruct(v)
                fn = fieldnames(v);
                for i = 1:length(fn)
                    if strncmpi(fn{i},'interpreter',length(fn{i}))
                        interp_value = v.(fn{i});
                        v = rmfield(v,fn{i});
                    end
                end
                varargin{arg} = v;
                arg = arg + 1;
            elseif ischar(v)
                if strncmpi(v,'interpreter',length(v))
                    interp_value = varargin{arg+1};
                    remove = [remove,arg,arg+1];
                end
                arg = arg + 2;
            elseif arg == 1 && isscalar(v) && ishandle(v) && ...
                    any(strcmp(get(h,'Type'),{'figure','uipanel'}))
                arg = arg + 1;
            else
                error('Invalid property or uicontrol parent.')
            end
        end
        varargin(remove) = [];

        % Create uicontrol, get its properties then hide it.
        if keep_handle
            set(h,varargin{:})
        else
            h = uicontrol(varargin{:});
        end
        s = get(h);
        if ~any(strcmp(s.Style,{'pushbutton','togglebutton','text'}))
            delete(h)
            error('''Style'' must be pushbutton, togglebutton or text.')
        end
        set(h,'Visible','off')

        % Create axes.
        parent = get(h,'Parent');
        ax = axes('Parent',parent,...
            'Units',s.Units,...
            'Position',s.Position,...
            'XTick',[],'YTick',[],...
            'XColor',s.BackgroundColor,...
            'YColor',s.BackgroundColor,...
            'Box','on',...
            'Color',s.BackgroundColor);
        % Adjust size of axes for best appearance.
        set(ax,'Units','pixels')
        pos = round(get(ax,'Position'));
        if strcmp(s.Style,'text')
            set(ax,'Position',pos + [0 1 -1 -1])
        else
            set(ax,'Position',pos + [4 4 -8 -8])
        end
        switch s.HorizontalAlignment
            case 'left'
                x = 0.0;
            case 'center'
                x = 0.5;
            case 'right'
                x = 1;
        end
        % Create text object.
        text_obj = text('Parent',ax,...
            'Position',[x,0.5],...
            'String',s.String,...
            'Interpreter',interp_value,...
            'HorizontalAlignment',s.HorizontalAlignment,...
            'VerticalAlignment','middle',...
            'FontName',s.FontName,...
            'FontSize',s.FontSize,...
            'FontAngle',s.FontAngle,...
            'FontWeight',s.FontWeight,...
            'Color',s.ForegroundColor);

        % If we are creating something that looks like a text uicontrol then we're
        % all done and we return the text object and axes handles rather than a
        % uicontrol handle.
        if strcmp(s.Style,'text')
            delete(h)
            if nargout
                hout = text_obj;
                ax_out = ax;
            end
            return
        end

        % Capture image of axes and then delete the axes.
        frame = getframe(ax);
        delete(ax)

        % Build RGB image, set background pixels to NaN and put it in 'CData' for
        % the uicontrol.
        if isempty(frame.colormap)
            rgb = frame.cdata;
        else
            rgb = reshape(frame.colormap(frame.cdata,:),[pos([4,3]),3]);
        end
        size_rgb = size(rgb);
        rgb = double(rgb)/255;
        back = repmat(permute(s.BackgroundColor,[1 3 2]),size_rgb(1:2));
        isback = all(rgb == back,3);
        rgb(repmat(isback,[1 1 3])) = NaN;
        set(h,'CData',rgb,'String','','Visible',s.Visible)

        % Assign output argument if necessary.
        if nargout
            hout = h;
        end

    end    
end
% Finally.
% Finally. PP.
% Finally. METR4202 2018 - PP