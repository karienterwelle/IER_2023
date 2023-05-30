%% Introduction + Important settings
% This section determines the range of kp- and kd-values for which an
% adequate step response (within 10% overshoot or offset) can be achieved.
% A predetermined set of kd-values (0 to 10 for knee or 0 to 2000 for hip) is explored and kp-values are increased
% from 0 to the point of passing the 10% overshoot. For every combination
% of kd and kp values it is saved whether the step-response was adequate. 
% For adequate running of these sections, settings need to be altered in the LOKOMAT_Simulation.slx file for
% either the step response of the knee joint or the hip joint. Read this carefully! 

% FOR STEP RESPONSE OF KNEE FLEXION: 
% - Ensure that the manual switch (SWITCH 3) between the predetermined trajectory and
%   the step input (STEP 3) is set to STEP 3.
% - Put the 'Final value' of STEP 3 to -(1/180)*pi !!Pay attention to
% negative sign!!
% - Ensure that the manual switches (SWITCH 1 & SWITCH 2) between the
%   predetermined trajectory and the step input (STEP 1 & STEP 2) are set to
%   STEP 1 & STEP 2. 
% - Put the 'Final value' of STEP 1 & STEP 2 to 0.
% - Ensure that the manual switch (SWITCH 4) between CONSTANT 1 and the
%   controller output line, is set to CONSTANT 1. 
% - Ensure that the manual switch (SWITCH 5) between CONSTANT 2 and the
%   controller output line, is set to CONSTANT 2. 
% - Ensure that the manual switch (SWITCH 6) between CONSTANT 3 and the
%   controller output line, is set to the controller output line. 
% - Set 'input signal unit' of CONVERTER 1 to 'm'
% - Set 'input signal unit' of CONVERTER 2 to 'm'
% - Set 'input signal unit' of CONVERTER 3 to 'N*m'
% - Within the LOKOMAT MODEL block, find 'Prismatic Left Actuator' & 'Prismatic Right Actuator' block.
%   For both of these blocks: Under 'Actuation' set 'Force' to 'Automatically computed' and 'Motion' to 'Provided by Input'. 
%   In case any lines disconnect, reconnect them in the empty input of the
%   block.
% - Within the LOKOMAT MODEL block, find 'Revolute Joint Shank Bar'. Within this block: 
%   Under 'Actuation' set 'Force' to 'Provided by Input' and 'Motion' to 'Automatically computed'. 
%   In case any lines disconnect, reconnect them in the empty input of the
%   block.

% FOR STEP RESPONSE OF HIP FLEXION/ABDUCTION
% - Ensure that the manual switch (SWITCH 3) between the predetermined trajectory and
%   the step input (STEP 3) is set to STEP 3.
% - Put the 'Final value' of STEP 3 to 0.
% - Ensure that the manual switches (SWITCH 1 & SWITCH 2) between the
%   predetermined trajectory and the step input (STEP 1 & STEP 2) are set to
%   STEP 1 & STEP 2. 
% - Put the 'Final value' of STEP 1 & STEP 2 to (0.5/180)*pi.
% - Ensure that the manual switch (SWITCH 4) between CONSTANT 1 and the
%   controller output line, is set to the controller output. 
%- Ensure that the manual switch (SWITCH 5) between CONSTANT 2 and the
% controller output line, is set to the controller output. 
%- Ensure that the manual switch (SWITCH 6) between CONSTANT 3 and the
% controller output line, is set to CONSTANT 3. 
% - Set 'input signal unit' of CONVERTER 1 to 'N'
% - Set 'input signal unit' of CONVERTER 2 to 'N'
% - Set 'input signal unit' of CONVERTER 3 to 'rad'
% - Within the LOKOMAT MODEL block, find 'Prismatic Left Actuator' & 'Prismatic Right Actuator' block.
%   For both of these blocks: Under 'Actuation' set 'Force' to 'Provided by Input' and 'Motion' to 'Automatically Computed'. 
%   In case any lines disconnect, reconnect them in the empty input of the
%   block.
% - Within the LOKOMAT MODEL block, find 'Revolute Joint Shank Bar'. Within this block: 
%   Under 'Actuation' set 'Force' to 'Automatically computed' and 'Motion' to 'Provided by Input'. 
%   In case any lines disconnect, reconnect them in the empty input of the
%   block.

%% Step Response of the Knee Joint
clear;
runtime = 4; % Runtime of the Lokomat simulation (s)
kd_vec = [0.5:0.5:10]; %Pre-determined range of kd-values that will be explored
Stab_Reg_Knee = cell(1,length(kd_vec));

for i = 1:length(kd_vec)
    kd = kd_vec(i);

    % Creating empty data vectors to save the overshoot, offset and 'binary', 
    % which holds either a 1 for adequate step response and a 0 for inadequate. 
    data = [];  
    overshoot_vec = [];
    offset_vec = [];
    binary_vec = [];
    kp_vec = [];
    
    % Giving initial values to overshoot and kp value.
    overshoot = 0; 
    kp = 0;

    while overshoot<0.1 % Further simulating after overshoot exceeds 10% has no use, since it will never fall back below 10% with larger kp values
    kp = kp+20; % Kp is increased in steps of 20 
    kp_vec = [kp_vec kp]; % Saving the used kp values

    [overshoot, offset, binary] = Step_Knee(kp,kd, runtime); % Function determining overshoot and offset in step response, and gives a binary value of 1 if both are within 10%
    
    overshoot_vec = [overshoot_vec overshoot];
    offset_vec = [offset_vec offset]; 
    binary_vec = [binary_vec binary];
    end 

    data = [kp_vec', overshoot_vec', offset_vec', binary_vec'];
   Stab_Reg_Knee{i} = data; % Saving the explored kp values and the results of their step response per every kv value.
    % !! SAVE THIS AS Stab_Reg_Knee.mat FROM YOUR WORKSPACE TO CALL ON IN THE
    % FIGURE_CREATION!!
end

% After determining the total region of adequate step response tracking, we
% need to determine the maximum force achieved in this tracking. Theory
% tells that the maximum force will be achieved when the kd and kp values
% are at its highest. 

% Obtaining maximum kp and kd values explored
kp = max(Stab_Reg_Knee{length(kd_vec)}(:,1));
kd = max(kd_vec); 

% Running the simulation one more time with these values
runtime = 4;
sim('Lokomat_Simulation', runtime)

% Taking the knee actuator torque data from the simulation
Torq_Knee = ans.yout{12}.Values.Data;
fs = runtime/(length(Torq_Knee)-1);

% Determining the maximum instantaneous torque from the controller
max_torque_knee = max(abs(Torq_Knee(1/fs:end)));

%% Step response of the hip joint
clear; 
kd_hip = [0:100:2000];
runtime = 4; 
Stab_Reg_Hip = cell(1,length(kd_hip));

for i = 1:length(kd_hip)
    kd = kd_hip(i);

    % Creating empty vectors to save data
    data = [];
    binary_vec = [];
    overshoot_vec = [];
    offset_vec = [];
    kp_vec = [];

    %Giving intial values to overshoot, offset and kp
    overshoot = 0;
    overshoot_int = 0; 
    overshoot_ext = 0; 
    undershoot_int = 0;
    undershoot_ext = 0; 
    kp = 0;

    while overshoot<0.1
    kp = kp+4000;
    kp_vec = [kp_vec kp];

    [overshoot, offset, binary] = Step_Hip(kp,kd,runtime);

    binary_vec = [binary_vec binary];
    overshoot_vec = [overshoot_vec overshoot];
    offset_vec = [offset_vec offset]; 
    end 

    data = [kp_vec', overshoot_vec', offset_vec', binary_vec'];
    Stab_Reg_Hip{i} = data;
     % !! SAVE THIS AS Stab_Reg_Hip.mat FROM YOUR WORKSPACE TO CALL ON IN THE
    % FIGURE_CREATION!!
end

% After determining the total region of adequate step response tracking, we
% need to determine the maximum force achieved in this tracking. Theory
% tells that the maximum force will be achieved when the kd and kp values
% are at its highest. 

% Obtaining maximum kp and kd values explored
kp = max(Stab_Reg_Hip{length(kd_vec)}(:,1));
kd = max(kd_vec); 

% Running the simulation one more time with these values
runtime = 4;
sim('Lokomat_Simulation', runtime)

% Taking the linear hip actuator force data from the simulation
Force_Int = ans.yout{13}.Values.Data;
Force_Ext = ans.yout{14}.Values.Data;
fs = runtime/(length(Force_Int)-1);

% Determining the maximum instantaneous force from the controller
max_force_int = max(abs(Force_Int(1/fs:end)));
max_force_ext = max(abs(Force_Ext(1/fs:end))); 
