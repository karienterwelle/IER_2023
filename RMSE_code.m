%% Introduction + Important settings
% This file contains the calculations of the Root Mean Square Error for
% the trajectory tracking of the three joint angles with different
% PD-parameters. In the Lokomat Simulation, a couple things need to be
% adjusted to ensure adequate simulation. Read these carefully! 

% FOR RMSE OF KNEE FLEXION: 
% - Ensure that the manual switch (SWITCH 3) between the predetermined trajectory and
%   the step input (STEP 3) is set to the predetermined trajectory.
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
%   For both of these block: Under 'Actuation' set 'Force' to 'Automatically computed' and 'Motion' to 'Provided by Input'. 
%   In case any lines disconnect, reconnect them in the empty input of the
%   block.
% - Within the LOKOMAT MODEL block, find 'Revolute Joint Shank Bar'. Within this block: 
%   Under 'Actuation' set 'Force' to 'Provided by Input' and 'Motion' to 'Automatically computed'. 
%   In case any lines disconnect, reconnect them in the empty input of the
%   block.

% FOR RMSE OF HIP FLEXION/ABDUCTION
% - Ensure that the manual switch (SWITCH 3) between the predetermined trajectory and
%   the step input (STEP 3) is set to STEP 3.
% - Put the 'Final value' of STEP 3 to 0.
% - Ensure that the manual switches (SWITCH 1 & SWITCH 2) between the
%   predetermined trajectory and the step input (STEP 1 & STEP 2) are set to
%   the predetermined trajectory. 
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
%   For both of these block: Under 'Actuation' set 'Force' to 'Provided by Input' and 'Motion' to 'Automatically Computed'. 
%   In case any lines disconnect, reconnect them in the empty input of the
%   block.
% - Within the LOKOMAT MODEL block, find 'Revolute Joint Shank Bar'. Within this block: 
%   Under 'Actuation' set 'Force' to 'Automatically computed' and 'Motion' to 'Provided by Input'. 
%   In case any lines disconnect, reconnect them in the empty input of the
%   block.

%% Determining RMSE for every combination of kd and kp for Knee flexion

clear;
kp = 0;
kd_vec = [0.5:0.5:10];   % The used kd range
D = load('Stab_Reg_Knee.mat'); % Loading the data containing the tested kp-values per kd value, and their resulting step response data. 
RMSE = cell(1,length(kd_vec)); % Creating an empty data cell 

for i = 1:length(kd_vec)
    kd = kd_vec(i); % Analyse per kd-value
    kp_vec = D.Stab_Reg_Knee{1, i}(:,1); % Take the vector containing all tested kp-values from the data file
    binary_vec = D.Stab_Reg_Knee{1, i}(:,4); % Take the vector containing a binary indicating whether the step response was adequate for the corresponding kp values.

   % The following section extracts the kp-values that resulted in an adequate step response
   % and calculates the RMSE in trajectory tracking for those kp-values.
   % It returnes the used kp_values (kp_vec_use) and their corresponding trajectory
   % tracking errors (RMSE_vec).
    ind = find(binary_vec == 1); % Find the indices where adequate step response was achieved
    if isempty(ind)
        kp_vec_use = NaN; % If no adequate step response was achieved for any kp-value, return 2 NaN vectors
        RMSE_vec = NaN; % If no adequate step response was achieved for any kp-value, return 2 NaN vectors
    else
        kp_vec_use = kp_vec(ind(1):ind(end)); % If there were adequate step responses achieved, take the corresponding kp values
        RMSE_vec = [];

        for j = 1:length(kp_vec_use)
    kp = kp_vec_use(j); % Take a kp value from the previously determined kp values with adequate step response
    runtime = 5; 
    sim('Lokomat_Simulation.slx', runtime); % Running the Lokomat simulation for 5 seconds
    Knee_des = ans.yout{2}.Values.Data; % The desired knee flexion angles over time in radians
    Knee_act = ans.yout{3}.Values.Data; % The actual knee flexion angles over time in radians
    RMSE_vec = [RMSE_vec rmse(Knee_des,Knee_act)]; % Add the new RMSE value between desired and actual path to the RMSE vector
        end
    end

RMSE{i} = [kp_vec_use, RMSE_vec']; % Save the data in a cell per kd-value 
% !! SAVE THIS AS RMSE_Knee.mat FROM YOUR WORKSPACE TO USE IN THE FIGURE_CREATION !!
end

% After determining the total region of adequate step response tracking, we
% need to determine the maximum force achieved in this tracking. Theory
% tells that the maximum force will be achieved when the kd and kp values
% are at its highest. 

% Obtaining maximum kp and kd values explored
kp = max(RMSE{length(kd_vec)}(:,1));
kd = max(kd_vec); 

% Running the simulation one more time with these values
runtime = 4;
sim('Lokomat_Simulation', runtime)

% Taking the knee actuator torque data from the simulation
Torq_Knee = ans.yout{12}.Values.Data;
fs = runtime/(length(Torq_Knee)-1);

% Determining the maximum instantaneous torque from the controller
max_torque_knee = max(abs(Torq_Knee(1/fs:end))); % To prevent high torques by initial offsets being found as the step-response torque, we look from 1 second.

%% Determining tracking error for hip abduction and flexion 

clear;
kp = 0;
kd_hip = [0:100:2000];  % The predetermined used kd-range
D = load('Stab_Reg_Hip.mat'); % Loading the data containing the tested kp-values per kd value, and their resulting step response data. 
RMSE = cell(1,length(kd_hip)); % Creating an empty data cell 

for i = 1:length(kd_hip)
    kd = kd_hip(i);  % Analyse per kd-value
    kp_vec = D.Stab_Reg_Hip{1, i}(:,1); % Take the vector containing all tested kp-values from the data file
    binary_vec = D.Stab_Reg_Hip{1, i}(:,4); % Take the vector containing a binary indicating whether the step response was adequate for the corresponding kp values.
    
   % The following section extracts the kp-values that resulted in an adequate step response
   % and calculates the RMSE in trajectory tracking for those kp-values.
   % It returnes the used kp_values (kp_vec_use) and their corresponding trajectory
   % tracking errors (RMSE_vec).
     ind = find(binary_vec == 1); % Find the indices where adequate step response was achieved
    
    if isempty(ind)
        kp_vec_use = NaN; % If no adequate step response was achieved for any kp-value, return 3 NaN vectors
        RMSE_abd = NaN; % If no adequate step response was achieved for any kp-value, return 3 NaN vectors
        RMSE_flex = NaN; % If no adequate step response was achieved for any kp-value, return 3 NaN vectors
    else
        kp_vec_use = kp_vec(ind(1):ind(end)); % If there were adequate step responses achieved, take the corresponding kp values
        RMSE_abd = [];
        RMSE_flex = [];
        for j = 1:length(kp_vec_use)
        kp = kp_vec_use(j);  % Take a kp value from the previously determined kp values with adequate step response
        runtime = 5; 
        sim('Lokomat_Simulation.slx', runtime); % Running the Lokomat simulatino for 5 seconds
        Abd_Des = ans.yout{8}.Values.Data; % The desired hip abduction angles over time in radians
        Abd_Act = ans.yout{9}.Values.Data; % The actual hip abduction angles over time in radians
        Flex_Des = ans.yout{10}.Values.Data; % The desired hip flexion angles over time in radians
        Flex_Act = ans.yout{11}.Values.Data; % The actual hip flexion angles over time in radians
        time = ans.yout{7}.Values.Time; % The time-stamps of the simulations
        fs = 1/(time(2)-time(1)); % The sampling frequency of the simulation
        start_ind = 0.7*fs; % To prevent initial offsets between start position of hip and desired start position to cause false RMSE, we start at 0.7 seconds
        RMSE_abd = [RMSE_abd rmse(Abd_Des(start_ind:end),Abd_Act(start_ind:end))]; % Add the new RMSE between desired and actual path to the RMSE vector
        RMSE_flex = [RMSE_flex rmse(Flex_Des(start_ind:end), Flex_Act(start_ind:end))]; % Add the new RMSE between desired and actual path to the RMSE vector
        end
    end

RMSE{i} = [kp_vec_use, RMSE_abd', RMSE_flex']; % Save the data in a cell per kd-value
% !! SAVE THIS AS RMSE_Hip.mat FROM YOUR WORKSPACE TO USE IN THE FIGURE_CREATION !!
end

% After determining the total region of adequate step response tracking, we
% need to determine the maximum force achieved in this tracking. Theory
% tells that the maximum force will be achieved when the kd and kp values
% are at its highest. 

% Obtaining maximum kp and kd values explored
kp = max(RMSE{length(kd_hip)}(:,1));
kd = max(kd_hip); 

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