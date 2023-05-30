% This function determines the overshoot and offset of PD-controlled step response for given kp and kd value, 
% and returns in 'binary' wether it is adequate.

function  [overshoot, offset, binary] = Step_Hip(kp,kd,runtime)
    kp = kp;
    kd = kd; 
    sim('Lokomat_Simulation.slx', runtime); 

    INT_Des = ans.yout{4}.Values.Data; % Obtaining the desired internal actuator position over time (step input)
    EXT_Des = ans.yout{5}.Values.Data; % Obtaining the desired external actuator position over time (step input)
    INT_Act = ans.yout{6}.Values.Data; % Obtaining the actual internal actuator position over time (step response)
    EXT_Act = ans.yout{7}.Values.Data; % Obtaining the actual external actuator position over time (step response)
    time = ans.yout{4}.Values.Time; % Timepoints of the simulation

    % The values of the linear actuators is derived from the step input on
    % the hip angles. Therefore, the step on the linear actuators does not
    % go from 0 to a self-written value, but follows from the inverse
    % kinematics in the Simulink. It needs to be determined what the
    % initial value is and what the value of the step input is.

    start_step_INT = min(INT_Des);  % Initial position internal actuator
    end_step_INT = max(INT_Des); % Step input value of the internal actuator
    start_step_EXT = min(EXT_Des); % Initial position external actuator
    end_step_EXT = max(EXT_Des); % Step input value of the external actuator
    step_size_INT = end_step_INT-start_step_INT; % Step size internal actuator
    step_size_EXT = end_step_EXT-start_step_EXT; % Step size external actuator

    fs = runtime/(length(time)-1); % Sampling frequency of the simulation

    ind = [];  % Creating an empty index vector
    ind_int = find(INT_Act(2/fs:end)>=end_step_INT); % Determining if there are indices where the actual internal actuator position crosses the step input.
    ind_ext = find(EXT_Act(1/fs:end)>=end_step_EXT); % Determining if there are indices where the actual internal actuator position crosses the step input.
  
    % If the step input is not crossed by either the internal or external actuator position, the overshoot is 0.
    if isempty(ind_int) && isempty(ind_ext) 
    overshoot = 0;

    % If the step input is only crossed by the internal actuator position,
    % overshoot is the maximum normalized difference between the actual and desired
    % actuator position of the INTERNAL actuator.
    elseif isempty(ind_ext)
        overshoot = (max(INT_Act)-end_step_INT)/step_size_INT;


    % If the step input is only crossed by the external actuator position,
    % overshoot is the maximum normalized difference between the actual and desired
    % actuator position of the EXTERNAL actuator.
    elseif isempty(ind_int)
        overshoot = (max(EXT_Act)-end_step_EXT)/step_size_EXT;

    % If the step input is only crossed by both actuator positions,
    % overshoot is the maximum normalized difference between the actual and desired
    % actuator position of EITHER actuator.
    else
        overshoot_int = (max(INT_Act(2/fs:end))-end_step_INT)/step_size_INT;
        overshoot_ext = (max(EXT_Act(2/fs:end))-end_step_EXT)/step_size_EXT;
        overshoot = max([overshoot_int overshoot_ext]);
    end 

    settle_val_int = abs(INT_Act(length(INT_Act))); % Final value of the internal actuator position
    settle_val_ext = abs(EXT_Act(length(EXT_Act))); % Final value of the external actuator position
    offset_int = abs(end_step_INT-settle_val_int)/(step_size_INT);  % Difference between the desired and actual final value of the internal actuator step response
    offset_ext = abs(end_step_EXT-settle_val_ext)/(step_size_EXT);   % Difference between the desired and actual final value of the external actuator step response
    offset = max([offset_int offset_ext]); % Maximum offset is the maximum offset of either actuator

    if overshoot<0.1 && abs(offset)<0.1 % If both overshoot and offset are below 10%, the step response is adequate.
    binary = 1;
    else 
      binary = 0;
    end
end

