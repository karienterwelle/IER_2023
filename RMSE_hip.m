function [kp_vec_use,RMSE_abd, RMSE_flex] = RMSE_hip(kp_vec, binary_vec) % Calculates Root Mean Square Error (RMSE) in trajectory tracking for given kd-value and kp-value vector
    
    
    ind = find(binary_vec == 1); % Find the indices where adequate step response was achieved
    figure; 
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
