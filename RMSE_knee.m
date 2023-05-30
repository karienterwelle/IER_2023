function [kp_vec_use, RMSE_vec] = RMSE_knee(kp_vec, binary_vec) % Calculates Root Mean Square Error (RMSE) in trajectory tracking for given kd-value and kp-value vector

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