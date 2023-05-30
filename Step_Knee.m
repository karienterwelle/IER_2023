% This function determines the overshoot and offset of PD-controlled step response for given kp and kd value, 
% and returns in 'binary' wether it is adequate.

function [overshoot, offset, binary] = Step_Knee(kp,kd,runtime) 
    kp = kp;
    kd = kd; 
    sim('Lokomat_Simulation.slx', runtime); % Running the simulation for the given kp and kd values

    Des_Knee = ans.yout{2}.Values.Data; % Obtaining the desired knee angle over time (step input)
    Act_Knee = ans.yout{3}.Values.Data; % Obtaining the actual knee angle over time (step response)
    time = ans.yout{3}.Values.Time;  % Obtaining the time values of the simulation

    fs = runtime/(length(time)-1); % Determining the sampling frequency of the simulation
    ind = []; % Creating an empty index vector
    ind = find(Act_Knee(1/fs:end)<=-(1/180)*pi+0.0025 & Act_Knee(1/fs:end)>=-(1/180)*pi-0.0025); % Determing the index where the actual knee angle crosses the step input angle (if this happens). 
   
    % If no crossing of the desired step input angle takes place, there is
    % no overshoot.
    if isempty(ind)
    overshoot = 0;

    % If crossing of the desired step input angle does take place,
    % overshoot is the max. difference between actual and desired angle,
    % normalized with the desired angle.
    else 
        overshoot = (abs(min(Act_Knee))-(1/180)*pi)/((1/180)*pi);
    end 

    settle_val = abs(Act_Knee(end)); % Final reached value of the knee angle
    offset = ((1/180)*pi-settle_val)/((1/180)*pi); % Difference between the desired and actual final value of the knee angle step response
    
    if overshoot<0.1 && abs(offset)<0.1 % If both overshoot and offset are below 10%, the step response is adequate.
    binary = 1;
    else 
      binary = 0;
    end

end
