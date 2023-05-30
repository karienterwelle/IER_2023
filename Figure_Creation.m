%% Creating the PD-parameters with adequate step response plot for the hip joint

Data_Knee = load('Stab_Reg_Knee.mat'); % Loading the file containing all tested kp and kd values, and their step response results
kd_vec = [0.5:0.5:10]; % The range of kd values that was tested
max_kp = max(Data_Knee.Stab_Reg_Knee{1,length(kd_vec)}(:,1)); % Determining the maximum tested kp-value

figure; hold on;
for i = 1:length(kd_vec) % Analyse per tested kd
    ind_stable = find(Data_Knee.Stab_Reg_Knee{1,i}(:,4)==1); % Find the kp values that resulted in an adequate step response
    kd_stable = kd_vec(i)*ones(length(ind_stable),1); % Vector with as length the number of kp values with adequate step response and with a constant kd-value
    kp_stable = Data_Knee.Stab_Reg_Knee{1,i}(ind_stable,1); % Vector with the kp values resulting in an adequate step response

    ind_unstable = find(Data_Knee.Stab_Reg_Knee{1,i}(:,4)==0); % Find the kp values that resulted in an inadequate step response
    kv_unstable = kd_vec(i)*ones(length(ind_unstable),1); % Vector with as length the number of kp values with inadequate step response and with a constant kd-value
    kp_unstable = Data_Knee.Stab_Reg_Knee{1,i}(ind_unstable,1); % Vector with the kp values resulting in an inadequate step response

    kp_remainder = [max(kp_unstable):20:max_kp]; % Vector with the untested kp-values for this specific kd value, up until the overall maximum tested kp-value. To get a square graph.
    kv_remainder = kd_vec(i)*ones(length(kp_remainder));  % Vector with as length the number of untested kp-valuse and with a constant kd-value

    plot(kd_stable,kp_stable, 'k*'); plot(kv_unstable, kp_unstable, 'ro'); plot(kv_remainder,kp_remainder,'ro'); % Both the 'remainder' and 'unstable' parts are inadequate step responses. The 'stable' part signifies an adequate step response.
end

% The legend of this figure was created manually in Powerpoint
title('Region of PD-parameter values that result in tracking of the step response within 10%');
ylabel('kp values (N*m/rad)')
xlabel('kd values (N*m/(rad/s))')

%% Creating the PD-parameters with adequate step response plot for the hip joint

clear; 
Data_Hip = load('Stab_Reg_Hip.mat');  % Loading the file containing all tested kp and kd values, and their step response results
kd_vec = [0:100:2000]; % The range of kd values that was tested
max_kp = max(Data_Hip.Stab_Reg_Hip{1,length(kd_vec)}(:,1)); % Determining the maximum tested kp-value

figure; hold on;
for i = 1:length(kd_vec) % Analyse per tested kd
    ind_stable = find(Data_Hip.Stab_Reg_Hip{1,i}(:,4)==1); % Find the kp values that resulted in an adequate step response
    kd_stable = kd_vec(i)*ones(length(ind_stable),1); % Vector with as length the number of kp values with adequate step response and with a constant kd-value
    kp_stable = Data_Hip.Stab_Reg_Hip{1,i}(ind_stable,1); % Vector with the kp values resulting in an adequate step response

    ind_unstable = find(Data_Hip.Stab_Reg_Hip{1,i}(:,4)==0); % Find the kp values that resulted in an inadequate step response
    kv_unstable = kd_vec(i)*ones(length(ind_unstable),1);  % Vector with as length the number of kp values with inadequate step response and with a constant kd-value
    kp_unstable = Data_Hip.Stab_Reg_Hip{1,i}(ind_unstable,1); % Vector with the kp values resulting in an inadequate step response

    kp_remainder = [max(kp_unstable):4000:max_kp]; % Vector with the untested kp-values for this specific kd value, up until the overall maximum tested kp-value. To get a square graph.
    kv_remainder = kd_vec(i)*ones(length(kp_remainder));  % Vector with as length the number of untested kp-valuse and with a constant kd-value

    plot(kd_stable,kp_stable, 'k*'); plot(kv_unstable, kp_unstable, 'ro'); plot(kv_remainder,kp_remainder,'ro'); % Both the 'remainder' and 'unstable' parts are inadequate step responses. The 'stable' part signifies an adequate step response.
end

% The legend of this figure was created manually in Powerpoint
title('Region of PD-parameter values that result in tracking of the step response within 10%');
ylabel('kp values (N/m)')
xlabel('kd values (N/(m/s))')


%% Creating 3D plot for RMSE Knee Flexion 

clear;
RMSE_Knee = load('RMSE_Knee.mat'); % Loading the file containing all tested kp and kd values, and their trajectory tracking RMSE
kd_vec = [0.5:0.5:10]; % The range of kd values that was tested
max_kp = max(RMSE_Knee.RMSE{1,length(kd_vec)}(:,1));  % Determining the maximum tested kp-value
min_kp = 0; % The minimum tested kp-value

RMSE_mat = [];
kp_vec_tot = min_kp:20:max_kp; % A vector containing all tested kp-values

for i = 1:length(kd_vec)  % Analyse per tested kd
    kp_vec = RMSE_Knee.RMSE{1,i}(:,1); % Obtain the tested kp-values for this specific kd-value
    RMSE_vec = RMSE_Knee.RMSE{1,i}(:,2); % Obtain the corresponding RMSE in trajectory tracking
    if isnan(kp_vec) % If no kp-values were tested (since none resulted in adequate step response), we create an RMSE zeros vector
        RMSE_vec = zeros((max_kp/20)+1,1); % Length is such that every kp-value from kp_vec_tot is matched to a 0 RMSE
    else
        kp_below = min(kp_vec); % Minimum kp-value tested for this specific kd-value
        kp_above = max(kp_vec); % Maximum kp-value tested for this specific kd-value
        diff_below = kp_below-min_kp; % Difference between the total minimum kp-value (0) and the current minimum tested kp-value for this specific kd-value
        diff_above = max_kp-kp_above; % Difference between the total maximum kp-value and the current maximum tested kp-value for this specific kd-value
        length_below = (diff_below/20); % The number of steps in kp between the total and current minimum
        length_above = diff_above/20; % The number of steps in kp between the total and current maximum

        % Filling the RMSE_vec with zeros at the untested kp-values above and below the min. and max. tested value. 
        % In between is the RMSE for the actually tested kp-values.
        RMSE_vec = [zeros(length_below,1); RMSE_vec; zeros(length_above,1)]; 
    end
    RMSE_mat = [RMSE_mat RMSE_vec]; %Creating a matrix of all the RMSE_vectors per kd-value
end

RMSE_mat = RMSE_mat*180/pi; % Converting radians to degrees for the figure
RMSE_mat(RMSE_mat==0) = NaN; % Setting all zero values (that were put at untested kp-values in the previous loop) to NaN, so they will not be plotted.
figure; hold on;
surf(kd_vec',kp_vec_tot',RMSE_mat); grid on; % Creating a 3D plot with kp and kd values on the x- and y-axis, and the RMSE as a colour gradient.
a = colorbar; % Displaying color bar
a.Label.String = 'RMSE (deg)'; % Labelling color bar
ylabel('kp values (N*m/rad)')
xlabel('kd values (N*m/(rad/s))')
title('Root Mean Square Error (RMSE) for trajectory tracking of knee flexion angle for different PD-parameters')


%% Creating 3D plot for RMSE Hip abduction

clear;
RMSE_Hip = load('RMSE_Hip.mat'); % Loading the file containing all tested kp and kd values, and their trajectory tracking RMSE
kd_vec = [0:100:2000]; % The range of kd values that was tested
max_kp = max(RMSE_Hip.RMSE{1,length(kd_vec)}(:,1));  % Determining the maximum tested kp-value
min_kp = 0; % The minimum tested kp-value

RMSE_mat = [];
kp_vec_tot = min_kp:4000:max_kp; % A vector containing all tested kp-values

for i = 1:length(kd_vec)
    kp_vec = RMSE_Hip.RMSE{1,i}(:,1);  % Obtain the tested kp-values for this specific kd-value
    RMSE_vec = RMSE_Hip.RMSE{1,i}(:,2); % Obtain the corresponding RMSE in trajectory tracking for hip abduction
    if isnan(kp_vec) % If no kp-values were tested (since none resulted in adequate step response), we create an RMSE zeros vector
        RMSE_vec = zeros((max_kp/4000)+1,1); % Length is such that every kp-value from kp_vec_tot is matched to a 0 RMSE
    else
        kp_below = min(kp_vec); % Minimum kp-value tested for this specific kd-value
        kp_above = max(kp_vec); % Maximum kp-value tested for this specific kd-value
        diff_below = kp_below-min_kp; % Difference between the total minimum kp-value (0) and the current minimum tested kp-value for this specific kd-value
        diff_above = max_kp-kp_above; % Difference between the total maximum kp-value and the current maximum tested kp-value for this specific kd-value
        length_below = (diff_below/4000);  % The number of steps in kp between the total and current minimum
        length_above = diff_above/4000;  % The number of steps in kp between the total and current maximum

        % Filling the RMSE_vec with zeros at the untested kp-values above and below the min. and max. tested value. 
        % In between is the RMSE for the actually tested kp-values.
        RMSE_vec = [zeros(length_below,1); RMSE_vec; zeros(length_above,1)];

    end
    RMSE_mat = [RMSE_mat RMSE_vec]; %Creating a matrix of all the RMSE_vectors per kd-value
end

RMSE_mat = RMSE_mat*180/pi;  % Converting radians to degrees for the figure
RMSE_mat(RMSE_mat==0) = NaN; % Setting all zero values (that were put at untested kp-values in the previous loop) to NaN, so they will not be plotted.
figure; hold on;
surf(kd_vec',kp_vec_tot',RMSE_mat); grid on;  % Creating a 3D plot with kp and kd values on the x- and y-axis, and the RMSE as a colour gradient.
a = colorbar; % Displaying color bar
a.Label.String = 'RMSE (deg)'; % Labelling color bar
xlabel('kd values (N/(m/s))')
ylabel('kp values (N/m)')
title('Root Mean Square Error (RMSE) for trajectory tracking of hip ab-/adduction for different PD-parameters')

%% Creating 3D plot for RMSE Hip Flexion

clear;
RMSE_Hip = load('RMSE_hip.mat'); % Loading the file containing all tested kp and kd values, and their trajectory tracking RMSE
kd_vec = [0:100:2000]; % The range of kd values that was tested
max_kp = max(RMSE_Hip.RMSE{1,length(kd_vec)}(:,1));  % Determining the maximum tested kp-value
min_kp = 0; % The minimum tested kp-value

RMSE_mat = [];
kp_vec_tot = min_kp:4000:max_kp; % A vector containing all tested kp-values

for i = 1:length(kd_vec)
    kp_vec = RMSE_Hip.RMSE{1,i}(:,1);  % Obtain the tested kp-values for this specific kd-value
    RMSE_vec = RMSE_Hip.RMSE{1,i}(:,3); % Obtain the corresponding RMSE in trajectory tracking for hip flexion
    if isnan(kp_vec) % If no kp-values were tested (since none resulted in adequate step response), we create an RMSE zeros vector
        RMSE_vec = zeros((max_kp/4000)+1,1); % Length is such that every kp-value from kp_vec_tot is matched to a 0 RMSE
    else
        kp_below = min(kp_vec); % Minimum kp-value tested for this specific kd-value
        kp_above = max(kp_vec); % Maximum kp-value tested for this specific kd-value
        diff_below = kp_below-min_kp; % Difference between the total minimum kp-value (0) and the current minimum tested kp-value for this specific kd-value
        diff_above = max_kp-kp_above; % Difference between the total maximum kp-value and the current maximum tested kp-value for this specific kd-value
        length_below = (diff_below/4000);  % The number of steps in kp between the total and current minimum
        length_above = diff_above/4000;  % The number of steps in kp between the total and current maximum

        % Filling the RMSE_vec with zeros at the untested kp-values above and below the min. and max. tested value. 
        % In between is the RMSE for the actually tested kp-values.
        RMSE_vec = [zeros(length_below,1); RMSE_vec; zeros(length_above,1)];

    end
    RMSE_mat = [RMSE_mat RMSE_vec]; %Creating a matrix of all the RMSE_vectors per kd-value
end

RMSE_mat = RMSE_mat*180/pi;  % Converting radians to degrees for the figure
RMSE_mat(RMSE_mat==0) = NaN; % Setting all zero values (that were put at untested kp-values in the previous loop) to NaN, so they will not be plotted.
figure; hold on;
surf(kd_vec',kp_vec_tot',RMSE_mat); grid on;  % Creating a 3D plot with kp and kd values on the x- and y-axis, and the RMSE as a colour gradient.
a = colorbar; % Displaying color bar
a.Label.String = 'RMSE (deg)'; % Labelling color bar
xlabel('kd values (N/(m/s))')
ylabel('kp values (N/m)')
title('Root Mean Square Error (RMSE) for trajectory tracking of hip flexion for different PD-parameters')





