%% Clear past plots, variables and console prints
close all; clear all; clc;

% Load data 
load 'EE5114_CA1.mat';

% acx = x-axis accelerometer reading
% acy = y-axis accelerometer reading
% acz = z-axis accelerometer reading
% 
% phi = Roll angle computed by the drone's on-board computer
% tht = Pitch angle computed by the drone's on-board computer
% psi = Yaw angle computed by the drone's on-board computer 
% 
% fix = GPS position fix signal 
% eph = GPS horizontal variance 
% epv = GPS vertical variance 
% lat = GPS Latitude
% lon = GPS Longitude
% alt = GPS altitude
% gps_nSat = Number of GPS satellites
% 
% out1 = Motor 1 signal
% out2 = Motor 2 signal
% out3 = Motor 3 signal
% out4 = Motor 4 signal

%% Accelerometer plot
figure; set(gcf,'numbertitle','off','name','Acceleration');  
subplot(3,1,1); plot(t, acx, 'b'); ylim([-2 2]); ylabel('acx (m/s^2)'); grid on; 
subplot(3,1,2); plot(t, acy, 'b'); ylim([-2 2]); ylabel('acy (m/s^2)'); grid on; 
subplot(3,1,3); plot(t, acz, 'b'); ylabel('acz (m/s^2)'); xlabel('time (s)'); grid on; 

%% Euler angles plot
figure; set(gcf,'numbertitle','off','name','Euler Angles');  
subplot(3,1,1); plot(t, rad2deg(phi), 'b'); ylabel('Roll (degree)'); grid on; 
subplot(3,1,2); plot(t, rad2deg(tht), 'b'); ylabel('Pitch (degree)'); grid on; 
subplot(3,1,3); plot(t, rad2deg(psi), 'b'); ylabel('Yaw (degree)'); xlabel('time (s)'); grid on; 

%% GPS plot
figure; set(gcf,'numbertitle','off','name','GPS');  

subplot(3,2,1); plot(t, lon); ylabel('Longitude'); grid on;
subplot(3,2,3); plot(t, lat); ylabel('Latitude'); grid on;
subplot(3,2,5); plot(t, alt); ylabel('Altitude'); grid on; xlabel('time (s)');

subplot(3,2,2); plot(t, gps_nSat, '.'); ylabel('Sat'); grid on;
subplot(3,2,4); plot(t, eph); ylabel('Eph'); grid on; ylim([0 5]);
subplot(3,2,6); plot(t, epv); ylabel('Epv'); grid on; ylim([0 5]); xlabel('time (s)');

%% Motor signal plot
figure; set(gcf,'numbertitle','off','name','Motor Signal');  
hold on;
plot(t,out1,'r');
plot(t,out2,'g');
plot(t,out3,'b');
plot(t,out4,'y');
legend('Motor1','Motor2','Motor3','Motor4'); 
ylabel('Motor inputs'); xlabel('time (s)'); ylim([1000 2000]); grid on;


%%%%%%%%%%%%%%%%%%%%%% Your own coding work start from here %%%%%%%%%%%%%%%%%%%%%%%%%

%% Convert GPS raw measurements to local NED position values
a = 6378137.0; 
f = 1/298.257223563;
b = (1 - f) * a;
t_min = 840; t_max = 1996;
t_min_index = 4099; t_max_index = 9804;
lat_rad = deg2rad(lat); lon_rad = deg2rad(lon);

% transfrom from GPS to ecef
N_fi = a ^ 2 ./ sqrt((a * cos(lat_rad)).^2 + (b * sin(lat_rad)).^2);
x_ecef = (N_fi + alt) .* cos(lat_rad) .* cos(lon_rad);
y_ecef = (N_fi + alt) .* cos(lat_rad) .* sin(lon_rad);
z_ecef = ((b / a) ^ 2 * N_fi + alt) .* sin(lat_rad);

% set the origin of ned frame
x_ecef_o = x_ecef(t_min_index); 
y_ecef_o = y_ecef(t_min_index); 
z_ecef_o = z_ecef(t_min_index);
lat_o = lat_rad(t_min_index);
lon_o = lon_rad(t_min_index);

rotation_mat = [
    -sin(lat_o) * cos(lon_o) -sin(lon_o) -cos(lat_o) * cos(lon_o);
    -sin(lat_o) * sin(lon_o) +cos(lon_o) -cos(lat_o) * sin(lon_o);
    +cos(lat_o)              +0          -sin(lat_o)
]';

% transform from ecef to ned
ned = rotation_mat * ([x_ecef'; y_ecef'; z_ecef'] - [x_ecef_o; y_ecef_o; z_ecef_o]);
x_ned = ned(1, :); y_ned = ned(2, :); z_ned = ned(3, :);

disp("My NED position done!");

%% test with matlab transform function
% t_min_index = 4099; t_max_index = 9804;
% [x_ned, y_ned, z_ned] = geodetic2ned(lat, lon, alt, lat(t_min_index), lon(t_min_index), alt(t_min_index), wgs84Ellipsoid);
% disp("Matlab NED position done!");

%% plot the ned position
figure; set(gcf, 'numbertitle','off','name','Ned position');

subplot(3,1,1); plot(t(t_min_index:t_max_index), x_ned(t_min_index:t_max_index)); ylabel('ned-x'); grid on;
subplot(3,1,2); plot(t(t_min_index:t_max_index), y_ned(t_min_index:t_max_index)); ylabel('ned-y'); grid on;
subplot(3,1,3); plot(t(t_min_index:t_max_index), z_ned(t_min_index:t_max_index)); ylabel('ned-z'); grid on; xlabel('time (s)');

%% Implement EKF to estimate NED position and velocity

function [ekf_output, gps_vel] = KarmanFilter(acc_noise, global_vars, drift_flag)
    % initialize global variable
    t = global_vars("t"); t_max_index = global_vars("t_max_index"); t_min_index = global_vars("t_min_index");
    x_ned = global_vars("x_ned"); y_ned = global_vars("y_ned"); z_ned = global_vars("z_ned"); 
    phi = global_vars("phi"); tht = global_vars("tht"); psi = global_vars("psi"); 
    acx = global_vars("acx"); acy = global_vars("acy"); acz = global_vars("acz");

    % initialize EKF matrix
    gps_vel = [t(t_min_index), 0, 0, 0];
    eph_mean = 1; epv_mean = 1; dt_mean = 1.0;
    var_x = eph_mean / 2; var_y = eph_mean / 2; var_z = epv_mean;

    % state variable, cov matrix and measurement matrix initialization
    if drift_flag
        ekf_output = zeros(t_max_index - t_min_index + 1, 10);
        ekf_output(1, end) = -1.0;
        p_k = diag([var_x, var_y, var_z, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]); 
        H = eye(6, 10);
    else
        ekf_output = zeros(t_max_index - t_min_index + 1, 9);
        p_k = diag([var_x, var_y, var_z, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0]); 
        H = eye(6, 9);
    end
    x_k = ekf_output(1, :)';

    % cov mat of measurement noise
    var_xyz = diag([var_x, var_y, var_z]);
    var_v = diag([2 * var_x / dt_mean ^ 2, 2 * var_y / dt_mean ^ 2, 2 * var_z / dt_mean ^ 2]);
    cov_xyz_v = diag([var_x / dt_mean, var_y / dt_mean, var_z / dt_mean]);
    R = [var_xyz cov_xyz_v; cov_xyz_v var_v];

    % check update condition 
    waiting_time = 0.0;
    last_update_ned = [x_ned(t_min_index); y_ned(t_min_index); z_ned(t_min_index)];

    for k = t_min_index + 1 : t_max_index
        dt = t(k) - t(k - 1);
        u_k = [acx(k) acy(k) acz(k) 1]';
        R_ned_body = rotz(psi(k)) * roty(tht(k)) * rotx(phi(k));

        % calculate F and G
        trans_f = [eye(3, 3)     dt .* eye(3)   -dt ^ 2 / 2 .* eye(3);
                zeros(3, 3)   eye(3)         -dt .* eye(3)];
        rot_f   = [eye(6)        zeros(6, 3); 
                zeros(3, 6)   R_ned_body];
        rot_g = [R_ned_body [0 0 9.8]'];

        if drift_flag
            F = [trans_f * rot_f, zeros(6, 1);
                zeros(3, 6), eye(3), dt * ones(3, 1);
                zeros(1, 9), 1];
            trans_g = [dt ^ 2 / 2 * eye(3); dt * eye(3); zeros(4, 3)];
        else
            trans_g = [dt ^ 2 / 2 * eye(3); dt * eye(3); zeros(3, 3)];
            F = [trans_f * rot_f;
                zeros(3, 6), eye(3)];
        end

        G = trans_g * rot_g;
        jacobin_f_x = F;

        % cov mat of observation noise
        Q = G * acc_noise * G';

        % prediction step
        x_k = F * x_k + G * u_k;
        p_k = jacobin_f_x * p_k * jacobin_f_x' + Q;
        ekf_output(k - t_min_index + 1, :) = x_k;

        % check new gps signal availablity
        ned_k = [x_ned(k), y_ned(k), z_ned(k)]';
        ned_k_1 = [x_ned(k - 1), y_ned(k - 1), z_ned(k - 1)]';
    
        waiting_time = waiting_time + dt;
        if isequal(ned_k, ned_k_1)
            new_gps_flag = waiting_time >= 1.0;
        else
            new_gps_flag = true;
        end

        if new_gps_flag
            % correction step
            S = H * p_k * H' + R;
            W = p_k * H' / S;
            y_k = H * x_k;
            delta_y = [ned_k - y_k(1:3) ; (ned_k - last_update_ned) ./ waiting_time - y_k(4:end)];
            gps_vel = [gps_vel; t(k), ((ned_k - last_update_ned) ./ waiting_time)'];

            % update step
            x_k = x_k + W * delta_y ;
            p_k = p_k - W * S * W';

            waiting_time = 0.0;
            last_update_ned = ned_k;
        end
        fprintf("k = %d done! correction step: %s\n", k - t_min_index + 1, mat2str(new_gps_flag));
    end
    disp("Karman Filter Done!");
end

%% Call Karman Filter function
global_vars = containers.Map();
global_vars("t") = t; global_vars("t_min_index") = t_min_index; global_vars("t_max_index") = t_max_index;
global_vars("x_ned") = x_ned; global_vars("y_ned") = y_ned; global_vars("z_ned") = z_ned;
global_vars("phi") = phi; global_vars("tht") = tht; global_vars("psi") = psi; 
global_vars("acx") = acx; global_vars("acy") = acy; global_vars("acz") = acz; 

[ekf_output_small_obs_noise, ~] = KarmanFilter(diag([0.1, 0.1, 0.1, 0]), global_vars, false);
[ekf_output_medium_obs_noise, ~] = KarmanFilter(diag([1, 1, 1, 0]), global_vars, false);
[ekf_output_large_obs_noise, gps_vel] = KarmanFilter(diag([5, 5, 5, 0]), global_vars, false);

%% Call Karman Filter with drift bias
[ekf_output_small_obs_noise_drift, ~] = KarmanFilter(diag([0.1, 0.1, 0.1, 0]), global_vars, true);

%% Result plots - Position
figure; set(gcf, 'numbertitle','off','name','EKF-Position');

subplot(3,1,1); hold on;
plot(t(t_min_index:t_max_index), x_ned(t_min_index:t_max_index), "r"); 
plot(t(t_min_index:t_max_index), ekf_output_small_obs_noise(:, 1), "g"); 
plot(t(t_min_index:t_max_index), ekf_output_medium_obs_noise(:, 1), "b");
plot(t(t_min_index:t_max_index), ekf_output_large_obs_noise(:, 1), "y"); 
legend("gps", "small acc noise", "medium acc noise", "large acc noise");
ylabel('x'); grid on;

subplot(3,1,2); hold on;
plot(t(t_min_index:t_max_index), y_ned(t_min_index:t_max_index), "r"); 
plot(t(t_min_index:t_max_index), ekf_output_small_obs_noise(:, 2), "g"); 
plot(t(t_min_index:t_max_index), ekf_output_medium_obs_noise(:, 2), "b");
plot(t(t_min_index:t_max_index), ekf_output_large_obs_noise(:, 2), "y"); 
legend("gps", "small acc noise", "medium acc noise", "large acc noise");
ylabel('y'); grid on;

subplot(3,1,3); hold on;
plot(t(t_min_index:t_max_index), z_ned(t_min_index:t_max_index), "r"); 
plot(t(t_min_index:t_max_index), ekf_output_small_obs_noise(:, 3), "g"); 
plot(t(t_min_index:t_max_index), ekf_output_medium_obs_noise(:, 3), "b");
plot(t(t_min_index:t_max_index), ekf_output_large_obs_noise(:, 3), "y"); 
legend("gps", "small acc noise", "medium acc noise", "large acc noise");
ylabel('z'); grid on; xlabel('time (s)');

%% Result plots - Velocity
figure; set(gcf, 'numbertitle','off','name','EKF-Velocity');
hold on;

subplot(3,1,1); hold on;
plot(gps_vel(:, 1), gps_vel(:, 2), "r");
plot(t(t_min_index:t_max_index), ekf_output_small_obs_noise(:, 4), "g"); 
plot(t(t_min_index:t_max_index), ekf_output_medium_obs_noise(:, 4), "b");
plot(t(t_min_index:t_max_index), ekf_output_large_obs_noise(:, 4), "y"); 
legend("gps", "small acc noise", "medium acc noise", "large acc noise");
ylabel('vx'); grid on;

subplot(3,1,2); hold on;
plot(gps_vel(:, 1), gps_vel(:, 3), "r");
plot(t(t_min_index:t_max_index), ekf_output_small_obs_noise(:, 5), "g"); 
plot(t(t_min_index:t_max_index), ekf_output_medium_obs_noise(:, 5), "b");
plot(t(t_min_index:t_max_index), ekf_output_large_obs_noise(:, 5), "y"); 
legend("gps", "small acc noise", "medium acc noise", "large acc noise");
ylabel('vy'); grid on;

subplot(3,1,3); hold on;
plot(gps_vel(:, 1), gps_vel(:, 4), "r");
plot(t(t_min_index:t_max_index), ekf_output_small_obs_noise(:, 6), "g"); 
plot(t(t_min_index:t_max_index), ekf_output_medium_obs_noise(:, 6), "b");
plot(t(t_min_index:t_max_index), ekf_output_large_obs_noise(:, 6), "y"); 
legend("gps", "small acc noise", "medium acc noise", "large acc noise");
ylabel('vz'); grid on; xlabel('time (s)');

%% Result plots - Acceleration Bias
figure; set(gcf, 'numbertitle','off','name','Constant-Acc-Bias'); hold on;

subplot(3,1,1); hold on;
plot(t(t_min_index:t_max_index), ekf_output_small_obs_noise(:, 7), "r"); 
plot(t(t_min_index:t_max_index), ekf_output_medium_obs_noise(:, 7), "g");
plot(t(t_min_index:t_max_index), ekf_output_large_obs_noise(:, 7), "b"); 
legend("small acc noise", "medium acc noise", "large acc noise");
ylabel('bias-x'); grid on;

subplot(3,1,2); hold on;
plot(t(t_min_index:t_max_index), ekf_output_small_obs_noise(:, 8), "r"); 
plot(t(t_min_index:t_max_index), ekf_output_medium_obs_noise(:, 8), "g");
plot(t(t_min_index:t_max_index), ekf_output_large_obs_noise(:, 8), "b"); 
legend("small acc noise", "medium acc noise", "large acc noise");
ylabel('bias-y'); grid on;

subplot(3,1,3); hold on;
plot(t(t_min_index:t_max_index), ekf_output_small_obs_noise(:, 9), "r"); 
plot(t(t_min_index:t_max_index), ekf_output_medium_obs_noise(:, 9), "g");
plot(t(t_min_index:t_max_index), ekf_output_large_obs_noise(:, 9), "b"); 
legend("small acc noise", "medium acc noise", "large acc noise");
ylabel('bias-z'); grid on; xlabel('time (s)');

%% Result plots - Acceleration Drift Bias
figure; set(gcf, 'numbertitle','off','name','Drift-Acc-Bias'); hold on;

subplot(3,1,1); hold on;
plot(t(t_min_index:t_max_index), ekf_output_small_obs_noise_drift(:, 7), "r"); 
plot(t(t_min_index:t_max_index), ekf_output_small_obs_noise(:, 7), "g");
legend("drift-bias", "constant-bias");
ylabel('bias-x'); grid on;

subplot(3,1,2); hold on;
plot(t(t_min_index:t_max_index), ekf_output_small_obs_noise_drift(:, 8), "r"); 
plot(t(t_min_index:t_max_index), ekf_output_small_obs_noise(:, 8), "g");
legend("drift-bias", "constant-bias");
ylabel('bias-y'); grid on;

subplot(3,1,3); hold on;
plot(t(t_min_index:t_max_index), ekf_output_small_obs_noise_drift(:, 9), "r"); 
plot(t(t_min_index:t_max_index), ekf_output_small_obs_noise(:, 9), "g");
legend("drift-bias", "constant-bias");
ylabel('bias-z'); grid on; xlabel('time (s)');

%% drift rate
figure; set(gcf, 'numbertitle','off','name','Drift-Rate'); hold on;
plot(t(t_min_index:t_max_index), ekf_output_small_obs_noise_drift(:, 10), "r"); 
ylabel('drift-rate'); grid on; xlabel('time (s)');