close all; clear all;

load Lidar_input;

threshold = 18000;

covariance = [0.01    0     0;  % x
                 0 0.01     0;  % y
                 0    0 0.015]; % angle
             
translation_pre = [0, 0];
theta_pre = 0;

%% objective function
global obj_func_y  obj_func_x;

function f = obj_func(T)
    global obj_func_y  obj_func_x;
    dx = T(1); dy = T(2); theta = T(3);
    transpose_mat = [cos(theta), -sin(theta), dx; sin(theta), cos(theta), dy; 0, 0, 1];
    error_mat = obj_func_y - transpose_mat * obj_func_x;
    f = trace(error_mat' * error_mat);
end
             
%% LiDAR Input Parameters
field_of_view = deg2rad(270);
readings_count = 1081;

for i = 1 : 1081
    bearing_mat(i) = (pi-field_of_view)/2+(i-1)*field_of_view/(readings_count-1);
end
cos_bearing_mat = cos(bearing_mat);
sin_bearing_mat = sin(bearing_mat);

% Limit lidar data range
output_temp = output;
for i = 1:size(output_temp,1)
	for j = 1:1081
		if (output_temp(i,j) > 35000)
			output_temp(i,j) = 0;
		end
	end
end

data_axes = subplot(1, 2, 1);
motion_axes = subplot(1, 2, 2);

motion_estimate = struct('x', 0, 'y', 0, 'theta', 0, 'point_count', 0);

points = slam_points(output_temp(1,:), bearing_mat, cos_bearing_mat, sin_bearing_mat);
[lines1, corners1] = slam_lidar_feat_extrn(points);

for i = 1:size(output_temp,1)-1
	points = slam_points(output_temp(i+1,:), bearing_mat, cos_bearing_mat, sin_bearing_mat);
	[lines2, corners2] = slam_lidar_feat_extrn(points);
	
	P = [];
	Q = [];
    
    assoc_count = 0;
	
	%% Associate
	if (~isempty(corners2))
		if (~isempty(corners1))
			known_corners_count = size(corners1, 2);
			detected_corners_count = size(corners2, 2);
			% Calculate Mahalanobic distance matrix
			mahalanobis_matrix = threshold.*ones(detected_corners_count, known_corners_count);
			for j = 1:detected_corners_count
				for k = 1:known_corners_count
					% Components of x-mu
					a = corners2(j).x-corners1(k).x;
					b = corners2(j).y-corners1(k).y;
					c = slam_in_pi(corners2(j).angle-corners1(k).angle);

					mahalanobis_dist = sqrt([a b c] * (covariance^-1) * [a; b; c]);
					if (mahalanobis_dist < threshold)
						mahalanobis_matrix(j,k) = mahalanobis_dist;
					end
				end
            end
            
			while 1
				mahalanobis_min = min(min(mahalanobis_matrix));
				if (mahalanobis_min < threshold)
					% Associate, add to matrix of points
					[d_index, k_index] = find(mahalanobis_matrix == mahalanobis_min);
					
					if (size(d_index, 1) > 1)
						d_index = d_index(1);
					end
					if (size(k_index, 1) > 1)
						k_index = k_index(1);
					end
					
					assoc_count = assoc_count + 1;
					P(assoc_count,:) = [corners1(k_index).x corners1(k_index).y];
					Q(assoc_count,:) = [corners2(d_index).x corners2(d_index).y];
                    
					% Eliminate row and column from further consideration
					mahalanobis_matrix(d_index,:) = (threshold+1).*ones(1, known_corners_count);
					mahalanobis_matrix(:,k_index) = (threshold+1).*ones(detected_corners_count, 1);
				else
					break;
				end
			end
		end
    end
	
	%% Motion Estimation
	if (assoc_count >= 2)
        % Missing codes start here ...
        
   		% Use matched corners to calculate rotation and translation between
   		% the two frames 
        % method one, by SVD

        avg_P = mean(P, 1)'; avg_Q = mean(Q, 1)';
        new_P = P' - avg_P; new_Q = Q' - avg_Q;
        [U, S, V] = svd(new_P * new_Q');
        d = sign(det(V * U'));
        R = V * [1, 0; 0, d] * U';
        T = R * avg_P - avg_Q;

        theta = -atan2(R(2), R(1));
        translation = [T(1), T(2)];

        % method two, by non-linear optimization

        % T_0 = [0.0, 0.0, 0.0];
        % obj_func_y = [P'; ones(1, size(P, 1))]; obj_func_x = [Q'; ones(1, size(Q, 1))];
        % options = optimoptions('fminunc', 'Algorithm', 'quasi-newton', 'Display', 'iter');
        % [T_opt, fval] = fminunc(@obj_func, T_0, options);
        % theta = T_opt(3); translation = T_opt(1:2);


        % Missing codes end here ...
		
        % Constrain x change to be within a threshold
        if (translation(1) - translation_pre(1) > 10)
            translation(1) = translation_pre(1) + 10;
        elseif (translation(1) - translation_pre(1) < -10)
            translation(1) = translation_pre(1) - 10;
        end
        
        % Constrain y change to be within a threshold
        if (translation(2) - translation_pre(2) > 10)
            translation(2) = translation_pre(2) + 10;
        elseif (translation(2) - translation_pre(2) < -10)
            translation(2) = translation_pre(2) - 10;
        end
        
        % Constrain theta change to be within a threshold
        if (slam_in_pi(theta - theta_pre) > 0.05)
            theta = theta_pre + 0.05;
        elseif (slam_in_pi(theta - theta_pre) < -0.05)
            theta = theta_pre - 0.05;
        end
        theta = slam_in_pi(theta);
        
        motion_estimate(i+1).x = translation(1);
        motion_estimate(i+1).y = translation(2);
        motion_estimate(i+1).theta = theta;
		motion_estimate(i+1).point_count = assoc_count;
        
	else
		% Assume constant velocity
		motion_estimate(i+1).x = translation_pre(1);
		motion_estimate(i+1).y = translation_pre(2);
		motion_estimate(i+1).theta = theta_pre;
		motion_estimate(i+1).point_count = assoc_count;
    end
	
	%% Plotting stuff
	cla(data_axes);
	cla(motion_axes);
	hold on;
	
	subplot(data_axes);
	points_x = output(i,:) .* cos_bearing_mat;
	points_y = output(i,:) .* sin_bearing_mat;
	plot(points_x, points_y, 'r.');
	axis equal;
	axis tight;
	
	subplot(motion_axes);
	low = -30000;
	high = 30000;
	plot([low high high low], [low low high high], 'k');
	
	if (assoc_count > 0)
		plot(P(:,1), P(:,2), 'r.');
		plot(Q(:,1), Q(:,2), 'b.');
    end

	text(0, 0, num2str(rad2deg(theta)));
	text(-27000, -27000, [num2str(translation(1)) ', ' num2str(translation(2))]);
	axis equal;
	axis tight;
	
	hold off;
	drawnow;
	
	%% Prep for next frame
	lines1 = lines2;
	corners1 = corners2;
    translation_pre = translation;
    theta_pre = theta;
end

%% Result saving
save motion_estimate;

%% Result plotting
path = zeros(4, size(motion_estimate, 2));

for i = 1:size(motion_estimate, 2)
    path(1,i) = motion_estimate(i).x;
    path(2,i) = motion_estimate(i).y;
    path(3,i) = motion_estimate(i).theta;
    path(4,i) = motion_estimate(i).point_count;
end

figure;
for i = 1:4
   subplot(4, 1, i);
   plot(path(i, :), 'b*'); 
end