function [known_corner, weight] = slam_crnr_kf(known_corner, detected_corner)
% Updates means, covariance for map features and update particle weight 
	
	% Calculate Kalman filter gain
	Qt = detected_corner.covariance; % measurement cov matrix
	Q = known_corner.covariance + Qt; % innovation cov matrix(H=I)
	K = known_corner.covariance * Q^-1; % Karman gain
    
	% Calculate innovation 
    x_innov = detected_corner.x - known_corner.x;
    y_innov = detected_corner.y - known_corner.y;
    heading_innov = slam_in_pi(detected_corner.heading - known_corner.heading);
    angle_innov = slam_in_pi(detected_corner.angle - known_corner.angle);
    innovation = [x_innov; y_innov; heading_innov; angle_innov];
    
    % Missing codes start here ...
    
    % Update mean of this corner 
    current_state = [known_corner.x; known_corner.y; known_corner.heading; known_corner.angle];
    updated_state = current_state + K * innovation;
    known_corner.x = updated_state(1); known_corner.y = updated_state(2);
    known_corner.heading = updated_state(3);
    known_corner.angle = updated_state(4);
    
	% Update covariance of this corner  
    known_corner.covariance = known_corner.covariance - K * Q * K';
    
    % Missing codes end here ...
    
    % Calculate importance weight introduced by this corner  
    weight = (det(2*pi*Q)^-0.5) * exp(-0.5*(innovation)'*(Q^-1)*(innovation));
    
	% Increment view count
	known_corner.view_count = known_corner.view_count + 1;
    
end

