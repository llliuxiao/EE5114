function [particles] = slam_resample(particles, init_weight)
	
	particles_count = size(particles, 2);
    new_particles = particles(1);

    weights = zeros(particles_count, 1); total_weights = 0;
    for i = 1:particles_count
        weights(i) = particles(i).weight;
        total_weights = total_weights + weights(i);
    end
    cumulative_weights = cumsum(weights) / total_weights;

	for i = 1:particles_count
        % Missing codes start here
        
        % Resamples particles based on their weights
        random_num = rand(1);
        index = find(cumulative_weights >= random_num, 1);
        new_particles(i) = particles(index);
        
        % Afterwards, each new partical should be given the same init_weight
        new_particles(i).weight = init_weight;
        
        % Missing codes end here
    end
    particles = new_particles;
end