function [estPose, particles] = MCL(particles,numParticles,map1,validScan,move,sampleTime,lidar)

    % prediction
    particles = particles + move * sampleTime + 0.08 * randn(3, numParticles);

    % beroz resani vazn ba lidar
    for p = 1:numParticles
        [ranges_p, angles_p] = lidar(particles(:,p)', map1);
        scan_p = lidarScan(ranges_p, angles_p);
        validScan_p = removeInvalidData(scan_p, 'RangeLimits', [0, 10]);
        weights(p) = Weighting(validScan_p, validScan);
    end

    % normalize/resample
    weights = weights / sum(weights);
    indices = resampleParticles(weights);
    particles = particles(:,indices);
    weights = ones(1, numParticles) / numParticles;
    estPose = particles * weights';
end
%%
function indices = resampleParticles(weights)
    edges = [0, cumsum(weights)];
    edges(end) = 1;
    numParticles = numel(weights);
    indices = zeros(1, numParticles);
    for i = 1:numParticles
        rnd = rand();
        indices(i) = find(edges >= rnd, 1) - 1;
    end
end
function likelihood = Weighting(scan1, scan2)
    if isempty(scan1.Ranges) || isempty(scan2.Ranges)
        likelihood = 0;
        return;
    end
    minLen = min(length(scan1.Ranges), length(scan2.Ranges));
    diff = abs(scan1.Ranges(1:minLen) - scan2.Ranges(1:minLen));
    likelihood = exp(-sum(diff.^2));
end