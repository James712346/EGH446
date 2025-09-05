start_pos = [0 0 pi/4];
dt = 0.5;

% Guidance
look_ahead_distance = 10;
K_theta = 2;
waypoint_locations = randi([-500 500],10,2);

% ordering waypoints
nPoints = size(waypoint_locations,1);

% waypoint closest to the start
distances = sqrt(sum(waypoint_locations.^2,2));
[~, start] = min(distances);

% Generate all permutations of other waypoints
other = setdiff(1:nPoints, start);
allroutes = perms(other);

allroutes = [repmat(start, size(allroutes,1),1) allroutes];

% Calculate the distance of each route
totaldist = zeros(size(allroutes,1),1);
for k = 1:size(allroutes,1)
    r = allroutes(k,:);
    distsum = 0;
    for i = 1:(nPoints-1)
        distsum = distsum + norm(waypoint_locations(r(i),:) - waypoint_locations(r(i+1),:));
    end
    totaldist(k) = distsum;
end

% determine the shortest route
[~, best] = min(totaldist);
optimalroute = allroutes(best,:);

% order waypoints based on travel order 
orderedWaypoints = waypoint_locations(optimalroute,:);

staticWaypoints = [0 0; 4 4;];
