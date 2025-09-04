function [target, done] = waypointSelector(waypoints, lookahead, pos, reset)
    persistent currentWaypoint nPoints visited

    nPoints = size(waypoints,1);

    if isempty(currentWaypoint) || isempty(visited) || reset==1
        currentWaypoint = 1;
        visited = false(1,nPoints);
    end

    % current waypoint
    current_loc = waypoints(currentWaypoint,:);
    dx = current_loc(1) - pos(1);
    dy = current_loc(2) - pos(2);

    % next waypoint
    if sqrt(dx^2 + dy^2) < lookahead
        visited(currentWaypoint) = true;
        if currentWaypoint < nPoints
            currentWaypoint = currentWaypoint + 1;
        end
    end

    % Outputs
    target = waypoints(currentWaypoint,:);
    done = all(visited);
end
