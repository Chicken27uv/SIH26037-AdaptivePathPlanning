function metrics = M7_CalculateMetrics(vehicleStates,steeringCommands,speedCommands,crossTrackErrors,targetPath,goalReached,map)

metrics.meanTrackingError = mean(abs(crossTrackErrors));
metrics.maximumTrackingError = max(abs(crossTrackErrors));

metrics.finalPositionError = norm( ...
    vehicleStates(end,1:2) - targetPath(end,:));

metrics.maximumSteering = max(abs(steeringCommands));
metrics.averageSpeed = mean(vehicleStates(:,4));

metrics.vehiclePathLength = sum( ...
    vecnorm(diff(vehicleStates(:,1:2)),2,2));

metrics.targetPathLength = sum( ...
    vecnorm(diff(targetPath),2,2));

metrics.goalReached = goalReached;

occupancy = getOccupancy( ...
    map, ...
    vehicleStates(:,1:2));

metrics.mapCollision = any(occupancy > 0.5);

metrics.collisionFree = ~metrics.mapCollision;

metrics.finalVehiclePosition = vehicleStates(end,1:2);
metrics.goalPosition = targetPath(end,:);
metrics.finalVehicleSpeed = vehicleStates(end,4);

end