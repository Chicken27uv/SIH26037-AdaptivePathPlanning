function metrics = M8_CalculateMetrics(result)

egoTrajectory = result.egoTrajectory;

pathLength = sum(vecnorm(diff(egoTrajectory),2,2));

dx = diff(egoTrajectory(:,1));
dy = diff(egoTrajectory(:,2));

heading = unwrap(atan2(dy,dx));

if numel(heading) > 2
    headingChange = diff(heading);
    pathSmoothness = sum(headingChange.^2);
else
    pathSmoothness = 0;
end

metrics.scenario = result.name;
metrics.completed = result.completed;
metrics.collisionFree = result.collisionFree;
metrics.minimumClearance = result.minimumClearance;
metrics.replanningLatency = result.replanningLatency;
metrics.pathSmoothness = pathSmoothness;
metrics.replanCount = result.replanCount;
metrics.pathLength = pathLength;
metrics.goalDistance = result.goalDistance;
metrics.minimumClearanceTime = result.minimumClearanceTime;

end