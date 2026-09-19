function metrics = M8_CalculateM6Metrics(output,input)

selectedStates = output.selectedStates;

pathLength = sum(vecnorm(diff(selectedStates(:,1:2)),2,2));

heading = unwrap(selectedStates(:,3));

if numel(heading) > 2
    headingChange = diff(heading);
    pathSmoothness = sum(headingChange.^2);
else
    pathSmoothness = 0;
end

goalDistance = norm( ...
    selectedStates(end,1:2) - input.goalPose(1:2));

completed = goalDistance < 1.0;

metrics.scenario = input.predictedObjects(1).class;
metrics.success = output.success;
metrics.completed = completed;
metrics.collisionFree = output.metrics.collisionFree;
metrics.minimumClearance = output.metrics.replannedObstacleClearance;
metrics.predictedClearance = output.predictedClearance;
metrics.planningTime = output.metrics.initialPlanningTime;
metrics.replanningLatency = output.metrics.replanningLatency;
metrics.pathLength = pathLength;
metrics.pathSmoothness = pathSmoothness;
metrics.replanCount = output.metrics.replanCount;
metrics.goalDistance = goalDistance;
metrics.replanRequired = output.replanRequired;

end