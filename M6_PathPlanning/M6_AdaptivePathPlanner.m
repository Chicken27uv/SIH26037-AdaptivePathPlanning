function output = M6_AdaptivePathPlanner(input)

cfg = projectConfig;

map = input.map;
startPose = input.startPose;
goalPose = input.goalPose;

if isfield(input,"decision")
    decision = input.decision;
else
    decision.replanRequired = false;
    decision.behavior = "CRUISE";
end

[pathInitial,infoInitial,initialPlanningTime] = ...
    M6_PlanPath(map,startPose,goalPose);

if ~infoInitial.IsPathFound
    output.success = false;
    output.initialPath = [];
    output.path = [];
    output.replannedPath = [];
    output.replanRequired = false;
    output.triggerPoint = [];
    output.conflictPoint = [];
    output.metrics = [];
    return
end

initialStates = pathInitial.States;

objects = input.predictedObjects;

bestClearance = inf;
bestConflictPoint = [];
bestTriggerPoint = [];
bestTriggerIndex = [];
bestPredictedTrajectory = [];

for i = 1:numel(objects)

    [predictedTrajectory,minClearance,conflictPoint,replanRequired,triggerIndex,triggerPoint] = ...
        M6_PredictConflict( ...
        initialStates, ...
        objects(i), ...
        cfg.planning.predictionHorizon, ...
        cfg.planning.collisionBuffer);

    if minClearance < bestClearance
        bestClearance = minClearance;
        bestConflictPoint = conflictPoint;
        bestTriggerPoint = triggerPoint;
        bestTriggerIndex = triggerIndex;
        bestPredictedTrajectory = predictedTrajectory;
    end

end

if isempty(bestConflictPoint)
    bestClearance = inf;
    bestTriggerIndex = 1;
end

replanRequired = decision.replanRequired || ...
    bestClearance < cfg.planning.collisionBuffer;

replanningLatency = 0;
replanCount = 0;
replannedPath = pathInitial;
replannedStates = initialStates;

if replanRequired

    obstacleCenter = bestConflictPoint;
    obstacleHalfSize = [2 2];

    [xo,yo] = meshgrid( ...
        obstacleCenter(1)-obstacleHalfSize(1):0.25:obstacleCenter(1)+obstacleHalfSize(1), ...
        obstacleCenter(2)-obstacleHalfSize(2):0.25:obstacleCenter(2)+obstacleHalfSize(2));

    setOccupancy(map,[xo(:) yo(:)],1);

    safeTriggerIndex = max(1,bestTriggerIndex - 12);

    while safeTriggerIndex > 1

        candidatePose = initialStates(safeTriggerIndex,:);

        occupancyValue = getOccupancy(map,candidatePose(1:2));

        if occupancyValue < 0.5
            break
        end

        safeTriggerIndex = safeTriggerIndex - 1;

    end

    replanningStartPose = initialStates(safeTriggerIndex,:);

    [replannedPath,infoReplanned,replanningLatency,replanCount] = ...
        M6_Replan( ...
        map, ...
        replanningStartPose, ...
        goalPose, ...
        true);

    if infoReplanned.IsPathFound
        replannedStates = replannedPath.States;
        selectedPath = replannedPath;
    else
        infoReplanned = infoInitial;
        selectedPath = pathInitial;
        replanningLatency = 0;
        replanCount = 0;
        replannedStates = initialStates;
    end

    actualTriggerPoint = replanningStartPose(1:2);

else

    infoReplanned = infoInitial;
    selectedPath = pathInitial;
    actualTriggerPoint = bestTriggerPoint;

end

if ~isempty(bestConflictPoint)

    obstacleCenter = bestConflictPoint;
    obstacleHalfSize = [2 2];

    replannedObstacleClearance = ...
        M6_CalculateClearance( ...
        replannedStates, ...
        obstacleCenter, ...
        obstacleHalfSize);

else

    replannedObstacleClearance = inf;

end

metrics = M6_CalculateMetrics( ...
    initialStates, ...
    replannedStates, ...
    initialPlanningTime, ...
    replanningLatency, ...
    bestClearance, ...
    replanCount, ...
    map, ...
    bestConflictPoint, ...
    [2 2]);

metrics.replannedObstacleClearance = replannedObstacleClearance;

output.success = infoReplanned.IsPathFound;
output.initialPath = pathInitial;
output.replannedPath = replannedPath;
output.path = selectedPath;
output.initialStates = initialStates;
output.selectedStates = selectedPath.States;
output.replanRequired = replanRequired;
output.triggerPoint = actualTriggerPoint;
output.conflictPoint = bestConflictPoint;
output.predictedTrajectory = bestPredictedTrajectory;
output.predictedClearance = bestClearance;
output.decision = decision;
output.metrics = metrics;

end