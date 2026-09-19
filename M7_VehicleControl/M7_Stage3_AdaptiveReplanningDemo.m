clear
clc
close all

map = M6_CreateMap;

startPose = [5 15 0];
goalPose = [55 15 0];

baseInput.map = map;
baseInput.startPose = startPose;
baseInput.goalPose = goalPose;
baseInput.predictedObjects = struct( ...
    "position",[5 0], ...
    "velocity",[0 0]);
baseInput.decision.replanRequired = false;
baseInput.decision.behavior = "CRUISE";

baseOutput = M6_AdaptivePathPlanner(baseInput);

if ~baseOutput.success
    error("Initial M6 path generation failed.")
end

initialStates = baseOutput.initialStates;

conflictIndex = max( ...
    10, ...
    min(size(initialStates,1)-10, ...
    round(size(initialStates,1)*0.55)));

conflictPosition = initialStates(conflictIndex,1:2);

adaptiveInput.map = map;
adaptiveInput.startPose = startPose;
adaptiveInput.goalPose = goalPose;

adaptiveInput.predictedObjects = struct( ...
    "position",conflictPosition, ...
    "velocity",[0 0]);

adaptiveInput.decision.replanRequired = true;
adaptiveInput.decision.behavior = "AVOID";

m6Output = M6_AdaptivePathPlanner(adaptiveInput);

if ~m6Output.success
    error("M6 adaptive replanning failed.")
end

initialPath = m6Output.initialStates;
replannedPath = m6Output.replannedPath.States;

targetPath = replannedPath(:,1:2);
targetHeading = replannedPath(:,3);

dt = 0.05;
simulationTime = 0:dt:20;

initialState = [ ...
    targetPath(1,1), ...
    targetPath(1,2), ...
    targetHeading(1), ...
    0];

vehicleStates = zeros(numel(simulationTime),4);
steeringCommands = zeros(numel(simulationTime),1);
speedCommands = zeros(numel(simulationTime),1);
crossTrackErrors = zeros(numel(simulationTime),1);
targetIndices = zeros(numel(simulationTime),1);

vehicleStates(1,:) = initialState;

previousIndex = 1;
goalReached = false;

for k = 1:numel(simulationTime)-1

    currentState = vehicleStates(k,:);

    [steeringCommand,speedCommand,targetIndex,crossTrackError] = ...
        M7_Controller( ...
        currentState, ...
        targetPath, ...
        targetHeading, ...
        previousIndex);

    nextState = M7_VehicleModel( ...
        currentState, ...
        steeringCommand, ...
        speedCommand, ...
        dt);

    vehicleStates(k+1,:) = nextState;

    steeringCommands(k) = steeringCommand;
    speedCommands(k) = speedCommand;
    crossTrackErrors(k) = crossTrackError;
    targetIndices(k) = targetIndex;

    previousIndex = targetIndex;

    goalDistance = norm( ...
        nextState(1:2) - targetPath(end,:));

    if goalDistance < 0.8 && nextState(4) < 0.5

        goalReached = true;

        vehicleStates(k+1:end,:) = repmat( ...
            nextState, ...
            numel(simulationTime)-k, ...
            1);

        steeringCommands(k+1:end) = 0;
        speedCommands(k+1:end) = 0;
        crossTrackErrors(k+1:end) = 0;
        targetIndices(k+1:end) = size(targetPath,1);

        break

    end

end

if ~goalReached

    steeringCommands(end) = steeringCommands(end-1);
    speedCommands(end) = speedCommands(end-1);
    crossTrackErrors(end) = crossTrackErrors(end-1);
    targetIndices(end) = targetIndices(end-1);

end

trackedTargetIndices = max( ...
    1, ...
    min(targetIndices,size(targetPath,1)));

meanTrackingError = mean(abs(crossTrackErrors));
maximumTrackingError = max(abs(crossTrackErrors));

finalPositionError = norm( ...
    vehicleStates(end,1:2) - targetPath(end,:));

maximumSteering = max(abs(steeringCommands));
averageSpeed = mean(vehicleStates(:,4));

vehiclePathLength = sum( ...
    vecnorm(diff(vehicleStates(:,1:2)),2,2));

initialPathLength = sum( ...
    vecnorm(diff(initialPath(:,1:2)),2,2));

replannedPathLength = sum( ...
    vecnorm(diff(replannedPath(:,1:2)),2,2));

mapOccupancy = getOccupancy( ...
    map, ...
    vehicleStates(:,1:2));

mapCollision = any(mapOccupancy > 0.5);

conflictDistance = min( ...
    vecnorm( ...
    vehicleStates(:,1:2) - ...
    conflictPosition, ...
    2,2));

figure

tiledlayout(2,2)

nexttile

show(map)

hold on

hInitial = plot( ...
    initialPath(:,1), ...
    initialPath(:,2), ...
    "--", ...
    "LineWidth",2);

hReplanned = plot( ...
    replannedPath(:,1), ...
    replannedPath(:,2), ...
    "LineWidth",2);

hVehicle = plot( ...
    vehicleStates(:,1), ...
    vehicleStates(:,2), ...
    "LineWidth",2);

hConflict = plot( ...
    conflictPosition(1), ...
    conflictPosition(2), ...
    "o", ...
    "MarkerSize",10, ...
    "LineWidth",2);

hGoal = plot( ...
    targetPath(end,1), ...
    targetPath(end,2), ...
    "x", ...
    "MarkerSize",10, ...
    "LineWidth",2);

legend( ...
    [hInitial hReplanned hVehicle hConflict hGoal], ...
    "Initial M6 Path", ...
    "Replanned M6 Path", ...
    "M7 Vehicle", ...
    "Predicted Obstacle", ...
    "Goal", ...
    "Location","best")

title("M7 Adaptive Replanning Execution")
xlabel("X [meters]")
ylabel("Y [meters]")
axis equal

nexttile

plot( ...
    simulationTime, ...
    crossTrackErrors, ...
    "LineWidth",2)

title("M7 Cross-Track Error")
xlabel("Time [seconds]")
ylabel("Error [meters]")
grid on

nexttile

plot( ...
    simulationTime, ...
    rad2deg(steeringCommands), ...
    "LineWidth",2)

title("M7 Steering Command")
xlabel("Time [seconds]")
ylabel("Steering [degrees]")
grid on

nexttile

plot( ...
    simulationTime, ...
    vehicleStates(:,4), ...
    "LineWidth",2)

hold on

plot( ...
    simulationTime, ...
    speedCommands, ...
    "--", ...
    "LineWidth",2)

legend( ...
    "Actual Speed", ...
    "Commanded Speed", ...
    "Location","best")

title("M7 Vehicle Speed")
xlabel("Time [seconds]")
ylabel("Speed [m/s]")
grid on

sgtitle("M7 Stage 3 — Adaptive Replanning and Vehicle Execution")

fprintf("\nM7 STAGE 3 ADAPTIVE REPLANNING\n")
fprintf("-----------------------------------\n")
fprintf("Initial M6 path generated:    %d\n",baseOutput.success)
fprintf("Adaptive M6 path generated:   %d\n",m6Output.success)
fprintf("M6 replanning required:       %d\n",m6Output.replanRequired)
fprintf("M6 replans:                   %d\n",m6Output.metrics.replanCount)
fprintf("Goal reached:                 %d\n",goalReached)
fprintf("Map collision:                %d\n",mapCollision)
fprintf("Mean tracking error:          %.3f m\n",meanTrackingError)
fprintf("Maximum tracking error:       %.3f m\n",maximumTrackingError)
fprintf("Final position error:         %.3f m\n",finalPositionError)
fprintf("Maximum steering command:     %.2f deg\n",rad2deg(maximumSteering))
fprintf("Average vehicle speed:        %.3f m/s\n",averageSpeed)
fprintf("Vehicle path length:          %.3f m\n",vehiclePathLength)
fprintf("Initial M6 path length:       %.3f m\n",initialPathLength)
fprintf("Replanned M6 path length:     %.3f m\n",replannedPathLength)
fprintf("Predicted clearance:          %.3f m\n",m6Output.predictedClearance)
fprintf("Final obstacle clearance:     %.3f m\n", ...
    m6Output.metrics.replannedObstacleClearance)
fprintf("Vehicle-obstacle clearance:   %.3f m\n",conflictDistance)
fprintf("Planning time:                %.4f s\n", ...
    m6Output.metrics.initialPlanningTime)
fprintf("Replanning latency:           %.4f s\n", ...
    m6Output.metrics.replanningLatency)
fprintf("Final vehicle position:       [%.2f %.2f]\n", ...
    vehicleStates(end,1),vehicleStates(end,2))
fprintf("Goal position:                [%.2f %.2f]\n", ...
    targetPath(end,1),targetPath(end,2))