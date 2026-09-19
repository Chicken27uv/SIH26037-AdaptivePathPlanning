clear
clc
close all

[targetPath,targetHeading] = M7_CreateTargetPath;

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

trackedTargetIndices = max(1,min(targetIndices,size(targetPath,1)));

positionErrors = vecnorm( ...
    vehicleStates(:,1:2) - ...
    targetPath(trackedTargetIndices,:), ...
    2,2);

meanTrackingError = mean(abs(crossTrackErrors));
maximumTrackingError = max(abs(crossTrackErrors));

finalPositionError = norm( ...
    vehicleStates(end,1:2) - targetPath(end,:));

maximumSteering = max(abs(steeringCommands));
averageSpeed = mean(vehicleStates(:,4));

figure

tiledlayout(2,2)

nexttile

plot( ...
    targetPath(:,1), ...
    targetPath(:,2), ...
    "--", ...
    "LineWidth",2)

hold on

plot( ...
    vehicleStates(:,1), ...
    vehicleStates(:,2), ...
    "LineWidth",2)

plot( ...
    targetPath(1,1), ...
    targetPath(1,2), ...
    "o", ...
    "MarkerSize",8, ...
    "LineWidth",2)

plot( ...
    targetPath(end,1), ...
    targetPath(end,2), ...
    "x", ...
    "MarkerSize",10, ...
    "LineWidth",2)

legend( ...
    "Target Path", ...
    "Actual Vehicle", ...
    "Start", ...
    "Goal", ...
    "Location","best")

title("M7 Vehicle Path Tracking")
xlabel("X [meters]")
ylabel("Y [meters]")
axis equal
grid on

nexttile

plot( ...
    simulationTime, ...
    crossTrackErrors, ...
    "LineWidth",2)

title("Cross-Track Error")
xlabel("Time [seconds]")
ylabel("Error [meters]")
grid on

nexttile

plot( ...
    simulationTime, ...
    rad2deg(steeringCommands), ...
    "LineWidth",2)

title("Steering Command")
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

title("Vehicle Speed")
xlabel("Time [seconds]")
ylabel("Speed [m/s]")
grid on

sgtitle("M7 Stage 1 Vehicle Motion and Control")

fprintf("\nM7 STAGE 1 VEHICLE MOTION AND CONTROL\n")
fprintf("---------------------------------------\n")
fprintf("Goal reached:                 %d\n",goalReached)
fprintf("Mean tracking error:          %.3f m\n",meanTrackingError)
fprintf("Maximum tracking error:       %.3f m\n",maximumTrackingError)
fprintf("Final position error:         %.3f m\n",finalPositionError)
fprintf("Maximum steering command:     %.2f deg\n",rad2deg(maximumSteering))
fprintf("Average vehicle speed:        %.3f m/s\n",averageSpeed)
fprintf("Final vehicle position:       [%.2f %.2f]\n", ...
    vehicleStates(end,1),vehicleStates(end,2))
fprintf("Target goal position:         [%.2f %.2f]\n", ...
    targetPath(end,1),targetPath(end,2))