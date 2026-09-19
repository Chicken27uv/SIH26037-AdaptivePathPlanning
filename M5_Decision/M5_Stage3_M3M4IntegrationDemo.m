clc
clear
close all

ego.position = [0 0];
ego.velocity = [5 0];
ego.heading = 0;

m3Output.tracks(1).class = "vehicle";
m3Output.tracks(1).position = [14 0];
m3Output.tracks(1).velocity = [1 0];

m3Output.tracks(2).class = "pedestrian";
m3Output.tracks(2).position = [8 -2];
m3Output.tracks(2).velocity = [0 1];

m3Output.tracks(3).class = "cattle";
m3Output.tracks(3).position = [6 5];
m3Output.tracks(3).velocity = [0 -0.5];

m4Output.predictions(1).trackIndex = 1;
m4Output.predictions(1).predictedTrajectory = ...
    [14 0;12 0;10 0;8 0;6 0];

m4Output.predictions(2).trackIndex = 2;
m4Output.predictions(2).predictedTrajectory = ...
    [8 -2;8 -1;8 0;8 1;8 2];

m4Output.predictions(3).trackIndex = 3;
m4Output.predictions(3).predictedTrajectory = ...
    [6 5;6 4.5;6 4;6 3.5;6 3];

input = M5_CreateM3M4Input(ego,m3Output,m4Output);

output = M5_DecisionLogic(input);

disp("M5 STAGE 3 M3 + M4 INTEGRATION")
disp("--------------------------------")
fprintf("M3 tracks received:          %d\n", ...
    output.metrics.numberOfObjects)

fprintf("Conflicting objects:         %d\n", ...
    output.metrics.numberOfConflicts)

fprintf("Selected target:             %s\n", ...
    output.targetObject)

fprintf("Behavior:                    %s\n", ...
    output.behavior)

fprintf("Risk level:                  %s\n", ...
    output.riskLevel)

fprintf("Replan required:             %d\n", ...
    output.replanRequired)

fprintf("Target speed:                %.2f m/s\n", ...
    output.targetSpeed)

fprintf("Decision latency:            %.6f s\n", ...
    output.metrics.decisionLatency)

disp(" ")
disp("M3 -> M4 -> M5 handoff completed successfully.")

figure
tiledlayout(1,3)

nexttile
hold on
grid on
axis equal

plot(ego.position(1),ego.position(2), ...
    "ko","MarkerFaceColor","k","MarkerSize",8)

for i = 1:numel(m3Output.tracks)

    plot( ...
        m3Output.tracks(i).position(1), ...
        m3Output.tracks(i).position(2), ...
        "rx","LineWidth",2,"MarkerSize",10)

end

xlim([-2 18])
ylim([-7 7])
xlabel("X (m)")
ylabel("Y (m)")
title("M3 Tracked Objects")

nexttile
hold on
grid on
axis equal

plot(ego.position(1),ego.position(2), ...
    "ko","MarkerFaceColor","k","MarkerSize",8)

for i = 1:numel(m4Output.predictions)

    trajectory = m4Output.predictions(i).predictedTrajectory;

    plot( ...
        trajectory(:,1), ...
        trajectory(:,2), ...
        "r--","LineWidth",1.5)

    plot( ...
        trajectory(1,1), ...
        trajectory(1,2), ...
        "rx","LineWidth",2,"MarkerSize",10)

end

xlim([-2 18])
ylim([-7 7])
xlabel("X (m)")
ylabel("Y (m)")
title("M4 Predicted Trajectories")

nexttile
hold on
grid on
axis equal

plot(ego.position(1),ego.position(2), ...
    "ko","MarkerFaceColor","k","MarkerSize",8)

for i = 1:numel(input.objects)

    object = input.objects(i);

    plot( ...
        object.position(1), ...
        object.position(2), ...
        "rx","LineWidth",2,"MarkerSize",10)

    trajectory = object.predictedTrajectory;

    plot( ...
        trajectory(:,1), ...
        trajectory(:,2), ...
        "r--","LineWidth",1.5)

end

xlim([-2 18])
ylim([-7 7])
xlabel("X (m)")
ylabel("Y (m)")
title(sprintf("M5 Decision: %s",output.behavior))

sgtitle("M3 Perception + M4 Prediction + M5 Decision")

function input = M5_CreateM3M4Input(ego,m3Output,m4Output)

input.ego = ego;

numberOfTracks = numel(m3Output.tracks);

input.objects = repmat(struct( ...
    "class","", ...
    "position",[0 0], ...
    "velocity",[0 0], ...
    "predictedTrajectory",[]),1,numberOfTracks);

for i = 1:numberOfTracks

    input.objects(i).class = ...
        string(m3Output.tracks(i).class);

    input.objects(i).position = ...
        m3Output.tracks(i).position;

    input.objects(i).velocity = ...
        m3Output.tracks(i).velocity;

    predictionIndex = find( ...
        [m4Output.predictions.trackIndex] == i,1);

    if isempty(predictionIndex)

        input.objects(i).predictedTrajectory = ...
            m3Output.tracks(i).position;

    else

        input.objects(i).predictedTrajectory = ...
            m4Output.predictions(predictionIndex).predictedTrajectory;

    end

end

end