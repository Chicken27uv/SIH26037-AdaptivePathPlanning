clc
clear
close all

ego.position = [5 15];
ego.velocity = [5 0];
ego.heading = 0;

m3Output.tracks(1).class = "vehicle";
m3Output.tracks(1).position = [24 15];
m3Output.tracks(1).velocity = [-1 0];

m3Output.tracks(2).class = "pedestrian";
m3Output.tracks(2).position = [18 8];
m3Output.tracks(2).velocity = [0 0.5];

m3Output.tracks(3).class = "cattle";
m3Output.tracks(3).position = [35 25];
m3Output.tracks(3).velocity = [0 -0.5];

m4Output.predictions(1).trackIndex = 1;
m4Output.predictions(1).predictedTrajectory = ...
    [24 15;23.5 15;23 15;22.5 15;22 15];

m4Output.predictions(2).trackIndex = 2;
m4Output.predictions(2).predictedTrajectory = ...
    [18 8;18 8.25;18 8.5;18 8.75;18 9];

m4Output.predictions(3).trackIndex = 3;
m4Output.predictions(3).predictedTrajectory = ...
    [35 25;35 24.75;35 24.5;35 24.25;35 24];

m5Input = M5_CreateM3M4Input(ego,m3Output,m4Output);
m5Output = M5_DecisionLogic(m5Input);

map = M6_CreateMap;

startPose = [5 15 0];
goalPose = [55 15 0];

m6Input.map = map;
m6Input.startPose = startPose;
m6Input.goalPose = goalPose;
m6Input.predictedObjects = m5Input.objects;
m6Input.decision.replanRequired = m5Output.replanRequired;
m6Input.decision.behavior = m5Output.behavior;

m6Output = M6_AdaptivePathPlanner(m6Input);

disp("M5 STAGE 4 M5 -> M6 HANDOFF")
disp("--------------------------------")
fprintf("M5 behavior:                 %s\n",m5Output.behavior)
fprintf("M5 risk level:               %s\n",m5Output.riskLevel)
fprintf("M5 target object:            %s\n",m5Output.targetObject)
fprintf("M5 replan required:          %d\n",m5Output.replanRequired)
fprintf("M5 target speed:             %.2f m/s\n",m5Output.targetSpeed)
fprintf("M5 decision latency:         %.6f s\n",m5Output.metrics.decisionLatency)
fprintf("M6 path generated:           %d\n",m6Output.success)
fprintf("M6 replanning required:      %d\n",m6Output.replanRequired)
fprintf("M6 replans:                  %d\n",m6Output.metrics.replanCount)
fprintf("M6 collision free:           %d\n",m6Output.metrics.collisionFree)
fprintf("M6 predicted clearance:      %.3f m\n",m6Output.metrics.minimumPredictedClearance)
fprintf("M6 final clearance:          %.3f m\n",m6Output.metrics.replannedObstacleClearance)
fprintf("M6 planning time:             %.6f s\n",m6Output.metrics.initialPlanningTime)
fprintf("M6 replanning latency:        %.6f s\n",m6Output.metrics.replanningLatency)

if ~exist("Results","dir")
    mkdir("Results")
end

if ~exist("Figures","dir")
    mkdir("Figures")
end

handoff.m5Behavior = m5Output.behavior;
handoff.m5RiskLevel = m5Output.riskLevel;
handoff.m5TargetObject = m5Output.targetObject;
handoff.m5ReplanRequired = m5Output.replanRequired;
handoff.m5TargetSpeed = m5Output.targetSpeed;
handoff.m5DecisionLatency = m5Output.metrics.decisionLatency;
handoff.m6Success = m6Output.success;
handoff.m6ReplanRequired = m6Output.replanRequired;
handoff.m6ReplanCount = m6Output.metrics.replanCount;
handoff.m6CollisionFree = m6Output.metrics.collisionFree;
handoff.m6PredictedClearance = m6Output.metrics.minimumPredictedClearance;
handoff.m6FinalClearance = m6Output.metrics.replannedObstacleClearance;
handoff.m6PlanningTime = m6Output.metrics.initialPlanningTime;
handoff.m6ReplanningLatency = m6Output.metrics.replanningLatency;

save("Results/M5_Stage4_Handoff.mat","handoff")

figure
tiledlayout(1,2)

nexttile
hold on
grid on
axis equal

show(map)

plot( ...
    m6Output.initialStates(:,1), ...
    m6Output.initialStates(:,2), ...
    "b--","LineWidth",1.5)

plot( ...
    m6Output.selectedStates(:,1), ...
    m6Output.selectedStates(:,2), ...
    "g-","LineWidth",2)

plot( ...
    m6Output.conflictPoint(1), ...
    m6Output.conflictPoint(2), ...
    "rx","LineWidth",2,"MarkerSize",12)

plot(startPose(1),startPose(2), ...
    "ko","MarkerFaceColor","k","MarkerSize",8)

plot(goalPose(1),goalPose(2), ...
    "ks","MarkerFaceColor","k","MarkerSize",8)

title("M5 Decision to M6 Adaptive Path")
xlabel("X (m)")
ylabel("Y (m)")
legend("Initial Path","Selected Path","Conflict","Start","Goal")

nexttile
hold on
grid on
axis equal

plot( ...
    m5Input.ego.position(1), ...
    m5Input.ego.position(2), ...
    "ko","MarkerFaceColor","k","MarkerSize",8)

for i = 1:numel(m5Input.objects)

    trajectory = m5Input.objects(i).predictedTrajectory;

    plot( ...
        trajectory(:,1), ...
        trajectory(:,2), ...
        "r--","LineWidth",1.5)

    plot( ...
        m5Input.objects(i).position(1), ...
        m5Input.objects(i).position(2), ...
        "rx","LineWidth",2,"MarkerSize",10)

end

plot( ...
    m6Output.selectedStates(:,1), ...
    m6Output.selectedStates(:,2), ...
    "g-","LineWidth",2)

xlim([0 60])
ylim([5 30])
xlabel("X (m)")
ylabel("Y (m)")
title(sprintf("M5: %s | M6: Replan %d", ...
    m5Output.behavior,m6Output.replanRequired))

sgtitle("M5 Decision + M6 Adaptive Path Planning")

disp(" ")
disp("M5 -> M6 handoff completed successfully.")

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