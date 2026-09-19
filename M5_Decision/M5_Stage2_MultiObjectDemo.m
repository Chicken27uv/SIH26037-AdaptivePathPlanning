clc
clear
close all

scenarioNames = [ ...
    "mixedTraffic", ...
    "pedestrianPriority", ...
    "cattlePriority", ...
    "multipleClear", ...
    "stopScenario", ...
    "criticalMixedTraffic"];

scenarioInputs = cell(size(scenarioNames));

scenarioInputs{1} = M5_CreateMixedTrafficScenario;
scenarioInputs{2} = M5_CreatePedestrianPriorityScenario;
scenarioInputs{3} = M5_CreateCattlePriorityScenario;
scenarioInputs{4} = M5_CreateMultipleClearScenario;
scenarioInputs{5} = M5_CreateStopScenario;
scenarioInputs{6} = M5_CreateCriticalMixedTrafficScenario;

results = strings(numel(scenarioNames),8);

figure
tiledlayout(2,3)

for i = 1:numel(scenarioNames)

    input = scenarioInputs{i};
    output = M5_DecisionLogic(input);

    results(i,:) = [ ...
        scenarioNames(i), ...
        output.behavior, ...
        string(output.replanRequired), ...
        output.riskLevel, ...
        output.targetObject, ...
        string(output.metrics.numberOfObjects), ...
        string(output.metrics.numberOfConflicts), ...
        string(output.metrics.decisionLatency)];

    nexttile
    hold on
    grid on
    axis equal

    plot(input.ego.position(1), ...
        input.ego.position(2), ...
        "ko","MarkerFaceColor","k","MarkerSize",9)

    for j = 1:numel(input.objects)

        plot(input.objects(j).position(1), ...
            input.objects(j).position(2), ...
            "rx","LineWidth",2,"MarkerSize",10)

        trajectory = input.objects(j).predictedTrajectory;

        plot(trajectory(:,1), ...
            trajectory(:,2), ...
            "r--","LineWidth",1)

        text( ...
            input.objects(j).position(1)+0.3, ...
            input.objects(j).position(2)+0.3, ...
            string(input.objects(j).class))

    end

    xlim([-2 22])
    ylim([-8 8])

    title(sprintf("%s | %s", ...
        scenarioNames(i), ...
        output.behavior))

    xlabel("X (m)")
    ylabel("Y (m)")

end

sgtitle("M5 Stage 2 Multi-Object Decision Validation")

disp("M5 STAGE 2 MULTI-OBJECT DECISION VALIDATION")
disp("---------------------------------------------")
disp("Scenario              Behavior          Replan       Risk        Target          Objects  Conflicts")

for i = 1:size(results,1)

    fprintf("%-20s %-16s %-11s %-10s %-15s %d        %d\n", ...
        results(i,1), ...
        results(i,2), ...
        results(i,3), ...
        results(i,4), ...
        results(i,5), ...
        str2double(results(i,6)), ...
        str2double(results(i,7)))

end

successfulDecisions = ...
    sum(results(:,2) ~= "");

replanningDecisions = ...
    sum(results(:,3) == "true");

averageLatency = mean(str2double(results(:,8)));

disp(" ")
fprintf("Scenarios evaluated:          %d\n",numel(scenarioNames))
fprintf("Successful decisions:         %d\n",successfulDecisions)
fprintf("Replanning decisions:         %d\n",replanningDecisions)
fprintf("Average decision latency:     %.6f s\n",averageLatency)

disp(" ")
disp("All M5 Stage 2 scenarios completed.")

function input = M5_CreateMixedTrafficScenario

input.ego.position = [0 0];
input.ego.velocity = [5 0];
input.ego.heading = 0;

input.objects(1) = M5_CreateObject( ...
    "vehicle",[15 0],[2 0],[15 0;17 0;19 0;21 0]);

input.objects(2) = M5_CreateObject( ...
    "pedestrian",[9 -3],[0 1],[9 -3;9 -1.5;9 0;9 1.5]);

input.objects(3) = M5_CreateObject( ...
    "two-wheeler",[12 4],[-1 -0.5],[12 4;11.5 3.5;11 3]);

end

function input = M5_CreatePedestrianPriorityScenario

input.ego.position = [0 0];
input.ego.velocity = [5 0];
input.ego.heading = 0;

input.objects(1) = M5_CreateObject( ...
    "vehicle",[8 0],[1 0],[8 0;9 0;10 0;11 0]);

input.objects(2) = M5_CreateObject( ...
    "pedestrian",[6 -1],[0 1],[6 -1;6 -0.5;6 0;6 0.5]);

input.objects(3) = M5_CreateObject( ...
    "auto-rickshaw",[18 3],[-1 0],[18 3;17 3;16 3]);

end

function input = M5_CreateCattlePriorityScenario

input.ego.position = [0 0];
input.ego.velocity = [5 0];
input.ego.heading = 0;

input.objects(1) = M5_CreateObject( ...
    "vehicle",[10 0],[1 0],[10 0;11 0;12 0]);

input.objects(2) = M5_CreateObject( ...
    "cattle",[5 0],[0.2 0],[5 0;5.2 0;5.4 0]);

input.objects(3) = M5_CreateObject( ...
    "pedestrian",[15 4],[0 0],[15 4;15 4;15 4]);

end

function input = M5_CreateMultipleClearScenario

input.ego.position = [0 0];
input.ego.velocity = [5 0];
input.ego.heading = 0;

input.objects(1) = M5_CreateObject( ...
    "vehicle",[18 6],[0 0],[18 6;18 6;18 6]);

input.objects(2) = M5_CreateObject( ...
    "pedestrian",[15 -6],[0 0],[15 -6;15 -6;15 -6]);

input.objects(3) = M5_CreateObject( ...
    "cattle",[20 5],[0 0],[20 5;20 5;20 5]);

end

function input = M5_CreateStopScenario

input.ego.position = [0 0];
input.ego.velocity = [5 0];
input.ego.heading = 0;

input.objects.class = "pedestrian";
input.objects.position = [0 3];
input.objects.velocity = [0 0];
input.objects.predictedTrajectory = [0 3;0 3;0 3];

end

function input = M5_CreateCriticalMixedTrafficScenario

input.ego.position = [0 0];
input.ego.velocity = [5 0];
input.ego.heading = 0;

input.objects(1) = M5_CreateObject( ...
    "vehicle",[8 0],[0 0],[8 0;8 0;8 0]);

input.objects(2) = M5_CreateObject( ...
    "cattle",[3 0],[-1 0],[3 0;2.5 0;2 0]);

input.objects(3) = M5_CreateObject( ...
    "pedestrian",[7 -1],[0 1],[7 -1;7 -0.5;7 0]);

end

function object = M5_CreateObject(class,position,velocity,predictedTrajectory)

object.class = class;
object.position = position;
object.velocity = velocity;
object.predictedTrajectory = predictedTrajectory;

end