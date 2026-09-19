clear
clc
close all

scenarioNames = [
    "VillageRoad"
    "UrbanIntersection"
    "HighwayMerge"
    "MarketArea"
    "CattleCrossing"
    ];

results = struct([]);

for i = 1:numel(scenarioNames)

    scenario = M8_CreateScenarioInputs(scenarioNames(i));

    result = M8_RunScenario(scenario);

    metrics = M8_CalculateMetrics(result);

    results(i).Scenario = metrics.scenario;
    results(i).Completed = metrics.completed;
    results(i).CollisionFree = metrics.collisionFree;
    results(i).MinimumClearance_m = metrics.minimumClearance;
    results(i).ReplanningLatency_s = metrics.replanningLatency;
    results(i).PathSmoothness = metrics.pathSmoothness;
    results(i).ReplanCount = metrics.replanCount;
    results(i).PathLength_m = metrics.pathLength;

end

resultsTable = struct2table(results);

completionRate = ...
    100 * mean(resultsTable.Completed);

collisionFreeRate = ...
    100 * mean(resultsTable.CollisionFree);

averageClearance = ...
    mean(resultsTable.MinimumClearance_m);

averageReplanningLatency = ...
    mean(resultsTable.ReplanningLatency_s);

averagePathSmoothness = ...
    mean(resultsTable.PathSmoothness);

totalReplans = ...
    sum(resultsTable.ReplanCount);

fprintf("\nM8 STAGE 2 FIVE-SCENARIO VALIDATION\n")
fprintf("-------------------------------------\n")

disp(resultsTable)

fprintf("Scenario completion rate:       %.1f %%\n",completionRate)
fprintf("Collision-free rate:             %.1f %%\n",collisionFreeRate)
fprintf("Average minimum clearance:       %.2f m\n",averageClearance)
fprintf("Average replanning latency:      %.4f s\n",averageReplanningLatency)
fprintf("Average path smoothness:         %.4f\n",averagePathSmoothness)
fprintf("Total replans:                   %d\n",totalReplans)

figure

bar(categorical(resultsTable.Scenario),resultsTable.MinimumClearance_m)

title("M8 Minimum Clearance Across SIH Scenarios")
xlabel("Scenario")
ylabel("Minimum Clearance [meters]")
grid on

figure

bar(categorical(resultsTable.Scenario),resultsTable.PathSmoothness)

title("M8 Path Smoothness Across SIH Scenarios")
xlabel("Scenario")
ylabel("Path Smoothness")
grid on

figure

bar(categorical(resultsTable.Scenario),resultsTable.PathLength_m)

title("M8 Path Length Across SIH Scenarios")
xlabel("Scenario")
ylabel("Path Length [meters]")
grid on