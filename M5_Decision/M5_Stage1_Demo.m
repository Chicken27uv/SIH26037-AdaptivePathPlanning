clc
clear
close all

scenarios = ["empty","slowvehicle","pedestrian","cattle","emergency","multi"];

results = strings(numel(scenarios),5);

for i = 1:numel(scenarios)

    input = M5_CreateTestInput(scenarios(i));
    output = M5_DecisionLogic(input);

    results(i,:) = [ ...
        scenarios(i), ...
        output.behavior, ...
        string(output.replanRequired), ...
        output.riskLevel, ...
        string(output.targetSpeed)];

end

disp("M5 STAGE 1 DECISION AND BEHAVIOR LOGIC")
disp("---------------------------------------")
disp("Scenario              Behavior          Replan       Risk        TargetSpeed")

for i = 1:size(results,1)

    fprintf("%-20s %-16s %-11s %-10s %.2f m/s\n", ...
        results(i,1), ...
        results(i,2), ...
        results(i,3), ...
        results(i,4), ...
        str2double(results(i,5)))

end

figure
tiledlayout(2,3)

for i = 1:numel(scenarios)

    input = M5_CreateTestInput(scenarios(i));
    output = M5_DecisionLogic(input);

    nexttile
    hold on
    axis equal
    grid on

    plot(input.ego.position(1), ...
        input.ego.position(2), ...
        "ko","MarkerFaceColor","k","MarkerSize",8)

    if ~isempty(input.objects)

        for j = 1:numel(input.objects)

            plot(input.objects(j).position(1), ...
                input.objects(j).position(2), ...
                "rx","LineWidth",2,"MarkerSize",10)

            plot(input.objects(j).predictedTrajectory(:,1), ...
                input.objects(j).predictedTrajectory(:,2), ...
                "r--")

        end

    end

    xlim([-2 20])
    ylim([-8 8])

    title(sprintf("%s | %s",scenarios(i),output.behavior))
    xlabel("X (m)")
    ylabel("Y (m)")

end

sgtitle("M5 Decision and Behavior Logic")

disp(" ")
disp("All M5 Stage 1 scenarios completed.")