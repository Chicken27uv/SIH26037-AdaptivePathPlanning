%% SIH 26037
% Module M3 - Perception and Object Tracking
% Developer: Daniel
%
% Stage M3-B:
% Synthetic road users + noisy sensor detections

clc;
clear;
close all;

rng(7);   % Repeatable random measurements

%% Simulation settings
dt = 0.1;
simTime = 12;
time = 0:dt:simTime;

%% Detection settings
detectionProbability = 0.92;

% Standard deviation of measurement noise (metres)
positionNoiseStd = 0.45;

%% Ego vehicle
ego.x = 0;
ego.y = 0;
ego.vx = 4;
ego.vy = 0;

%% Surrounding road users
actors(1).id = 1;
actors(1).type = "Car";
actors(1).state = [20, 2.5, -1.0, 0];

actors(2).id = 2;
actors(2).type = "Pedestrian";
actors(2).state = [12, 7, 0, -0.8];

actors(3).id = 3;
actors(3).type = "Auto-rickshaw";
actors(3).state = [8, -4, 2.0, 0.25];

actors(4).id = 4;
actors(4).type = "Two-wheeler";
actors(4).state = [-5, -2, 5.0, 0.15];

actors(5).id = 5;
actors(5).type = "Cattle";
actors(5).state = [25, -7, -0.15, 0.7];

%% Statistics
totalPossibleDetections = 0;
successfulDetections = 0;

positionErrors = [];

%% Figure
figure( ...
    'Name','SIH 26037 - M3-B Sensor Detections', ...
    'NumberTitle','off');

%% Simulation loop
for k = 1:length(time)

    cla;
    hold on;
    grid on;

    %% Draw road
    patch([-10 55 55 -10], ...
          [-8 -8 8 8], ...
          [0.85 0.85 0.85], ...
          'EdgeColor','none');

    %% Update ego
    ego.x = ego.x + ego.vx*dt;
    ego.y = ego.y + ego.vy*dt;

    plot(ego.x,ego.y,'ks', ...
        'MarkerSize',12, ...
        'MarkerFaceColor','k');

    text(ego.x,ego.y-1, ...
        'EGO', ...
        'HorizontalAlignment','center', ...
        'FontWeight','bold');

    %% Update actors
    for i = 1:length(actors)

        % Update ground-truth position
        actors(i).state(1) = ...
            actors(i).state(1) + actors(i).state(3)*dt;

        actors(i).state(2) = ...
            actors(i).state(2) + actors(i).state(4)*dt;

        trueX = actors(i).state(1);
        trueY = actors(i).state(2);

        %% Plot TRUE actor location
        plot(trueX,trueY,'bo', ...
            'MarkerSize',9, ...
            'LineWidth',1.5);

        trueLabel = sprintf( ...
            '%s (GT)',actors(i).type);

        text(trueX+0.3,trueY+0.4,trueLabel);

        %% Attempt sensor detection
        totalPossibleDetections = ...
            totalPossibleDetections + 1;

        detected = rand <= detectionProbability;

        if detected

            successfulDetections = ...
                successfulDetections + 1;

            %% Add measurement noise
            measuredX = trueX + ...
                positionNoiseStd*randn;

            measuredY = trueY + ...
                positionNoiseStd*randn;

            %% Calculate measurement error
            errorDistance = hypot( ...
                measuredX-trueX, ...
                measuredY-trueY);

            positionErrors(end+1) = errorDistance;

            %% Plot detection
            plot(measuredX,measuredY,'rx', ...
                'MarkerSize',11, ...
                'LineWidth',2);

            %% Connect true position and measurement
            plot([trueX measuredX], ...
                 [trueY measuredY], ...
                 'k:');

        end

    end

    %% Figure formatting
    xlim([-10 55]);
    ylim([-12 12]);

    axis equal;

    xlabel('Longitudinal Position X (m)');
    ylabel('Lateral Position Y (m)');

    title(sprintf( ...
        'M3-B: Ground Truth vs Sensor Detections | Time = %.1f s', ...
        time(k)));

    %% Simple legend
    h1 = plot(nan,nan,'bo', ...
        'MarkerSize',9, ...
        'LineWidth',1.5);

    h2 = plot(nan,nan,'rx', ...
        'MarkerSize',11, ...
        'LineWidth',2);

    h3 = plot(nan,nan,'ks', ...
        'MarkerSize',10, ...
        'MarkerFaceColor','k');

    legend([h1 h2 h3], ...
        {'Ground Truth','Sensor Detection','Ego Vehicle'}, ...
        'Location','northeastoutside');

    drawnow;

end

%% Calculate metrics

actualDetectionRate = ...
    successfulDetections / totalPossibleDetections;

meanPositionError = mean(positionErrors);

rmsePosition = sqrt(mean(positionErrors.^2));

%% Display results

fprintf('\n');
fprintf('========================================\n');
fprintf(' SIH 26037 - M3-B RESULTS\n');
fprintf('========================================\n');

fprintf('Possible detections : %d\n', ...
    totalPossibleDetections);

fprintf('Successful detections: %d\n', ...
    successfulDetections);

fprintf('Detection rate      : %.2f %%\n', ...
    actualDetectionRate*100);

fprintf('Mean position error : %.3f m\n', ...
    meanPositionError);

fprintf('Position RMSE       : %.3f m\n', ...
    rmsePosition);

fprintf('========================================\n');

disp("M3-B simulation completed successfully.");