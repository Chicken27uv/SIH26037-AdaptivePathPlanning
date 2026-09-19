%% SIH 26037
% Module M2 - Multi-Sensor Simulation and Fusion
% Stage M2-C: LiDAR-like and Radar-like detections
% Developer: Daniel (supporting M2)

clc;
clear;
close all;

rng(20);

%% Simulation settings
dt = 0.1;
simTime = 10;
time = 0:dt:simTime;

%% LiDAR settings
lidarRange = 35;                 % metres
lidarDetectionProbability = 0.96;
lidarNoiseStd = 0.18;            % metres

%% Radar settings
radarRange = 45;                 % metres
radarDetectionProbability = 0.94;
radarPositionNoiseStd = 0.55;    % metres
radarVelocityNoiseStd = 0.35;    % m/s

%% Ego vehicle
ego.x = 0;
ego.y = 0;
ego.vx = 3.5;
ego.vy = 0;

%% Actors
actors(1).id = 1;
actors(1).type = "Car";
actors(1).state = [22, 2.5, -1.0, 0];

actors(2).id = 2;
actors(2).type = "Pedestrian";
actors(2).state = [14, 6.5, 0, -0.7];

actors(3).id = 3;
actors(3).type = "Auto-rickshaw";
actors(3).state = [10, -4.5, 1.8, 0.2];

actors(4).id = 4;
actors(4).type = "Two-wheeler";
actors(4).state = [-4, -2.0, 4.5, 0.1];

actors(5).id = 5;
actors(5).type = "Cattle";
actors(5).state = [27, -7.0, -0.1, 0.65];

%% Metrics
lidarOpportunities = 0;
lidarDetections = 0;
lidarErrors = [];

radarOpportunities = 0;
radarDetections = 0;
radarPositionErrors = [];
radarVelocityErrors = [];

%% Figure
figure( ...
    'Name','SIH 26037 - M2-C LiDAR and Radar', ...
    'NumberTitle','off');

%% Simulation loop
for k = 1:length(time)

    cla;
    hold on;
    grid on;

    %% Road
    patch([-15 60 60 -15], ...
          [-10 -10 10 10], ...
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

    %% Draw LiDAR coverage
    theta = linspace(0,2*pi,200);

    plot( ...
        ego.x + lidarRange*cos(theta), ...
        ego.y + lidarRange*sin(theta), ...
        'b:');

    %% Draw Radar coverage
    plot( ...
        ego.x + radarRange*cos(theta), ...
        ego.y + radarRange*sin(theta), ...
        'r--');

    %% Update actors
    for i = 1:length(actors)

        actors(i).state(1) = ...
            actors(i).state(1) + actors(i).state(3)*dt;

        actors(i).state(2) = ...
            actors(i).state(2) + actors(i).state(4)*dt;

        trueX = actors(i).state(1);
        trueY = actors(i).state(2);

        trueVx = actors(i).state(3);
        trueVy = actors(i).state(4);

        %% Plot ground truth
        plot(trueX,trueY,'ko', ...
            'MarkerSize',7);

        text(trueX+0.3,trueY+0.4, ...
            actors(i).type);

        %% Relative range
        rangeToActor = hypot( ...
            trueX-ego.x, ...
            trueY-ego.y);

        %% --------------------------------
        % LiDAR-like detection
        %% --------------------------------

        if rangeToActor <= lidarRange

            lidarOpportunities = ...
                lidarOpportunities + 1;

            if rand <= lidarDetectionProbability

                lidarX = ...
                    trueX + lidarNoiseStd*randn;

                lidarY = ...
                    trueY + lidarNoiseStd*randn;

                lidarDetections = ...
                    lidarDetections + 1;

                lidarError = hypot( ...
                    lidarX-trueX, ...
                    lidarY-trueY);

                lidarErrors(end+1) = ...
                    lidarError;

                %% Plot LiDAR detection
                plot(lidarX,lidarY,'b+', ...
                    'MarkerSize',10, ...
                    'LineWidth',2);
            end
        end

        %% --------------------------------
        % Radar-like detection
        %% --------------------------------

        if rangeToActor <= radarRange

            radarOpportunities = ...
                radarOpportunities + 1;

            if rand <= radarDetectionProbability

                radarX = ...
                    trueX + radarPositionNoiseStd*randn;

                radarY = ...
                    trueY + radarPositionNoiseStd*randn;

                radarVx = ...
                    trueVx + radarVelocityNoiseStd*randn;

                radarVy = ...
                    trueVy + radarVelocityNoiseStd*randn;

                radarDetections = ...
                    radarDetections + 1;

                radarPositionError = hypot( ...
                    radarX-trueX, ...
                    radarY-trueY);

                radarVelocityError = hypot( ...
                    radarVx-trueVx, ...
                    radarVy-trueVy);

                radarPositionErrors(end+1) = ...
                    radarPositionError;

                radarVelocityErrors(end+1) = ...
                    radarVelocityError;

                %% Plot Radar detection
                plot(radarX,radarY,'rx', ...
                    'MarkerSize',9, ...
                    'LineWidth',1.5);

                %% Radar velocity vector
                quiver( ...
                    radarX, ...
                    radarY, ...
                    radarVx, ...
                    radarVy, ...
                    0.4, ...
                    'r', ...
                    'LineWidth',1);
            end
        end
    end

    %% Formatting
    xlim([-15 60]);
    ylim([-15 15]);

    axis equal;

    xlabel('Longitudinal Position X (m)');
    ylabel('Lateral Position Y (m)');

    title(sprintf( ...
        'M2-C LiDAR + Radar Simulation | Time = %.1f s', ...
        time(k)));

    %% Legend
    h1 = plot(nan,nan,'ko');
    h2 = plot(nan,nan,'b+','LineWidth',2);
    h3 = plot(nan,nan,'rx','LineWidth',1.5);
    h4 = plot(nan,nan,'ks','MarkerFaceColor','k');

    legend( ...
        [h1 h2 h3 h4], ...
        {'Ground Truth', ...
         'LiDAR Detection', ...
         'Radar Detection', ...
         'Ego Vehicle'}, ...
        'Location','northeastoutside');

    drawnow;
end

%% LiDAR metrics
lidarDetectionRate = ...
    lidarDetections / lidarOpportunities;

lidarMeanError = ...
    mean(lidarErrors);

lidarRMSE = ...
    sqrt(mean(lidarErrors.^2));

%% Radar metrics
radarDetectionRate = ...
    radarDetections / radarOpportunities;

radarMeanPositionError = ...
    mean(radarPositionErrors);

radarPositionRMSE = ...
    sqrt(mean(radarPositionErrors.^2));

radarMeanVelocityError = ...
    mean(radarVelocityErrors);

radarVelocityRMSE = ...
    sqrt(mean(radarVelocityErrors.^2));

%% Results
fprintf('\n');
fprintf('====================================================\n');
fprintf(' SIH 26037 - M2-C SENSOR RESULTS\n');
fprintf('====================================================\n');

fprintf('\nLiDAR Results\n');
fprintf('----------------------------------------\n');

fprintf('Detection opportunities : %d\n', ...
    lidarOpportunities);

fprintf('LiDAR detections        : %d\n', ...
    lidarDetections);

fprintf('Detection rate          : %.2f %%\n', ...
    lidarDetectionRate*100);

fprintf('Mean position error     : %.3f m\n', ...
    lidarMeanError);

fprintf('Position RMSE           : %.3f m\n', ...
    lidarRMSE);

fprintf('\nRadar Results\n');
fprintf('----------------------------------------\n');

fprintf('Detection opportunities : %d\n', ...
    radarOpportunities);

fprintf('Radar detections        : %d\n', ...
    radarDetections);

fprintf('Detection rate          : %.2f %%\n', ...
    radarDetectionRate*100);

fprintf('Mean position error     : %.3f m\n', ...
    radarMeanPositionError);

fprintf('Position RMSE           : %.3f m\n', ...
    radarPositionRMSE);

fprintf('Mean velocity error     : %.3f m/s\n', ...
    radarMeanVelocityError);

fprintf('Velocity RMSE           : %.3f m/s\n', ...
    radarVelocityRMSE);

fprintf('====================================================\n');

disp("M2-C LiDAR and Radar simulation completed successfully.");