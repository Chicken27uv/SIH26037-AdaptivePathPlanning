%% SIH 26037
% Module M2 - Multi-Sensor Simulation and Fusion
% Stage M2-D: Camera + LiDAR + Radar fusion
%
% Lightweight weighted fusion without
% Sensor Fusion and Tracking Toolbox.

clc;
clear;
close all;

rng(30);

%% Simulation settings
dt = 0.1;
simTime = 10;
time = 0:dt:simTime;

%% -------------------------------------------------
% SENSOR SETTINGS
%% -------------------------------------------------

% Camera
cameraRange = 30;
cameraHalfFOVdeg = 18;
cameraDetectionProbability = 0.90;
cameraNoiseStd = 0.35;

% LiDAR
lidarRange = 35;
lidarDetectionProbability = 0.96;
lidarNoiseStd = 0.18;

% Radar
radarRange = 45;
radarDetectionProbability = 0.94;
radarPositionNoiseStd = 0.55;
radarVelocityNoiseStd = 0.35;

%% -------------------------------------------------
% FUSION WEIGHTS
%% -------------------------------------------------

% Position weights are inverse measurement variance

cameraWeight = 1/(cameraNoiseStd^2);
lidarWeight  = 1/(lidarNoiseStd^2);
radarWeight  = 1/(radarPositionNoiseStd^2);

%% Ego vehicle
ego.x = 0;
ego.y = 0;
ego.vx = 3.5;
ego.vy = 0;

%% -------------------------------------------------
% ACTORS
%% -------------------------------------------------

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

numActors = length(actors);

%% -------------------------------------------------
% METRICS
%% -------------------------------------------------

cameraErrors = [];
lidarErrors = [];
radarErrors = [];
fusedErrors = [];

radarVelocityErrors = [];

fusionCount = 0;

sensorContributionCount = zeros(1,3);

%% Figure
figure( ...
    'Name','SIH 26037 - M2-D Multi-Sensor Fusion', ...
    'NumberTitle','off');

%% -------------------------------------------------
% MAIN LOOP
%% -------------------------------------------------

for k = 1:length(time)

    cla;
    hold on;
    grid on;

    %% Road
    patch([-15 60 60 -15], ...
          [-10 -10 10 10], ...
          [0.85 0.85 0.85], ...
          'EdgeColor','none');

    %% Ego update
    ego.x = ego.x + ego.vx*dt;
    ego.y = ego.y + ego.vy*dt;

    plot(ego.x,ego.y,'ks', ...
        'MarkerSize',12, ...
        'MarkerFaceColor','k');

    text(ego.x,ego.y-1, ...
        'EGO', ...
        'HorizontalAlignment','center', ...
        'FontWeight','bold');

    %% Camera FOV
    fovAngle = deg2rad(cameraHalfFOVdeg);

    leftX = ego.x + cameraRange*cos(fovAngle);
    leftY = ego.y + cameraRange*sin(fovAngle);

    rightX = ego.x + cameraRange*cos(fovAngle);
    rightY = ego.y - cameraRange*sin(fovAngle);

    plot([ego.x leftX], ...
         [ego.y leftY], ...
         'k--');

    plot([ego.x rightX], ...
         [ego.y rightY], ...
         'k--');

    %% -------------------------------------------------
    % ACTOR LOOP
    %% -------------------------------------------------

    for i = 1:numActors

        %% Update ground truth
        actors(i).state(1) = ...
            actors(i).state(1) + actors(i).state(3)*dt;

        actors(i).state(2) = ...
            actors(i).state(2) + actors(i).state(4)*dt;

        trueX = actors(i).state(1);
        trueY = actors(i).state(2);

        trueVx = actors(i).state(3);
        trueVy = actors(i).state(4);

        %% Ground truth
        plot(trueX,trueY,'ko', ...
            'MarkerSize',8, ...
            'LineWidth',1.2);

        %% Relative geometry
        dx = trueX - ego.x;
        dy = trueY - ego.y;

        rangeToActor = hypot(dx,dy);

        angleToActor = atan2(dy,dx);

        %% ---------------------------------------------
        % SENSOR MEASUREMENTS
        %% ---------------------------------------------

        hasCamera = false;
        hasLidar = false;
        hasRadar = false;

        cameraPos = [NaN NaN];
        lidarPos  = [NaN NaN];
        radarPos  = [NaN NaN];

        radarVel = [NaN NaN];

        %% CAMERA
        cameraVisible = ...
            dx > 0 && ...
            rangeToActor <= cameraRange && ...
            abs(angleToActor) <= fovAngle;

        if cameraVisible && ...
                rand <= cameraDetectionProbability

            hasCamera = true;

            cameraPos = [
                trueX + cameraNoiseStd*randn, ...
                trueY + cameraNoiseStd*randn
            ];

            cameraErrors(end+1) = ...
                norm(cameraPos-[trueX trueY]);

            plot(cameraPos(1),cameraPos(2), ...
                'gx', ...
                'MarkerSize',9, ...
                'LineWidth',1.5);
        end

        %% LIDAR
        if rangeToActor <= lidarRange && ...
                rand <= lidarDetectionProbability

            hasLidar = true;

            lidarPos = [
                trueX + lidarNoiseStd*randn, ...
                trueY + lidarNoiseStd*randn
            ];

            lidarErrors(end+1) = ...
                norm(lidarPos-[trueX trueY]);

            plot(lidarPos(1),lidarPos(2), ...
                'b+', ...
                'MarkerSize',9, ...
                'LineWidth',1.5);
        end

        %% RADAR
        if rangeToActor <= radarRange && ...
                rand <= radarDetectionProbability

            hasRadar = true;

            radarPos = [
                trueX + radarPositionNoiseStd*randn, ...
                trueY + radarPositionNoiseStd*randn
            ];

            radarVel = [
                trueVx + radarVelocityNoiseStd*randn, ...
                trueVy + radarVelocityNoiseStd*randn
            ];

            radarErrors(end+1) = ...
                norm(radarPos-[trueX trueY]);

            radarVelocityErrors(end+1) = ...
                norm(radarVel-[trueVx trueVy]);

            plot(radarPos(1),radarPos(2), ...
                'rx', ...
                'MarkerSize',9, ...
                'LineWidth',1.5);
        end

        %% ---------------------------------------------
        % POSITION FUSION
        %% ---------------------------------------------

        weightedX = 0;
        weightedY = 0;
        totalWeight = 0;

        contributionCount = 0;

        if hasCamera

            weightedX = weightedX + ...
                cameraWeight*cameraPos(1);

            weightedY = weightedY + ...
                cameraWeight*cameraPos(2);

            totalWeight = totalWeight + ...
                cameraWeight;

            contributionCount = contributionCount + 1;

            sensorContributionCount(1) = ...
                sensorContributionCount(1) + 1;
        end

        if hasLidar

            weightedX = weightedX + ...
                lidarWeight*lidarPos(1);

            weightedY = weightedY + ...
                lidarWeight*lidarPos(2);

            totalWeight = totalWeight + ...
                lidarWeight;

            contributionCount = contributionCount + 1;

            sensorContributionCount(2) = ...
                sensorContributionCount(2) + 1;
        end

        if hasRadar

            weightedX = weightedX + ...
                radarWeight*radarPos(1);

            weightedY = weightedY + ...
                radarWeight*radarPos(2);

            totalWeight = totalWeight + ...
                radarWeight;

            contributionCount = contributionCount + 1;

            sensorContributionCount(3) = ...
                sensorContributionCount(3) + 1;
        end

        %% Produce fused object if any sensor detected it
        if totalWeight > 0

            fusedX = weightedX/totalWeight;
            fusedY = weightedY/totalWeight;

            fusionCount = fusionCount + 1;

            fusedError = hypot( ...
                fusedX-trueX, ...
                fusedY-trueY);

            fusedErrors(end+1) = ...
                fusedError;

            %% Fused estimate
            plot(fusedX,fusedY,'md', ...
                'MarkerSize',11, ...
                'LineWidth',2);

            %% Label
            label = sprintf( ...
                'Fused | %s | %d sensors', ...
                actors(i).type, ...
                contributionCount);

            text( ...
                fusedX+0.4, ...
                fusedY+0.4, ...
                label, ...
                'FontWeight','bold');

            %% Use radar velocity if available
            if hasRadar

                quiver( ...
                    fusedX, ...
                    fusedY, ...
                    radarVel(1), ...
                    radarVel(2), ...
                    0.4, ...
                    'm', ...
                    'LineWidth',1.2);
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
        'M2-D Camera + LiDAR + Radar Fusion | Time = %.1f s', ...
        time(k)));

    %% Legend
    h1 = plot(nan,nan,'ko');
    h2 = plot(nan,nan,'gx','LineWidth',1.5);
    h3 = plot(nan,nan,'b+','LineWidth',1.5);
    h4 = plot(nan,nan,'rx','LineWidth',1.5);
    h5 = plot(nan,nan,'md','LineWidth',2);

    legend( ...
        [h1 h2 h3 h4 h5], ...
        {'Ground Truth', ...
         'Camera', ...
         'LiDAR', ...
         'Radar', ...
         'Fused Estimate'}, ...
        'Location','northeastoutside');

    drawnow;
end

%% -------------------------------------------------
% METRICS
%% -------------------------------------------------

cameraRMSE = ...
    sqrt(mean(cameraErrors.^2));

lidarRMSE = ...
    sqrt(mean(lidarErrors.^2));

radarRMSE = ...
    sqrt(mean(radarErrors.^2));

fusedRMSE = ...
    sqrt(mean(fusedErrors.^2));

radarVelocityRMSE = ...
    sqrt(mean(radarVelocityErrors.^2));

%% -------------------------------------------------
% RESULTS
%% -------------------------------------------------

fprintf('\n');
fprintf('====================================================\n');
fprintf(' SIH 26037 - M2-D SENSOR FUSION RESULTS\n');
fprintf('====================================================\n');

fprintf('Camera position RMSE : %.3f m\n', ...
    cameraRMSE);

fprintf('LiDAR position RMSE  : %.3f m\n', ...
    lidarRMSE);

fprintf('Radar position RMSE  : %.3f m\n', ...
    radarRMSE);

fprintf('Fused position RMSE  : %.3f m\n', ...
    fusedRMSE);

fprintf('\nRadar velocity RMSE  : %.3f m/s\n', ...
    radarVelocityRMSE);

fprintf('\nFused object estimates : %d\n', ...
    fusionCount);

fprintf('\nSensor contributions\n');
fprintf('Camera : %d\n',sensorContributionCount(1));
fprintf('LiDAR  : %d\n',sensorContributionCount(2));
fprintf('Radar  : %d\n',sensorContributionCount(3));

fprintf('====================================================\n');

%% -------------------------------------------------
% RMSE COMPARISON
%% -------------------------------------------------

figure( ...
    'Name','M2-D Position RMSE Comparison', ...
    'NumberTitle','off');

sensorRMSE = [
    cameraRMSE
    lidarRMSE
    radarRMSE
    fusedRMSE
];

bar(sensorRMSE);

set(gca, ...
    'XTick',1:4, ...
    'XTickLabel', ...
    {'Camera','LiDAR','Radar','Fused'});

ylabel('Position RMSE (m)');
title('M2-D Sensor vs Fused Position Accuracy');

grid on;

disp("M2-D multi-sensor fusion completed successfully.");