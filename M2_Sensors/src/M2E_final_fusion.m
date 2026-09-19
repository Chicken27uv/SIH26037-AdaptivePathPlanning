%% SIH 26037
% Module M2 - Multi-Sensor Simulation and Fusion
% Stage M2-E: Final confidence-aware sensor fusion
%
% Fusion strategy:
% - LiDAR preferred for position
% - Camera + Radar fused if LiDAR unavailable
% - Radar preferred for velocity
% - Camera provides semantic class when available
%
% Output:
% Fused object list suitable for M3

clc;
clear;
close all;

rng(40);

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
% FALLBACK POSITION WEIGHTS
%% -------------------------------------------------

cameraWeight = 1/(cameraNoiseStd^2);
radarWeight = 1/(radarPositionNoiseStd^2);

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

fusedPositionErrors = [];
fusedVelocityErrors = [];

fusionCount = 0;

lidarPositionUsed = 0;
fallbackPositionUsed = 0;
radarVelocityUsed = 0;

cameraClassUsed = 0;
fallbackClassUsed = 0;

%% Store final-frame fused objects
finalFusedObjects = struct( ...
    'ObjectID', {}, ...
    'Class', {}, ...
    'Position', {}, ...
    'Velocity', {}, ...
    'PositionSource', {}, ...
    'VelocitySource', {});

%% Figure
figure( ...
    'Name','SIH 26037 - M2-E Final Sensor Fusion', ...
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

    %% Current-frame output
    frameFusedObjects = struct( ...
        'ObjectID', {}, ...
        'Class', {}, ...
        'Position', {}, ...
        'Velocity', {}, ...
        'PositionSource', {}, ...
        'VelocitySource', {});

    %% -------------------------------------------------
    % ACTOR LOOP
    %% -------------------------------------------------

    for i = 1:numActors

        %% Ground-truth update
        actors(i).state(1) = ...
            actors(i).state(1) + actors(i).state(3)*dt;

        actors(i).state(2) = ...
            actors(i).state(2) + actors(i).state(4)*dt;

        trueX = actors(i).state(1);
        trueY = actors(i).state(2);

        trueVx = actors(i).state(3);
        trueVy = actors(i).state(4);

        truePosition = [trueX trueY];
        trueVelocity = [trueVx trueVy];

        %% Plot truth
        plot(trueX,trueY,'ko', ...
            'MarkerSize',7);

        %% Relative geometry
        dx = trueX - ego.x;
        dy = trueY - ego.y;

        rangeToActor = hypot(dx,dy);
        angleToActor = atan2(dy,dx);

        %% ---------------------------------------------
        % SENSOR FLAGS
        %% ---------------------------------------------

        hasCamera = false;
        hasLidar = false;
        hasRadar = false;

        cameraPos = [NaN NaN];
        lidarPos = [NaN NaN];
        radarPos = [NaN NaN];

        radarVel = [NaN NaN];

        cameraClass = "";

        %% ---------------------------------------------
        % CAMERA
        %% ---------------------------------------------

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

            % In this prototype the camera supplies class
            cameraClass = actors(i).type;

            plot(cameraPos(1),cameraPos(2), ...
                'gx', ...
                'MarkerSize',9, ...
                'LineWidth',1.3);
        end

        %% ---------------------------------------------
        % LIDAR
        %% ---------------------------------------------

        if rangeToActor <= lidarRange && ...
                rand <= lidarDetectionProbability

            hasLidar = true;

            lidarPos = [
                trueX + lidarNoiseStd*randn, ...
                trueY + lidarNoiseStd*randn
            ];

            plot(lidarPos(1),lidarPos(2), ...
                'b+', ...
                'MarkerSize',9, ...
                'LineWidth',1.5);
        end

        %% ---------------------------------------------
        % RADAR
        %% ---------------------------------------------

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

            plot(radarPos(1),radarPos(2), ...
                'rx', ...
                'MarkerSize',9, ...
                'LineWidth',1.3);
        end

        %% ---------------------------------------------
        % FUSION
        %% ---------------------------------------------

        if hasCamera || hasLidar || hasRadar

            fusionCount = fusionCount + 1;

            %% POSITION
            if hasLidar

                % LiDAR is most accurate for position
                fusedPosition = lidarPos;
                positionSource = "LiDAR";

                lidarPositionUsed = ...
                    lidarPositionUsed + 1;

            elseif hasCamera && hasRadar

                % Weighted fallback if LiDAR is missing
                fusedX = ...
                    (cameraWeight*cameraPos(1) + ...
                     radarWeight*radarPos(1)) / ...
                    (cameraWeight + radarWeight);

                fusedY = ...
                    (cameraWeight*cameraPos(2) + ...
                     radarWeight*radarPos(2)) / ...
                    (cameraWeight + radarWeight);

                fusedPosition = [fusedX fusedY];

                positionSource = ...
                    "Camera+Radar";

                fallbackPositionUsed = ...
                    fallbackPositionUsed + 1;

            elseif hasCamera

                fusedPosition = cameraPos;
                positionSource = "Camera";

                fallbackPositionUsed = ...
                    fallbackPositionUsed + 1;

            else

                fusedPosition = radarPos;
                positionSource = "Radar";

                fallbackPositionUsed = ...
                    fallbackPositionUsed + 1;
            end

            %% VELOCITY
            if hasRadar

                fusedVelocity = radarVel;
                velocitySource = "Radar";

                radarVelocityUsed = ...
                    radarVelocityUsed + 1;

            else

                % No direct velocity measurement available
                fusedVelocity = [NaN NaN];
                velocitySource = "Unavailable";
            end

            %% CLASS
            if hasCamera

                fusedClass = cameraClass;

                cameraClassUsed = ...
                    cameraClassUsed + 1;

            else

                % Standalone fallback:
                % object type is retained from scenario metadata
                fusedClass = actors(i).type;

                fallbackClassUsed = ...
                    fallbackClassUsed + 1;
            end

            %% -----------------------------------------
            % METRICS
            %% -----------------------------------------

            fusedPositionErrors(end+1) = ...
                norm(fusedPosition - truePosition);

            if hasRadar

                fusedVelocityErrors(end+1) = ...
                    norm(fusedVelocity - trueVelocity);
            end

            %% -----------------------------------------
            % OUTPUT STRUCTURE
            %% -----------------------------------------

            fusedObject.ObjectID = actors(i).id;
            fusedObject.Class = fusedClass;
            fusedObject.Position = fusedPosition;
            fusedObject.Velocity = fusedVelocity;
            fusedObject.PositionSource = positionSource;
            fusedObject.VelocitySource = velocitySource;

            frameFusedObjects(end+1) = ...
                fusedObject;

            %% -----------------------------------------
            % VISUALIZATION
            %% -----------------------------------------

            plot( ...
                fusedPosition(1), ...
                fusedPosition(2), ...
                'md', ...
                'MarkerSize',11, ...
                'LineWidth',2);

            text( ...
                fusedPosition(1)+0.4, ...
                fusedPosition(2)+0.4, ...
                sprintf( ...
                '%s | P:%s | V:%s', ...
                fusedClass, ...
                positionSource, ...
                velocitySource), ...
                'FontWeight','bold');

            %% Velocity arrow
            if hasRadar

                quiver( ...
                    fusedPosition(1), ...
                    fusedPosition(2), ...
                    fusedVelocity(1), ...
                    fusedVelocity(2), ...
                    0.4, ...
                    'm', ...
                    'LineWidth',1.2);
            end
        end
    end

    %% Save latest frame
    finalFusedObjects = ...
        frameFusedObjects;

    %% Formatting
    xlim([-15 60]);
    ylim([-15 15]);

    axis equal;

    xlabel('Longitudinal Position X (m)');
    ylabel('Lateral Position Y (m)');

    title(sprintf( ...
        'M2-E Confidence-Aware Sensor Fusion | Time = %.1f s', ...
        time(k)));

    h1 = plot(nan,nan,'ko');
    h2 = plot(nan,nan,'gx','LineWidth',1.3);
    h3 = plot(nan,nan,'b+','LineWidth',1.5);
    h4 = plot(nan,nan,'rx','LineWidth',1.3);
    h5 = plot(nan,nan,'md','LineWidth',2);

    legend( ...
        [h1 h2 h3 h4 h5], ...
        {'Ground Truth', ...
         'Camera', ...
         'LiDAR', ...
         'Radar', ...
         'Fused Object'}, ...
        'Location','northeastoutside');

    drawnow;
end

%% -------------------------------------------------
% FINAL METRICS
%% -------------------------------------------------

fusedPositionRMSE = ...
    sqrt(mean(fusedPositionErrors.^2));

fusedMeanPositionError = ...
    mean(fusedPositionErrors);

fusedVelocityRMSE = ...
    sqrt(mean(fusedVelocityErrors.^2));

%% -------------------------------------------------
% DISPLAY FINAL RESULTS
%% -------------------------------------------------

fprintf('\n');
fprintf('====================================================\n');
fprintf(' SIH 26037 - M2-E FINAL FUSION RESULTS\n');
fprintf('====================================================\n');

fprintf('Fused position RMSE       : %.3f m\n', ...
    fusedPositionRMSE);

fprintf('Mean fused position error : %.3f m\n', ...
    fusedMeanPositionError);

fprintf('Fused velocity RMSE       : %.3f m/s\n', ...
    fusedVelocityRMSE);

fprintf('\nTotal fused objects        : %d\n', ...
    fusionCount);

fprintf('\nPosition source usage\n');
fprintf('LiDAR primary              : %d\n', ...
    lidarPositionUsed);

fprintf('Fallback position          : %d\n', ...
    fallbackPositionUsed);

fprintf('\nSemantic / velocity usage\n');
fprintf('Camera class used          : %d\n', ...
    cameraClassUsed);

fprintf('Radar velocity used        : %d\n', ...
    radarVelocityUsed);

fprintf('====================================================\n');

%% -------------------------------------------------
% FINAL OBJECT TABLE FOR M3
%% -------------------------------------------------

fprintf('\nFinal-frame fused object list for M3:\n');
fprintf('------------------------------------------------------------\n');

fprintf('%-5s %-16s %-10s %-10s %-10s %-10s\n', ...
    'ID', ...
    'Class', ...
    'X (m)', ...
    'Y (m)', ...
    'Vx', ...
    'Vy');

fprintf('------------------------------------------------------------\n');

for i = 1:length(finalFusedObjects)

    fprintf('%-5d %-16s %-10.2f %-10.2f ', ...
        finalFusedObjects(i).ObjectID, ...
        finalFusedObjects(i).Class, ...
        finalFusedObjects(i).Position(1), ...
        finalFusedObjects(i).Position(2));

    if all(~isnan(finalFusedObjects(i).Velocity))

        fprintf('%-10.2f %-10.2f\n', ...
            finalFusedObjects(i).Velocity(1), ...
            finalFusedObjects(i).Velocity(2));

    else

        fprintf('%-10s %-10s\n', ...
            'N/A', ...
            'N/A');
    end
end

%% Save interface data for M3
save( ...
    '../data/M2_fused_output.mat', ...
    'finalFusedObjects');

disp("M2-E final sensor fusion completed successfully.");
disp("M2 fused output saved for M3.");