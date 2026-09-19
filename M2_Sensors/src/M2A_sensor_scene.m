%% SIH 26037
% Module M2 - Multi-Sensor Simulation and Fusion
% Stage M2-A: Synthetic multi-actor scene
% Developer: Daniel (supporting Vyshal's M2)

clc;
clear;
close all;

%% Simulation settings
dt = 0.1;
simTime = 10;
time = 0:dt:simTime;

%% Ego vehicle
ego.x = 0;
ego.y = 0;
ego.vx = 3.5;
ego.vy = 0;

%% Surrounding road users
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

%% Figure
figure( ...
    'Name','SIH 26037 - M2-A Synthetic Sensor Scene', ...
    'NumberTitle','off');

%% Simulation
for k = 1:length(time)

    cla;
    hold on;
    grid on;

    %% Road area
    patch([-10 55 55 -10], ...
          [-8 -8 8 8], ...
          [0.85 0.85 0.85], ...
          'EdgeColor','none');

    %% Update ego
    ego.x = ego.x + ego.vx*dt;
    ego.y = ego.y + ego.vy*dt;

    plot(ego.x, ego.y, 'ks', ...
        'MarkerSize',12, ...
        'MarkerFaceColor','k');

    text(ego.x, ego.y-1, ...
        'EGO', ...
        'HorizontalAlignment','center', ...
        'FontWeight','bold');

    %% Sensor field-of-view guides
    % Camera-like forward field
    cameraRange = 30;
    cameraHalfWidth = 8;

    plot([ego.x ego.x+cameraRange], ...
         [ego.y ego.y+cameraHalfWidth], ...
         'k--');

    plot([ego.x ego.x+cameraRange], ...
         [ego.y ego.y-cameraHalfWidth], ...
         'k--');

    % Radar range circle
    radarRange = 25;

    th = linspace(0,2*pi,150);
    plot(ego.x + radarRange*cos(th), ...
         ego.y + radarRange*sin(th), ...
         ':');

    %% Update actors
    for i = 1:length(actors)

        actors(i).state(1) = ...
            actors(i).state(1) + actors(i).state(3)*dt;

        actors(i).state(2) = ...
            actors(i).state(2) + actors(i).state(4)*dt;

        x = actors(i).state(1);
        y = actors(i).state(2);

        plot(x,y,'o', ...
            'MarkerSize',10, ...
            'LineWidth',2);

        text(x+0.4,y+0.4, ...
            sprintf('ID %d | %s', ...
            actors(i).id, ...
            actors(i).type));
    end

    %% Formatting
    xlim([-10 55]);
    ylim([-12 12]);
    axis equal;

    xlabel('Longitudinal Position X (m)');
    ylabel('Lateral Position Y (m)');

    title(sprintf( ...
        'M2-A Synthetic Multi-Actor Scene | Time = %.1f s', ...
        time(k)));

    drawnow;
end

disp("M2-A synthetic sensor scene completed successfully.");