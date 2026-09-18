function cfg = projectConfig()
%PROJECTCONFIG Shared configuration for SIH 26037 project.
%
% This file defines common units, coordinate conventions, scenario names,
% and integration interface conventions used across M1-M8.

%% Project information
cfg.project.name = "SIH26037_AdaptivePathPlanning";
cfg.project.problemStatement = "SIH 26037";

%% Units
cfg.units.position = "m";
cfg.units.velocity = "m/s";
cfg.units.acceleration = "m/s^2";
cfg.units.heading = "rad";
cfg.units.time = "s";

%% Coordinate convention
% World-frame convention:
%   x = forward/east-west direction of the scenario
%   y = lateral/north-south direction of the scenario
%
% Position:
%   [x y]
%
% Velocity:
%   [vx vy]
%
% Heading:
%   radians, measured counter-clockwise from +x.

cfg.coordinates.positionFormat = "[x y]";
cfg.coordinates.velocityFormat = "[vx vy]";
cfg.coordinates.headingFormat = "rad";

%% Common scenario names
cfg.scenarios = [
    "VillageRoad"
    "UrbanIntersection"
    "HighwayMerge"
    "MarketArea"
    "CattleCrossing"
    ];

%% Planning defaults
cfg.planning.predictionHorizon = 3.0;   % seconds
cfg.planning.replanningEnabled = true;
cfg.planning.collisionBuffer = 1.0;    % meters

%% Integration interface
% Common tracked/predicted object representation:
%
% object.id
% object.class
% object.position     -> [x y] m
% object.velocity     -> [vx vy] m/s
% object.heading      -> rad
%
% Trajectory representation:
%   N x 2 matrix [x y]

cfg.interface.positionFormat = "[x y]";
cfg.interface.velocityFormat = "[vx vy]";
cfg.interface.trajectoryFormat = "N x 2 [x y]";

end