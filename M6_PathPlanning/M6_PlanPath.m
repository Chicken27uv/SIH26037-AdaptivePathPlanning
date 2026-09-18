function [path,info,planningTime] = M6_PlanPath(map,startPose,goalPose)

ss = stateSpaceSE2;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

validator = validatorOccupancyMap(ss);
validator.Map = map;
validator.ValidationDistance = 0.1;

planner = plannerHybridAStar(validator, ...
    MinTurningRadius=3, ...
    MotionPrimitiveLength=4, ...
    MotionDirection="forward");

tic
[path,~,info] = plan(planner,startPose,goalPose);
planningTime = toc;

end