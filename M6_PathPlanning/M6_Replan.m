function [replannedPath,info,replanningLatency,replanCount] = M6_Replan(map,startPose,goalPose,replanRequired)

if replanRequired
    [replannedPath,info,replanningLatency] = M6_PlanPath(map,startPose,goalPose);
    replanCount = 1;
else
    replannedPath = [];
    info.IsPathFound = false;
    replanningLatency = 0;
    replanCount = 0;
end

end