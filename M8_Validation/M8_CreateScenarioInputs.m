function scenario = M8_CreateScenarioInputs(name)

scenario.name = name;
scenario.dt = 0.1;
scenario.duration = 20;
scenario.time = 0:scenario.dt:scenario.duration;

switch name

    case "VillageRoad"
        scenario.startPose = [5 15 0];
        scenario.goalPoint = [55 15];
        scenario.obstacleStart = [30 5];
        scenario.obstacleVelocity = [0 1];
        scenario.avoidanceStart = 8;
        scenario.avoidanceEnd = 12;
        scenario.avoidanceOffset = 4;

    case "UrbanIntersection"
        scenario.startPose = [5 15 0];
        scenario.goalPoint = [55 15];
        scenario.obstacleStart = [30 5];
        scenario.obstacleVelocity = [0 1];
        scenario.avoidanceStart = 8;
        scenario.avoidanceEnd = 12;
        scenario.avoidanceOffset = 4;

    case "HighwayMerge"
        scenario.startPose = [5 10 0];
        scenario.goalPoint = [55 10];
        scenario.obstacleStart = [18 7];
        scenario.obstacleVelocity = [1.2 0.3];
        scenario.avoidanceStart = 8;
        scenario.avoidanceEnd = 12;
        scenario.avoidanceOffset = 3;

    case "MarketArea"
        scenario.startPose = [5 15 0];
        scenario.goalPoint = [55 15];
        scenario.obstacleStart = [28 8];
        scenario.obstacleVelocity = [0.2 0.7];
        scenario.avoidanceStart = 8;
        scenario.avoidanceEnd = 12;
        scenario.avoidanceOffset = 4;

    case "CattleCrossing"
        scenario.startPose = [5 15 0];
        scenario.goalPoint = [55 15];
        scenario.obstacleStart = [30 5];
        scenario.obstacleVelocity = [0 1];
        scenario.avoidanceStart = 8;
        scenario.avoidanceEnd = 12;
        scenario.avoidanceOffset = 4;

    otherwise
        error("Unknown scenario: %s",name)

end

scenario.egoSpeed = 2.5;
scenario.minimumSafeClearance = 1.0;

end