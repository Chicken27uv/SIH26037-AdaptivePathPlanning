function decision = M5_SelectBehavior(risk,ego,object)

distance = risk.distance;
minimumPredictedDistance = risk.minimumPredictedDistance;
safetyMargin = risk.safetyMargin;

vehicleClasses = ...
    ["vehicle","car","truck","bus","two-wheeler","auto-rickshaw"];

isFollowableVehicle = any(string(object.class) == vehicleClasses);

if risk.level >= 3

    if minimumPredictedDistance <= 0.75*safetyMargin
        behavior = "EMERGENCY_STOP";
        targetSpeed = 0;
    else
        behavior = "STOP";
        targetSpeed = 0;
    end

    replanRequired = true;

elseif risk.level == 2

    sameDirection = dot(object.velocity,ego.velocity) > 0;

    if isFollowableVehicle && sameDirection && distance > safetyMargin

        behavior = "FOLLOW";
        targetSpeed = min(norm(ego.velocity),max(norm(object.velocity),0));
        replanRequired = false;

    else

        behavior = "AVOID";
        targetSpeed = min(norm(ego.velocity),2.0);
        replanRequired = true;

    end

else

    behavior = "CRUISE";
    targetSpeed = min(norm(ego.velocity),5.0);
    replanRequired = false;

end

decision.behavior = behavior;
decision.replanRequired = replanRequired;
decision.targetSpeed = targetSpeed;

end