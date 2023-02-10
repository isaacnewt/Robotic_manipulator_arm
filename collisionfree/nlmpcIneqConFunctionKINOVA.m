function cineq = nlmpcIneqConFunctionKINOVA(X,U,e,data, safetyDistance, world, robot)

    % Copyright 2020 The MathWorks, Inc.

    p = data.PredictionHorizon;
    numJoints = data.NumOfOutputs;
    numBodies = robot.NumBodies;
    numObstacles = numel(world);
    allDistances = zeros(p*numBodies*numObstacles,1);
    for i =1:p
        collisionConfig = X(i+1,1:numJoints);
        [~, separationDist, ~] = checkCollision(robot, collisionConfig', world, 'IgnoreSelfCollision', 'On', 'Exhaustive', 'on');
        tempDistances = separationDist(1:numBodies,1:numObstacles);
        tempDistances(isinf(tempDistances)|isnan(tempDistances)) = 1e5; % set to random large distance
        allDistances((1+(i-1)*numBodies*numObstacles):numBodies*numObstacles*i,1) = reshape(tempDistances', [numBodies*numObstacles,1]);   
    end
    cineq = -allDistances + safetyDistance;
end