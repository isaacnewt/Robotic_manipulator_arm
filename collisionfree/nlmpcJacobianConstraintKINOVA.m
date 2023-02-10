function [G,Gmv,Ge] = nlmpcJacobianConstraintKINOVA(X,U,e,data, world, robot)

    % Copyright 2020 The MathWorks, Inc.

    p = data.PredictionHorizon;
    numJoints = data.NumOfOutputs;
    numBodies = robot.NumBodies;
    numObstacles = numel(world);

    % Initialize Jacobians
    G = zeros(p, numJoints*2, p*numBodies*numObstacles);
    Gmv = zeros(p, numJoints, p*numBodies*numObstacles);
    Ge = zeros(p*numBodies*numObstacles,1);
    
    iter = 1;
    for i=1:p
        collisionConfig = X(i+1,1:numJoints);
        [~, ~, allWntPts] = checkCollision(robot, collisionConfig', world, 'IgnoreSelfCollision', 'On', 'Exhaustive', 'on'); 
         for j=1:numBodies
            for k=1:numObstacles
                wtnPts = allWntPts(1+(j-1)*3:3+(j-1)*3, 1+(k-1)*2:2+(k-1)*2);
                if isempty(wtnPts(isinf(wtnPts)|isnan(wtnPts)))
                    if any((wtnPts(:,1)-wtnPts(:,2))~=0) 
                        normal = (wtnPts(:,1)-wtnPts(:,2))/norm(wtnPts(:,1)-wtnPts(:,2));
                    else
                        normal = [0;0;0];
                    end
                    bodyJacobian = geometricJacobian(robot,collisionConfig', robot.BodyNames{j});
                    G(i, 1:numJoints,  iter)= -normal' * bodyJacobian(4:6,:);
                else
                    G(i, 1:numJoints,  iter) = zeros(1, numJoints);
                end
                iter = iter + 1;                 
            end           
         end        
    end

end