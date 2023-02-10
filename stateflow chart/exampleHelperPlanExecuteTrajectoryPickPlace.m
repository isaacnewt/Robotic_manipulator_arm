function [positions, velocities, accelerations, timestamp, success] = exampleHelperPlanExecuteTrajectoryPickPlace(robot, mpcTimeStep, obstacles, endEffector, jointInit, taskFinal, tolerance, avoidCollisions)
% This function is for internal use and may be removed in a future release
%
%exampleHelperPlanExecuteTrajectoryPickPlace Generate and simulate motion along a collision-free trajectory
%   This function generates a collision-free trajectory between an initial
%   configuration given by JOINTINIT and a target task-space orientation,
%   provided by TASKFINAL. The function uses a nonlinear model predictive
%   controller to generate optimal trajectories under the constraint that
%   they cannot be in collision. The function then simulates the tracking
%   behavior of the manipulator to those reference trajectories as modeled
%   by a manipulator under closed-loop control, and updates the
%   visualization. The function returns the positions, velocities, and
%   acceleration from the generated reference trajectories.
%
%   This helper function uses nlmpcmove to generate the trajectories,
%   checkCollision to check for
%   collisions, and jointSpaceMotionModel to simulate the motion of the
%   manipulator.

%   Copyright 2020 The MathWorks, Inc.

% Plan collision-free trajectories using nonlinear model predictive control 
curFormat = robot.DataFormat;
robot.DataFormat = 'column';

% Disable display messages
mpcverbosity off;

% Get number of joints
numJoints = numel(robot.homeConfiguration);

% Get number of obstacles
numObstacles = numel(obstacles);

% Get number of collision bodies
if ~isempty(obstacles) && avoidCollisions
    [~, separationDist, ~] = checkCollision(robot, homeConfiguration(robot), obstacles, 'IgnoreSelfCollision', 'On', 'Exhaustive', 'on');
    tempDistances = separationDist(1:robot.NumBodies,1:numObstacles);
    bodyIndices = find((isinf(tempDistances(:,1))|isnan(tempDistances(:,1)))==0);
    numBodies = numel(bodyIndices);
else
    numBodies = robot.NumBodies;
end
        
% Current robot joint configuration
currentRobotJConfig = wrapToPi(jointInit');

% Final (desired) end-effector pose
anglesFinal = rotm2eul(taskFinal(1:3,1:3),'XYZ');
poseFinal = [taskFinal(1:3,4);anglesFinal']; % 6x1 vector for final pose: [x, y, z, phi, theta, psi]

% Initialize safety distance away from the obstacles
safetyDistance = 0.005; 

% World of obstacles
world = obstacles;

%% Set up the Nonlinear Model Predictive Controller (NLMPC)
% Cost weights
Qr = diag([1 1 1 0 0 0]); % running cost weight on desired end-effector pose [x, y, z, phi, theta, psi]
Qt = diag([10 10 10 1 1 1]); % terminal cost weight on desired end-effector pose [x, y, z, phi, theta, psi]
Qu = diag([1 1 1 1 1 1 1])/10; % input cost weight on joint accelerations qDdot
Qv = diag([1 1 1 1 1 1 1])/10;

% Model joints as double integrators
nx = numJoints * 2; % [q,qDot]
ny = numJoints; % [q]
nu = numJoints; % [qDdot]

% Initialize nlmpc object
nlobj = nlmpc(nx,ny,nu);

% Solver time step
Ts = mpcTimeStep; % seconds
nlobj.Ts = Ts; 

% Configure NLMPC solver functions
nlobj.Model.StateFcn = @(x,u) nlmpcModel(x,u);  

nlobj.Model.OutputFcn = @(x,u) x(1:numJoints);

nlobj.Optimization.CustomCostFcn = @(X,U,e,data) nlmpcCostFunction(X,U,e,data, poseFinal, robot, endEffector, Qr, Qt, Qu, Qv); 

if ~isempty(world) && avoidCollisions
    nlobj.Optimization.CustomIneqConFcn = @(X,U,e,data) myIneqConFunction(X,U,e,data, safetyDistance, world, robot);
end

nlobj.Jacobian.OutputFcn = @(x,u) nlmpcJacobianOutputModel(x,u);

nlobj.Jacobian.StateFcn = @(x,u) nlmpcJacobianModel(x,u);

nlobj.Jacobian.CustomCostFcn = @(X,U,e,data) nlmpcJacobianCost(X,U,e,data, poseFinal, robot, endEffector, Qr, Qt, Qu, Qv);

if ~isempty(world) && avoidCollisions
    nlobj.Jacobian.CustomIneqConFcn = @(X,U,e,data) nlmpcJacobianConstraint(X,U,e,data, world, robot);
end

nlobj.Optimization.SolverOptions.FunctionTolerance = 0.01;

nlobj.Optimization.SolverOptions.StepTolerance = 0.01;

nlobj.Optimization.SolverOptions.MaxIter = 5;

nlobj.Optimization.UseSuboptimalSolution = true;

nlobj.Optimization.ReplaceStandardCost = true;

% nlobj.Optimization.SolverOptions.Display = 'iter-detailed';

nlobj.Optimization.SolverOptions.ConstraintTolerance = 0.01;

% Set constraint on States and MV.
stateMinValues = {-174.53;-2.2000;-174.53;-2.5656;-174.53;-2.0500;-174.53;...
    -0.8727;-0.8727;-0.8727;-0.8727;-0.8727;-0.8727;-0.8727};
stateMaxValues = {174.53;2.2000;174.53;2.5656;174.53;2.0500;174.53;...
    0.8727;0.8727;0.8727;0.8727;0.8727;0.8727;0.8727};

nlobj.States = struct('Min',stateMinValues,...
    'Max',stateMaxValues);
nlobj.MV = struct('Min',{-1;-1;-1;-1;-10;-10;-10},'Max',{1;1;1;1;10;10;10});

% Time horizon in seconds
p = 2; 
nlobj.PredictionHorizon = p; % prediction horizon
nlobj.ControlHorizon = 1; % control horizon

%% Generate Reference Trajectories using Closed-loop trajectory optimization

% Initial conditions
x0 = [currentRobotJConfig', zeros(1,numJoints)];
u0 = zeros(1,numJoints);
options = nlmpcmoveopt;
maxIters = 50;
success = 1;

% Initialize arrays to store the results
positions = zeros(numJoints,maxIters+1);
positions(:,1) = x0(1:numJoints)';
velocities = zeros(numJoints,maxIters+1);
velocities(:,1) = x0(numJoints+1:end)';
accelerations = zeros(numJoints,maxIters+1);
accelerations(:,1) = u0';
timestamp = zeros(1,maxIters+1);


% Run nlmpc iteratively over the specified time horizon until goal is
% achieved or up to maxIters.
mv = u0;
time = 0;
numInfeas = 0;
% Uncomment below to display all successful outputs
% disp('Calculating collision-free trajectory...')
for timestep=1:maxIters
    % Optimize next trajectory point
    [mv,options,info] = nlmpcmove(nlobj,x0,mv,[],[], options);
    
    if info.ExitFlag < 0
        numInfeas = numInfeas + 1;
        disp('Failed to compute a feasible trajectory in this step...')
    end
    
    if numInfeas>2
        warning('Many infeasible solutions in a row. Aborting...')
        success = 0;
        break;
    end
    
    % Update initial state and time for next iteration
    x0 = info.Xopt(2,:);
    time = time + nlobj.Ts;
    
    % Store trajectory points
    positions(:,timestep+1) = x0(1:numJoints)';
    velocities(:,timestep+1) = x0(numJoints+1:end)';
    accelerations(:,timestep+1) = info.MVopt(2,:)';
    timestamp(timestep+1) = time;
    
    % Check if goal is achieved 
    jointTempFinal = info.Xopt(2,1:numJoints);
    taskTempFinal = getTransform(robot, jointTempFinal', endEffector);
    anglesTempFinal = rotm2eul(taskTempFinal(1:3,1:3), 'XYZ');
    poseTempFinal =  [taskTempFinal(1:3,4);anglesTempFinal'];
    diffTerminal = abs([poseFinal(1:3)-poseTempFinal(1:3); angdiff(poseTempFinal(4:6),poseFinal(4:6))]);
    if all(diffTerminal<tolerance)       
        break; % goal achieved
    end
end

robot.DataFormat = curFormat;

%% Output the reference trajectories
tFinal = timestep+1;
positions = positions(:,1:tFinal);
velocities = velocities(:,1:tFinal);
accelerations = accelerations(:,1:tFinal);
timestamp = timestamp(:,1:tFinal);
return;

%% Helper nlmpc functions

function dxdt = nlmpcModel(x,u)
    dxdt = zeros(size(x));
    dxdt(1:numJoints) = x(numJoints+1:end);
    dxdt(numJoints+1:end) = u;
end

function [A, B] = nlmpcJacobianModel(x,u)
    A = zeros(numJoints*2, numJoints * 2);    
    A(1:numJoints, numJoints+1:end) = eye(numJoints);
    B = zeros(numJoints*2,numJoints);
    B(numJoints+1:end,:)=eye(numJoints); 
end

function [C, D] = nlmpcJacobianOutputModel(x,u)
    C = zeros(numJoints, numJoints * 2);
    C(1:numJoints, 1:numJoints) = eye(numJoints);
    D = zeros(numJoints, numJoints);
end

function cost =  nlmpcCostFunction(X,U,e,data, poseFinal, robot, endEffector, Qr, Qt, Qu, Qv)  
    % Running Cost
    costRunning = 0;
    for i= 2:p+1
        jointTemp = X(i,1:numJoints);
        taskTemp = getTransform(robot, jointTemp', endEffector);
        anglesTemp = rotm2eul(taskTemp(1:3,1:3), 'XYZ');
        poseTemp =  [taskTemp(1:3,4);anglesTemp'];
        diffRunning = [poseFinal(1:3)-poseTemp(1:3); angdiff(poseTemp(4:6),poseFinal(4:6))];
        costRunningTemp = diffRunning' * Qr * diffRunning;
        costRunning = costRunning + costRunningTemp + U(i,:)*Qu*U(i,:)';
    end
    
    % Terminal cost
    costTerminal = diffRunning'* Qt * diffRunning + X(p+1,numJoints+1:end)*Qv*X(p+1,numJoints+1:end)';

    % Total Cost
    cost = costRunning + costTerminal;
end

function [G,Gmv,Ge] = nlmpcJacobianCost(X,U,e,data, poseFinal, robot, endEffector, Qr, Qt, Qu, Qv)
    % Initialize Jacobians
    G = zeros(p,numJoints*2);
    Gmv = zeros(p,numJoints);
    Ge = 0;
    
    % Update G
    for i=1:p
        jointTemp = X(i+1,1:numJoints);
        taskTemp = getTransform(robot, jointTemp', endEffector);
        anglesTemp = rotm2eul(taskTemp(1:3,1:3), 'XYZ');
        poseTemp =  [taskTemp(1:3,4);anglesTemp'];
        diffRunning = [poseFinal(1:3)-poseTemp(1:3); angdiff(poseTemp(4:6),poseFinal(4:6))]; 
        
        % From geometric to analytical robot Jacobian
        rx = anglesTemp(1);
        py = anglesTemp(2);
        B = [ 1 0 sin(py); 0 cos(rx) -cos(py)*sin(rx); 0 sin(rx) cos(py)*cos(rx) ]; 
        
        % Robot Jacobian
        robotJacobianTemp = geometricJacobian(robot,jointTemp',endEffector);
        robotJacobian = robotJacobianTemp;
        robotJacobian(1:3,:) = robotJacobianTemp(4:6,:);
        robotJacobian(4:6,:) = B\robotJacobianTemp(1:3,:);
        
        % Running cost Jacobian
        G(i,1:numJoints) = (-2 * diffRunning' * Qr * robotJacobian); 
        Gmv(i,:) = 2 * U(i+1,:) * Qu;
    end

    % Terminal cost Jacobian
    G(p,1:numJoints) = G(p,1:numJoints) + (-2 * diffRunning' * Qt * robotJacobian);
    G(p,numJoints+1:end) = 2 * X(p+1,numJoints+1:end) * Qv;

end

function cineq = myIneqConFunction(X,U,e,data, safetyDistance, world, robot)
  % Copyright 2019 The MathWorks, Inc.
    allDistances = zeros(p*numBodies*numObstacles,1);
    for i =1:p
        collisionConfig = X(i+1,1:numJoints);
        [~, separationDist, ~] = checkCollision(robot, collisionConfig', world, 'IgnoreSelfCollision', 'On', 'Exhaustive', 'on');
        tempDistances = separationDist(1:robot.NumBodies,1:numObstacles);
        tempDistances(all(isinf(tempDistances)|isnan(tempDistances),2),:) = []; % remove inf and nans
        tempDistances(isinf(tempDistances)|isnan(tempDistances)) = 0;
        allDistances((1+(i-1)*numBodies*numObstacles):numBodies*numObstacles*i,1) = reshape(tempDistances', [numBodies*numObstacles,1]);   
    end
    cineq = -allDistances + safetyDistance;
end


function [G,Gmv,Ge] = nlmpcJacobianConstraint(X,U,e,data, world, robot)

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
                bodyNow = bodyIndices(j);
                wtnPts = allWntPts(1+(bodyNow-1)*3:3+(bodyNow-1)*3, 1+(k-1)*2:2+(k-1)*2);
                if isempty(wtnPts(isinf(wtnPts)|isnan(wtnPts)))
                    if any((wtnPts(:,1)-wtnPts(:,2))~=0) 
                        normal = (wtnPts(:,1)-wtnPts(:,2))/norm(wtnPts(:,1)-wtnPts(:,2));
                    else
                        normal = [0;0;0];
                    end
                    bodyJacobian = geometricJacobian(robot,collisionConfig', robot.BodyNames{bodyNow});
                    G(i, 1:numJoints,  iter)= -normal' * bodyJacobian(4:6,:);
                else
                    G(i, 1:numJoints,  iter) = zeros(1, numJoints);
                end
                iter = iter + 1;                 
            end           
         end        
    end

end
end


