% Copyright 2020 The MathWorks, Inc.

% Query poses of the moving obstacles at time t = time
aPoseObsNow = interp1(timeObs,aPosesObs,time);
aMovObs.Pose = trvec2tform(aPoseObsNow);
bPoseObsNow = interp1(timeObs,bPosesObs,time);
bMovObs.Pose = trvec2tform(bPoseObsNow);
world = {aMovObs, bMovObs};

% Update constraint functions
nlobj.Optimization.CustomIneqConFcn = ...
    @(X,U,e,data) nlmpcIneqConFunctionKINOVA(X,U,e,data,safetyDistance,world,robot);
nlobj.Jacobian.CustomIneqConFcn = ...
    @(X,U,e,data) nlmpcJacobianConstraintKINOVA(X,U,e,data,world,robot);
