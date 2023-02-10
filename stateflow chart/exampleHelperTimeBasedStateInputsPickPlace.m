function stateDot = exampleHelperTimeBasedStateInputsPickPlace(obj, timeInterval, jointStates, t, state)
% This function is for internal use and may be removed in a future release
%
%exampleHelperTimeBasedStateInputsPickPlace Wrapper function for simulating joint-space time-based tracking with an ODE solver
%   This function is used to pass time-based inputs to the derivative
%   method of jointSpaceMotionModel so that it can be used in an ODE
%   Solver. ODE Solvers require a function handle that takes time as input
%   and output the state derivative. However, in the trajectory tracking
%   case, the state derivative is dependent on the interpolated value of
%   the reference trajectory at that instant in time. As a result, this
%   helper function first computes the target state at the instant in time
%   given by t using interpolation, then passes that to the derivative of
%   the provided jointSpaceMotionModel object, which computes the state
%   derivative.

% Copyright 2020 The MathWorks, Inc.

    % Compute the interpolated target state given the linear trajectory
    % between the first and last values of jointStates over the specified
    % time interval at the current time, t
    targetState = interp1(timeInterval, jointStates, t);
    
    % Compute state derivative
    stateDot = derivative(obj, state, targetState);
end
