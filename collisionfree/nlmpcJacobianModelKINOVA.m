function [A, B] = nlmpcJacobianModelKINOVA(x,u)

    % Copyright 2020 The MathWorks, Inc.

    numJoints = length(u);
    A = zeros(numJoints*2, numJoints * 2);
    
    A(1:numJoints, numJoints+1:end) = eye(numJoints);
    B = zeros(numJoints*2,numJoints);
    B(numJoints+1:end,:)=eye(numJoints); 
end