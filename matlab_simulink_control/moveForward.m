function moveForward(left_cmd, right_cmd)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

global targetPosition;

setTargetPosition(targetPosition(1),targetPosition(2),targetPosition(3),targetPosition(4),targetPosition(5),targetPosition(6),targetPosition(7),targetPosition(8),targetPosition(9),targetPosition(10),targetPosition(11),targetPosition(12),targetPosition(13)+right_cmd, targetPosition(14)+left_cmd,targetPosition(15),targetPosition(16), targetPosition(17)-right_cmd, targetPosition(18)-left_cmd,targetPosition(19),targetPosition(20)); 


end

