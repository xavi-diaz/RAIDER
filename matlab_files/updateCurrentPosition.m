function updateCurrentPosition()
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

global currentPosition;
global targetPosition;

for idx=1:20    
    currentPosition(idx) = targetPosition(idx);
end  
    
end

