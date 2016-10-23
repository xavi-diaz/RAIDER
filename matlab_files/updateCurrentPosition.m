function updateCurrentPosition()
% copies the targetPosition(20) array values into the currentPosition(20) array when a motion has finished


global currentPosition;
global targetPosition;

for idx=1:20    
    currentPosition(idx) = targetPosition(idx);
end  
    
end

