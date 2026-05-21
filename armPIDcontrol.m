function [armTc] = armPIDcontrol(theta, thetaGuidance, thetaDot, thetaDotGuidance, armGainVals)

Kp = -armGainVals(1);
Kd = -armGainVals(2);

speedError = thetaDot - thetaDotGuidance; 
speedTuned = Kd * speedError;

posError = theta - thetaGuidance;
posTuned = Kp * posError;

armTc = speedTuned + posTuned; 

end