clear all;

% initialize devices and functionalites

ik = InverseKinematic();

cupCenterPointRCS = [-0.350, 0.2, 0.200];
orientationA = [pi, pi/2, 0.0]; % Rotationsvektor
ik.get_thetas(cupCenterPointRCS, orientationA);
