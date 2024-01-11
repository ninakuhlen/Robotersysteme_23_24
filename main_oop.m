


clear all;

config = jsondecode(fileread('./config.json'));

addpath(config.paths.camera_directory);
addpath(config.paths.communication_directory);
% addpath('.\robot');

device = RobotUR3()
ik = InverseKinematic()
camera = Camera3D(config.paths.camera_parameters)
detector = CupDetector(config.parameters.cup_detection_type)

waitfor(device, "state", 1);

disp("Step 1");

waitfor(device, "state", 2);

disp("Step 2");

waitfor(device, "state", 3);

disp("Step 3");

waitfor(device, "state", 4);

disp("Step 4");

% \\\\\\\\\\\\\\\\\\\\ 1 //////////////////// %
% calculate transformation matrix

% take a snapshot
image = imread(config.paths.calibration_image);

% calculate transformation from pcs to rcs
transform = TransformPCStoCCS(image, camera.parameters, config.parameters.edge_length);
transform.extendToRCS(config.parameters.board_origin_rcs, config.parameters.board_corner_rcs);


% \\\\\\\\\\\\\\\\\\\\ 2 //////////////////// %
% detect cup and get its location

% take a snapshot
cupCenterPointPCS = camera.stream(@detector.detectCup);

% % receive cup location from detector
cupCenterPointPCS = cupCenterPointPCS(1:2);

% apply full transformation on the testing set
cupCenterPointRCS = transform.apply(cupCenterPointPCS);

% remove augmentation
cupCenterPointRCS = cupCenterPointRCS(1:2)

% add height to cup coordinates
% ACHTUNG KORREKTURWERT
cupCenterPointRCS(1) = cupCenterPointRCS(1) + config.parameters.cup_x_correction;
cupCenterPointRCS(2) = cupCenterPointRCS(2) + config.parameters.cup_y_correction;
cupCenterPointRCS(3) = config.parameters.cup_z_coordinate;

cupCenterPointRCS = cupCenterPointRCS / 1000;

% \\\\\\\\\\\\\\\\\\\\ 3 //////////////////// %
% move to cup location and grap cup

q = [0.0 -pi/2 -pi/2 -pi/2 -pi/2 0.0];
device.moveJ(q);
device.closeGripper();
device.openGripper();

testRot = [0.0, pi/2, 0.0]; % Rotationsvektor
ik.get_thetas(cupCenterPointRCS, testRot);
q = ik.currentThetas
device.moveL(q)

pause

gripPos = cupCenterPointRCS;
gripPos(3) = config.parameters.final_grip_height;
ik.get_thetas(gripPos, testRot)
q = ik.currentThetas
device.moveL(q)
% device.moveL([cupCenterPointRCS(1), cupCenterPointRCS(2), cupCenterPointRCS(3), testRot(1), testRot(2), testRot(3)])

pause

gripPos = cupCenterPointRCS;
gripPos(3) = config.parameters.lift_grip_height;
ik.get_thetas(gripPos, testRot)
q = ik.currentThetas
device.moveL(q)