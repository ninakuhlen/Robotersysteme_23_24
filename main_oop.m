clear all;

ENABLE_TCPIP = true;
config = jsondecode(fileread('./config.json'));

addpath(config.paths.camera_directory);
addpath(config.paths.communication_directory);
% addpath('.\robot');

% initialize devices and functionalites
device = RobotUR3();
ik = InverseKinematic();
camera = Camera3D(config.paths.camera_parameters);
detector = CupDetector(config.parameters.cup_detection_type);

% load image for calculation of transformation matrix from pcs to rcs
image = imread(config.paths.calibration_image);

% calculate transformation from pcs to rcs
transform = TransformPCStoCCS(image, camera.parameters, config.parameters.edge_length);
transform.extendToRCS(config.parameters.board_origin_rcs, config.parameters.board_corner_rcs);

disp("Setup Ready, waiting for commands from Master")

if ENABLE_TCPIP
    waitfor(device, "state", 1);
end
disp("Step 1 [Cup detection]");

% \\\\\\\\\\\\\\\\\\\\ 1 //////////////////// %
% detect cup and get its location

% take a snapshot
cupCenterPointPCS = camera.stream(@detector.detectCup);

% % receive cup location from detector
cupCenterPointPCS = cupCenterPointPCS(1:2);

% apply full transformation on the testing set
cupCenterPointRCS = transform.apply(cupCenterPointPCS);

% remove augmentation
cupCenterPointRCS = cupCenterPointRCS(1:2);

% add height to cup coordinates
cupCenterPointRCS(3) = config.parameters.cup_z_coordinate;
cupCenterPointRCS = cupCenterPointRCS / 1000;

% return success
device.answerMaster();
disp("Step 1 complete - Send confirmation of success to Master");

if ENABLE_TCPIP
    waitfor(device, "state", 2);
end
disp("Step 2 [Grip cup and move to pouring position]");

% \\\\\\\\\\\\\\\\\\\\ 2 //////////////////// %
% move to cup location and grap cup

q = device.HOME;
q(3) = deg2rad(90);

% go to approach position
device.moveJ(q);

% q(5) = q(5) - deg2rad(90);
% device.moveJ(q);

orientationA = [0.0, pi/2, 0.0]; % Rotationsvektor
ik.get_thetas(cupCenterPointRCS, orientationA);
q = ik.currentThetas;
device.moveL(q);

% wait for user to ensure an opened gripper before resuming
popupWindow = msgbox("Open Gripper and press OK to continue", "icon", "warn");
waitfor(popupWindow);

gripPos = cupCenterPointRCS;
gripPos(3) = config.parameters.final_grip_height;
ik.get_thetas(gripPos, orientationA)
q = ik.currentThetas;
device.moveL(q)
% device.moveL([cupCenterPointRCS(1), cupCenterPointRCS(2), cupCenterPointRCS(3), testRot(1), testRot(2), testRot(3)])

% wait for user to ensure a closed gripper before resuming
popupWindow = msgbox("Close Gripper and press OK to continue", "icon", "warn");
waitfor(popupWindow);

% move robot to position above located cup
gripPos = cupCenterPointRCS;
gripPos(3) = config.parameters.lift_grip_height;
ik.get_thetas(gripPos, orientationA);
q = ik.currentThetas;
device.moveL(q);

% device.answerMaster();
% disp("Step 2 complete - Send confirmation of success to Master");
% 
% if ENABLE_TCPIP
%     waitfor(device, "state", 3);
% end
% disp("Step 3 [Move cup to pouring position]");

% \\\\\\\\\\\\\\\\\\\\ 3 //////////////////// %
% move cup to pouring position

% move robot end effector to known distance to axis 1
knownRadiusPos = [0.553, 0, 0.24];
ik.get_thetas(knownRadiusPos, orientationA);
q = ik.currentThetas;
device.moveL(q);

% move to filling position
q1_old = q(1);
q(1) = deg2rad(94);
device.moveJ(q);

% device.answerMaster();
% disp("Step 3 complete - Send confirmation of success to Master");
% 
% if ENABLE_TCPIP
%     waitfor(device, "state", 4);
% end
% disp("Step 4 [Move cup to drop off point]");

device.answerMaster();
disp("Step 2 complete - Send confirmation of success to Master");

if ENABLE_TCPIP
    waitfor(device, "state", 3);
end
disp("Step 3 [Start pouring]");

device.answerMaster();

disp("Step 3 complete - Send confirmation of success to Master");

if ENABLE_TCPIP
    waitfor(device, "state", 4);
end
disp("Step 4 [Move cup to drop off point]");

% \\\\\\\\\\\\\\\\\\\\ 4 //////////////////// %
% move robot to position above cup drop off point

% set lower velocity for testing purposes
% device.v = 0.1;

% move robot to position above cup drop off point
gripPos = cupCenterPointRCS;
gripPos(3) = config.parameters.lift_grip_height;
ik.get_thetas(gripPos, orientationA);
q = ik.currentThetas;
device.moveL(q);

% move robot to position above cup drop off point
gripPos = cupCenterPointRCS;
gripPos(3) = config.parameters.final_grip_height;
ik.get_thetas(gripPos, orientationA);
q = ik.currentThetas;
device.moveL(q);

popupWindow = msgbox("Open Gripper and press OK to continue", "icon", "warn");
waitfor(popupWindow);

% move robot to position above cup drop off point
gripPos = cupCenterPointRCS;
gripPos(3) = config.parameters.lift_grip_height;
ik.get_thetas(gripPos, orientationA)
q = ik.currentThetas;
device.moveL(q);

% go to Home position
q_home = device.HOME;
device.moveJ(q_home);

device.answerMaster();
disp("Step 4 complete - Send confirmation of success to Master");
disp("Process finished")

% move to filling position
% joint_angles = [103.54, -81.39, 97.06, -15.61, 88.02, 269.92]
% orientationB = [1.01, 1.321, -0.997];
% % RPY = [1.635, 1.561, 0.339];
% ik.get_thetas([0.15, -0.45, 0.25], [1.01, 1.321, -0.997])
% q = ik.currentThetas;
% device.moveL(q)
% device.moveL([150.0, -450, 250, 1.01, 1.321, -0.997])