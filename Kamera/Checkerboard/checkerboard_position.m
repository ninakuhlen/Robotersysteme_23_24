% Load camera parameters (replace 'cameraParams.mat' with your file)
% load('../../../Camera_Kalibrierung/Cam_Calib_Nina_HR.mat');
load('../../Camera_Kalibrierung\cam_params_nina_medium_resolution.mat') %-----> insert camera parameters (from calibration) here <----

% Display the variables in the loaded file
whos


% Read the image of the checkerboard
%image = imread('./Images/checkerboard_Color.png'); % replace with your image file -----> insert file of checkerboard image <----
image = imread('.\Images\checkerboard_Color.png');

% Berechnen Sie das Zentrum des Bildes
[imageHeight, imageWidth, ~] = size(image);
imageCenterX = imageWidth / 2;
imageCenterY = imageHeight / 2;

% Detect the checkerboard corners in the image
[imagePoints, boardSize] = detectCheckerboardPoints(image);

% Define the world coordinates of checkerboard corners
% This depends on the size of your checkerboard and the dimensions of the squares
squareSize = 30; % square size in millimeters
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Estimate the pose of the checkerboard
[rotationMatrix, translationVector] = extrinsics(imagePoints, worldPoints, cameraParams);

% Select a corner point to find its position (e.g., first corner)
cornerIndex = 1;
cornerWorldPoint = worldPoints(cornerIndex, :);

% Transform the corner point from world to camera coordinates
% Reshape cornerWorldPoint to be a 3x1 column vector
cornerWorldPoint = [cornerWorldPoint, 0]'; % Add a zero for the Z-coordinate
cornerCameraPoint = (rotationMatrix * cornerWorldPoint) + translationVector';

% Display the result
fprintf('Position of the corner in camera coordinates (in mm):\n X: %f (red)\n Y: %f (green)\n Z: %f\n', cornerCameraPoint);

% Visualize the detected corners on the image
imshow(image);
hold on;
plot(imagePoints(:, 1), imagePoints(:, 2), 'ro'); % Plot all detected corners as red circles
plot(imagePoints(cornerIndex, 1), imagePoints(cornerIndex, 2), 'go', 'MarkerSize', 10); % Highlight the selected corner with a green circle

% Draw the camera coordinate system axes
% Axis length in pixels (you may need to adjust this based on your image)
axisLength = 50;

% Draw the X-axis in red
line([imagePoints(cornerIndex, 1), imagePoints(cornerIndex, 1) + axisLength], [imagePoints(cornerIndex, 2), imagePoints(cornerIndex, 2)], 'Color', 'r', 'LineWidth', 2);

% Draw the Y-axis in green
line([imagePoints(cornerIndex, 1), imagePoints(cornerIndex, 1)], [imagePoints(cornerIndex, 2), imagePoints(cornerIndex, 2) - axisLength], 'Color', 'g', 'LineWidth', 2);

% Draw a crosshair at the center of the image
% Draw a crosshair at the center of the image
crosshairSize = 20; % size of the crosshair in pixels
line([imageCenterX - crosshairSize, imageCenterX + crosshairSize], [imageCenterY, imageCenterY], 'Color', 'b', 'LineWidth', 2); % Horizontal line
line([imageCenterX, imageCenterX], [imageCenterY - crosshairSize, imageCenterY + crosshairSize], 'Color', 'b', 'LineWidth', 2); % Vertical line

hold off;
