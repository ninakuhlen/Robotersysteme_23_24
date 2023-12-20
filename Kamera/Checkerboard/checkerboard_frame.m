% % Read the image of the checkerboard
% image = imread('C:\Users\ninak\Documents\Westfälische Hochschule\Master Robotik\23_24_WS\Robotersysteme\repository\Kamera\Checkerboard\Images\lab_checkerboard_Color.png');
%
% % Detect the checkerboard corners in the image
% [imagePoints, ~] = detectCheckerboardPoints(image);
%
% % Calculate the center of the image
% [imageHeight, imageWidth, ~] = size(image);
% centerX = imageWidth / 2;
% centerY = imageHeight / 2;
%
% % Define the checkerboard origin (point 1 in the imagePoints)
% checkerboardOrigin = imagePoints(1, :);
%
% % Calculate the distances from the center of the image to the checkerboard origin
% distanceX = checkerboardOrigin(1) - centerX;
% distanceY = checkerboardOrigin(2) - centerY;
%
% % Display the distances
% fprintf('Distance from the center of the image to the checkerboard origin:\n');
% fprintf('X-direction: %f pixels\n', distanceX);
% fprintf('Y-direction: %f pixels\n', distanceY);

%________________________________________________
clear all

% Load camera parameters
load('../../Camera_Kalibrierung\Cam_Param_1.mat'); % Load camera parameters

% Read the image of the checkerboard
image = imread('C:\Users\ninak\Documents\Westfälische Hochschule\Master Robotik\23_24_WS\Robotersysteme\repository\Kamera\Checkerboard\Images\04_lab_checkerboard_Color.png');

%__________________________________________________


% Calculate the center of the image. This is used as a reference point for measurements.
[imageHeight, imageWidth, ~] = size(image);
centerX = imageWidth / 2;
centerY = imageHeight / 2;

% Detect the corners of the checkerboard in the image.
% These points are used for calculating measurements and drawing axes.
[imagePoints, boardSize] = detectCheckerboardPoints(image);

% Define the physical dimensions of the checkerboard squares.
% Generate world coordinates for the checkerboard corners based on these dimensions.
squareSize = 30; % Square size in millimeters.
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Estimate the pose of the checkerboard relative to the camera.
% This includes the rotation matrix and translation vector of the checkerboard.
[rotationMatrix, translationVector] = extrinsics(imagePoints, worldPoints, cameraParams);

% Define specific points on the checkerboard to be used for drawing axes.
% 'origin' is the starting point of the axes.
origin = imagePoints(1, :); % Using the first detected corner as the origin.
xAxisPoint = imagePoints(37, :); % Point 37 for the x-axis.
yAxisPoint = imagePoints(6, :); % Point 6 for the y-axis.

% Calculate the world coordinates of the image center.
% This is used as a reference point for distance measurements.
centerImagePoint = [centerX, centerY];
centerWorldPoint = pointsToWorld(cameraParams, rotationMatrix, translationVector, centerImagePoint);

% Calculate the distance from the image center to the checkerboard origin in millimeters.
cornerImagePoint = imagePoints(1, :);
cornerWorldPoint = pointsToWorld(cameraParams, rotationMatrix, translationVector, cornerImagePoint);
distanceX_mm_origin = abs(cornerWorldPoint(1) - centerWorldPoint(1));
distanceY_mm_origin = abs(cornerWorldPoint(2) - centerWorldPoint(2));

% Calculate the distance from the image center to checkerboard point 6 in millimeters.
point6Image = imagePoints(6, :);
point6World = pointsToWorld(cameraParams, rotationMatrix, translationVector, point6Image);
distanceX_mm_point6 = abs(point6World(1) - centerWorldPoint(1));
distanceY_mm_point6 = abs(point6World(2) - centerWorldPoint(2));

% Display the image and plot the detected corners and axes.
imshow(image);
hold on;

% Offset value to ensure measurement lines are distinct and do not overlap.
offset = 2; % Offset for measurement lines  in pixels.


% Plot and label all detected corners.
for i = 1:size(imagePoints, 1)
    plot(imagePoints(i, 1), imagePoints(i, 2), 'ro'); % Plot corner as a red circle.
    text(imagePoints(i, 1) + 5, imagePoints(i, 2), num2str(i), 'Color', 'yellow', 'FontSize', 8); % Label the corner.
end

% Highlight the origin and point 6 with green circles.
plot(origin(1), origin(2), 'go', 'MarkerSize', 10);
plot(yAxisPoint(1), yAxisPoint(2), 'go', 'MarkerSize', 10);

% Draw axes from the origin to specified points.
line([origin(1), xAxisPoint(1)], [origin(2), xAxisPoint(2)], 'Color', 'm', 'LineWidth', 2); % x-axis in magenta.
line([origin(1), yAxisPoint(1)], [origin(2), yAxisPoint(2)], 'Color', 'c', 'LineWidth', 2); % y-axis in cyan.

% Draw a crosshair at the center of the image.
crosshairSize = 20; % Size of the crosshair in pixels.
line([centerX - crosshairSize, centerX + crosshairSize], [centerY, centerY], 'Color', 'b', 'LineWidth', 2); % Horizontal line.
line([centerX, centerX], [centerY - crosshairSize, centerY + crosshairSize], 'Color', 'b', 'LineWidth', 2); % Vertical line.

% Draw and annotate measurement lines from the image center to the checkerboard origin.
line([centerX, origin(1)], [centerY - offset, centerY - offset], 'Color', 'g', 'LineWidth', 2); % X-direction line to origin, moved down
line([centerX - offset, centerX - offset], [centerY, origin(2)], 'Color', 'g', 'LineWidth', 2); % Y-direction line to origin, moved left
text((centerX + origin(1))/2, centerY - offset - 15, ['P1: ' num2str(round(distanceX_mm_origin)) ' mm'], 'Color', 'yellow', 'FontSize', 15); % X-distance annotation
text(centerX - offset, (centerY + origin(2))/2, ['P1: ' num2str(round(distanceY_mm_origin)) ' mm'], 'Color', 'yellow', 'FontSize', 15); % Y-distance annotation

% Draw and annotate measurement lines from the image center to checkerboard point 6.
line([centerX, yAxisPoint(1)], [centerY + offset, centerY + offset], [centerY, centerY], 'Color', 'r', 'LineWidth', 2); % X-direction line to point 6, moved up
line([centerX + offset, centerX + offset], [centerY, yAxisPoint(2)], 'Color', 'r', 'LineWidth', 2); % Y-direction line to point 6, moved right
text((centerX + yAxisPoint(1))/2, centerY + offset + 15, ['P6: ' num2str(round(distanceX_mm_point6)) ' mm'], 'Color', 'yellow', 'FontSize', 15); % X-distance annotation to point 6
text(centerX + offset, (centerY + yAxisPoint(2))/2, ['P6: ' num2str(round(distanceY_mm_point6)) ' mm'], 'Color', 'yellow', 'FontSize', 15); % Y-distance annotation to point 6

hold off;

