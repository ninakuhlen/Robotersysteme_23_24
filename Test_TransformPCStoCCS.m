clear all;

% load relevant parameters from config file
config = jsondecode(fileread('./config.json'));

% add path to camera
addpath(config.paths.camera_directory);

% load image for calculation of transformation matrix from pcs to rcs
image = imread(config.paths.calibration_image);

% get camera parameters
load(config.paths.camera_parameters);

% calculate transformation from pcs to ccs
transform = TransformPCStoCCS(image, cameraParams, config.parameters.edge_length);


%% visualization

% show image
imshow(image);
hold on;

% draw all coordinate systems
drawPCS(100);
drawCCS(transform.cameraParameters.PrincipalPoint, 100);
drawBCS(transform.pointsPCS, transform.dimensions);

% draw points of intercestion of the checkerboard
drawImagePoints(transform.pointsPCS);

% highlight the origin and point 6 with green circles.
highlightPoint(transform.pointsPCS, 1);
highlightPoint(transform.pointsPCS, 6);

% measure the real world distances along the x and y axis of the ccs and
% visualize them
measurePointAndPlot(transform.pointsPCS, 1, transform.cameraParameters.PrincipalPoint, transform.extrinsics, transform.intrinsics, 4, [225, 193, 110], -1);
measurePointAndPlot(transform.pointsPCS, 6, transform.cameraParameters.PrincipalPoint, transform.extrinsics, transform.intrinsics, 8, [93, 63, 211], 1);

hold off;

%% visualization function definitions

function drawPCS(axisLength)

offset = 10;

framePoints = [[offset offset]; [offset offset]; [offset offset]];
framePoints(1,1) = framePoints(1,1) + axisLength;
framePoints(3,2) = framePoints(3,2) + axisLength;

% draw the x axis in red
line(framePoints(1:2,1), framePoints(1:2,2), 'Color', 'r', 'LineWidth', 2);
% draw the y axis in green
line(framePoints(2:3,1), framePoints(2:3,2), 'Color', 'g', 'LineWidth', 2);
% write the frame name aside the axes
text(framePoints(2,1) + offset + 5, framePoints(2,2) + offset + 5, 'PCS', 'Color', 'b', 'FontSize', 8, 'BackgroundColor', [1, 1, 1]);
end

function drawCCS(principalPointPCS, axisLength)

offset = 5;

% construct the three points to draw a 2d coordinate frame
framePoints = [principalPointPCS; principalPointPCS; principalPointPCS];
framePoints(1,1) = framePoints(1,1) - axisLength;
framePoints(3,2) = framePoints(3,2) - axisLength;

% draw the x axis in red
line(framePoints(1:2,1), framePoints(1:2,2), 'Color', 'r', 'LineWidth', 2);
% draw the y axis in green
line(framePoints(2:3,1), framePoints(2:3,2), 'Color', 'g', 'LineWidth', 2);
% write the frame name aside the axes
text(framePoints(2,1) + offset, framePoints(2,2) + offset + 5, 'CCS', 'Color', 'b', 'FontSize', 8, 'BackgroundColor', [1, 1, 1]);
end

function drawBCS(pointsPCS, checkerboardSize)

offset = 10;

% construct the three points to draw a 2d coordinate frame
framePoints = [pointsPCS(checkerboardSize(2),:); pointsPCS(1,:); pointsPCS(2,:)];

% draw the x axis in red
line(framePoints(1:2,1), framePoints(1:2,2), 'Color', 'r', 'LineWidth', 2);
% draw the y axis in green
line(framePoints(2:3,1), framePoints(2:3,2), 'Color', 'g', 'LineWidth', 2);
% write the frame name aside the axes
text(framePoints(2,1) + offset, framePoints(2,2) - offset, 'BCS', 'Color', 'y', 'FontSize', 8);
end

function drawImagePoints(pointsPCS)

offset = 5;

% draw circles around the points of intersection
plot(pointsPCS(:, 1), pointsPCS(:, 2), 'ro');

% label these circles with their index
for i = 1:size(pointsPCS, 1)
    text(pointsPCS(i, 1) + offset, pointsPCS(i, 2), num2str(i), 'Color', 'y', 'FontSize', 8);
end
end

function highlightPoint(imagePoints, index)
plot(imagePoints(index, 1), imagePoints(index, 2), 'go', 'MarkerSize', 10);
end

function measurePointAndPlot(pointsPCS, index, principalPointPCS, cameraExtrinsics, cameraIntrinsics, scetchOffset, color, dir)

imagePoint = pointsPCS(index,:);
principalPointPCS = double(principalPointPCS);

% transform the principal point to the ccs
principalWorldPoint = img2world2d(principalPointPCS, cameraExtrinsics, cameraIntrinsics);

% transform the chosen point to the ccs
imageWorldPoint = img2world2d(imagePoint, cameraExtrinsics, cameraIntrinsics);

% get both the pixel and real world distances between principal point and
% the selected image point
pixelDistance = imagePoint - principalPointPCS ;
worldDistance = imageWorldPoint - principalWorldPoint;

% manage the offset for visualization ...
offprincipalPointPCS = principalPointPCS + [scetchOffset -scetchOffset];
xPoint = offprincipalPointPCS + [pixelDistance(1)+scetchOffset 0];
yPoint = offprincipalPointPCS + [0 pixelDistance(2)-scetchOffset];

gain = 3;
% ... and merge the frame points
framePoints = [xPoint; offprincipalPointPCS; yPoint];

% plot the distances and labels in a random color
color = color ./ 255;
plot(framePoints(:,1), framePoints(:,2), 'Color', color, 'LineWidth', 2);
text(framePoints(1,1) + gain*abs(scetchOffset), framePoints(1,2) + dir * 2* gain*4, ['P', num2str(index), 'X: ', num2str(int64(worldDistance(1))), ' mm'], 'Color', color, 'FontSize', 15, 'BackgroundColor', [1, 1, 1], 'HorizontalAlignment','right');
text(framePoints(3,1) + gain*abs(scetchOffset), framePoints(3,2) - gain*2*4, ['P', num2str(index), 'Y: ', num2str(int64(worldDistance(2))), ' mm'], 'Color', color, 'FontSize', 15, 'BackgroundColor', [1, 1, 1]);
end
