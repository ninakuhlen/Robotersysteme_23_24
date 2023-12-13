function get_camera_tm()

pipe = realsense.pipeline();

profile = pipe.start();

for i = 1:5
    fs = pipe.wait_for_frames();
end

while true
    fs = pipe.wait_for_frames();

    color = fs.get_color_frame();

    colorData = color.get_data();

    colorImage = permute(reshape(colorData',  [3, color.get_width(), color.get_height()]), [3 2 1]);

%     imshow(colorImage)

    find_checherboard(colorImage)

end

pipe.stop()

end

function find_checherboard(image)

load('C:\Users\William\Documents\GitHub\Robotersysteme_23_24\Kamera\cam_params_nina_medium_resolution.mat') 
% -----> insert camera parameters (from calibration) here <----
% Die Helligkeit der Kamera ist hoch einzustellen!
% Display the variables in the loaded file
whos

% Abstand Roboterbasis blaues Kreuz: in x-Richtung 675 und in y-Richtung 60 mm

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

disp(size(worldPoints))

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
% hold on;
% plot(imagePoints(:, 1), imagePoints(:, 2), 'ro'); % Plot all detected corners as red circles
% plot(imagePoints(cornerIndex, 1), imagePoints(cornerIndex, 2), 'go', 'MarkerSize', 10); % Highlight the selected corner with a green circle
% 
% % Draw the camera coordinate system axes
% % Axis length in pixels (you may need to adjust this based on your image)
% axisLength = 50;
% 
% % Draw the X-axis in red
% line([imagePoints(cornerIndex, 1), imagePoints(cornerIndex, 1) + axisLength], [imagePoints(cornerIndex, 2), imagePoints(cornerIndex, 2)], 'Color', 'r', 'LineWidth', 2);
% 
% % Draw the Y-axis in green
% line([imagePoints(cornerIndex, 1), imagePoints(cornerIndex, 1)], [imagePoints(cornerIndex, 2), imagePoints(cornerIndex, 2) - axisLength], 'Color', 'g', 'LineWidth', 2);
% 
% % Draw a crosshair at the center of the image
% % Draw a crosshair at the center of the image
% crosshairSize = 20; % size of the crosshair in pixels
% line([imageCenterX - crosshairSize, imageCenterX + crosshairSize], [imageCenterY, imageCenterY], 'Color', 'b', 'LineWidth', 2); % Horizontal line
% line([imageCenterX, imageCenterX], [imageCenterY - crosshairSize, imageCenterY + crosshairSize], 'Color', 'b', 'LineWidth', 2); % Vertical line
% hold off;
end
