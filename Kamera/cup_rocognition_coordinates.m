% Add the RealSense directory to the MATLAB path
addpath('./+realsense/');

% Initialize YOLOv4 object detector
yolov4 = yolov4ObjectDetector("csp-darknet53-coco");

% Make Pipeline object to manage streaming
pipe = realsense.pipeline();
profile = pipe.start();  % Start the pipeline here

% Create an align object to align depth frames to color frames
align_to = realsense.stream.color;
align = realsense.align(align_to);

% Extrahieren Sie die intrinsischen Parameter aus dem Farbstream
color_stream = profile.get_stream(realsense.stream.color).as('video_stream_profile');
intrinsics = color_stream.get_intrinsics();

while true
    % Get frames and align them
    fs = pipe.wait_for_frames();
    aligned_frames = align.process(fs);

    color_frame = aligned_frames.get_color_frame();
    depth_frame = aligned_frames.get_depth_frame();

    % Extract color and depth data
    color_data = color_frame.get_data();
    color_img = permute(reshape(color_data',[3,color_frame.get_width(),color_frame.get_height()]),[3 2 1]);

    % Detect the cup and get its center location
    cupCenter = detect_cup(color_img, yolov4);
    disp(cupCenter);

    % If a cup was detected, find its depth and calculate world coordinates
    if ~isempty(cupCenter)
        cupDepth = detect_depth(depth_frame, cupCenter);
        disp(['Depth of the cup: ', num2str(cupDepth), ' meters']);

        % Calculate world coordinates
        [worldX, worldY, worldZ] = depth_to_world(intrinsics, cupCenter(1), cupCenter(2), cupDepth);
        disp(['World coordinates of the cup: X=', num2str(worldX), ', Y=', num2str(worldY), ', Z=', num2str(worldZ)]);

        % Annotate and display the image
        imshow(color_img)
        hold on;
        plot(cupCenter(1), cupCenter(2), 'r+', 'MarkerSize', 10, 'LineWidth', 2);
        hold off;
    end
end

function cupCenter = detect_cup(img, yolov4)
    cupCenter = [];

    % Perform object detection
    detections = detect(yolov4, img);

    % Check if any cups were detected
    if ~isempty(detections)
        % Assuming detections are [x, y, width, height]
        % Calculate the center of the first detected cup
        xCenter = detections(1, 1) + detections(1, 3) / 2;
        yCenter = detections(1, 2) + detections(1, 4) / 2;
        cupCenter = [xCenter, yCenter];
    end
end

function cupDepth = detect_depth(depth, cupLocation)
    % Get the distance at the specific point
    cupDepth = depth.get_distance(cupLocation(1), cupLocation(2));
end

function [worldX, worldY, worldZ] = depth_to_world(intrinsics, pixelX, pixelY, depth)
    % Umrechnung der Pixelkoordinaten in Weltkoordinaten
    fx = intrinsics.fx;  % Brennweite in x-Richtung
    fy = intrinsics.fy;  % Brennweite in y-Richtung
    cx = intrinsics.ppx; % Hauptpunkt x-Koordinate
    cy = intrinsics.ppy; % Hauptpunkt y-Koordinate

    % Umrechnung unter Verwendung der intrinsischen Kameraparameter
    worldX = (pixelX - cx) * depth / fx;
    worldY = (pixelY - cy) * depth / fy;
    worldZ = depth;
end
