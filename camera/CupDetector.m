classdef CupDetector < handle
    properties
        % object detector
        detector
        detectionMode
    end

    methods
        function object = CupDetector(detectionMode)
            object.detectionMode = detectionMode;

            if detectionMode == "ai"
                % Initialize YOLOv4 object detector
                object.detector = yolov4ObjectDetector("csp-darknet53-coco");
            end
        end % CupDetector

        function [cupCenterPoint, terminate] = detectCup(self, colorImage, depthData)

            if strcmp(self.detectionMode, "ai")
                % perform object detection
                detections = detect(self.detector, colorImage);

                % check if any cups were detected
                if ~isempty(detections)
                    disp('Cup detected!')
                    % assuming detections are [x, y, width, height]

                    imshow(colorImage);

                    % calculate the center of the first detected cup
                    xCenter = detections(1, 1) + detections(1, 3) / 2;
                    yCenter = detections(1, 2) + detections(1, 4) / 2;
                    cupCenterPoint = [xCenter, yCenter];
                    cupCenterPoint(3) = depthData.get_distance(xCenter, yCenter);

                    terminate = true;
                else
                    disp('No Cup detected!')
                    cupCenterPoint = [0 0 0];
                    terminate = false;
                end
            elseif strcmp(self.detectionMode, "analytical")
                image = rgb2gray(colorImage);
                image = imbinarize(image, "global");
                image = imcomplement(image);

                [centers, radii] = imfindcircles(image,[50 100],"Sensitivity", 0.95);
                if ~isempty(centers)
                    imshow(colorImage);
                    viscircles(centers, radii,'EdgeColor','b');
                    cupCenterPoint = centers(1,:);
                    cupCenterPoint(3) = depthData.get_distance(cupCenterPoint(1), cupCenterPoint(2));
                    disp("Cup detected")
                    terminate = true;
                else
                    disp('No Cup detected!')
                    cupCenterPoint = [0 0 0];
                    terminate = false;
                end
            end % if self.detectionMode
        end % detectCup
    end % methods
end