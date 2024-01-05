classdef CupDetector < handle
    properties
        % object detector
        detector
    end

    methods
        function object = CupDetector()
            % Initialize YOLOv4 object detector
            object.detector = yolov4ObjectDetector("csp-darknet53-coco");
        end

        function [cupCenterPoint, terminate] = detectCup(self, colorImage, depthData)

            % perform object detection
            detections = detect(self.detector, colorImage);

            % check if any cups were detected
            if ~isempty(detections)
                % assuming detections are [x, y, width, height]

                % calculate the center of the first detected cup
                xCenter = detections(1, 1) + detections(1, 3) / 2;
                yCenter = detections(1, 2) + detections(1, 4) / 2;
                cupCenterPoint = [xCenter, yCenter];
                cupCenterPoint(3) = depthData.get_distance(xCenter, yCenter);

                terminate = true;
            else
                terminate = false;

            end
        end
    end
end