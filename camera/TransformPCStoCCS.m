classdef TransformPCStoCCS < handle
    properties
        % shape of the checkerboard
        dimensions
        % edge length of a square
        edgeLength
        % camera parameters
        cameraParameters
        intrinsics
        extrinsics
        transformToRCS
        % checkerboard points in ...
        pointsPCS % ... pixel coordinates
        pointsBCS % ... board coordinates
        pointsCCS % ... camera coordinates
        pointsRCS % ... robot coordinates
    end
    methods
        function object = TransformPCStoCCS(image, cameraParameters, edgeLength)

            % store camera parameters and edge length in object
            object.cameraParameters = cameraParameters;
            object.intrinsics = cameraParameters.Intrinsics;
            object.edgeLength = edgeLength;

            % identify the points of intersection in the checkerboard pattern and
            % return them as a list of xy coordinates.
            % https://de.mathworks.com/help/vision/ref/detectcheckerboardpoints.html#responsive_offcanvas
            [object.pointsPCS, object.dimensions] = detectCheckerboardPoints(image);
            object.dimensions = object.dimensions - 1;

            % generate the coordinates of the points of intersection within the
            % checkerboard pattern. The unit is determined by the unit chosen in
            % squareSize.
            % https://de.mathworks.com/help/vision/ref/generatecheckerboardpoints.html#responsive_offcanvas
            object.pointsBCS = generateCheckerboardPoints(object.dimensions, object.edgeLength);

            % get the world coordinates xy of the checkerboard points of intersection
            object.pointsCCS = object.generateInCCS();

            disp(object.pointsCCS(1,:))
            disp(object.pointsCCS(6,:))

            % estimate an extrinsic transformation matrix
            % https://de.mathworks.com/help/vision/ref/estimateextrinsics.html#responsive_offcanvas
            object.extrinsics = estimateExtrinsics(object.pointsPCS, object.pointsCCS, object.cameraParameters.Intrinsics);

            % check the transformed checkerboard diagonal against an ideal diagonal calculated from the checkerboard metadata
            score = object.checkPlausability();
            disp("Score: " + num2str(round(score,2)) + " %");


            GenPointsCCS = img2world2d(object.pointsPCS, object.extrinsics, object.intrinsics);
            
            diff = object.pointsCCS - GenPointsCCS


        end % TransformPCStoRCS

        function extendToRCS(self, boardOriginRCS, boardCornerRCS)

            % construct the 42 checkerboard points from points 1 and 6 in the rcs
            self.pointsRCS = generateInRCS(self, boardOriginRCS, boardCornerRCS);

            % calculate a new set of checkerboard points in ccs
            augPointsCCS = img2world2d(self.pointsPCS, self.extrinsics, self.intrinsics);

            % match orientation of ccs and rcs
            vectorCCS = augPointsCCS(1,:) - augPointsCCS(6,:);
            vectorCCS = vectorCCS / norm(vectorCCS);
            vectorCCS(3) = 0;

            vectorRCS = self.pointsRCS(1,:) - self.pointsRCS(6,:);
            vectorRCS = vectorRCS / norm(vectorRCS);
            vectorRCS(3) = 0;

            % rotate the ccs vector by 180 degree around the x axis to match
            % orientation of the z axes
            rotationX = rotx(pi);
            vectorCCS = rotationX * vectorCCS';

            angleZ = acos(dot(vectorCCS, vectorRCS));
            rotationZ = rotz(angleZ);

            augPointsCCS(:,3) = 0;
            rotationMatrix =  rotationX * rotationZ;
            augPointsCCS = rotationMatrix * augPointsCCS';

            augPointsCCS = augPointsCCS(:, 1:2);
            translationVector = self.pointsRCS(1,:) - augPointsCCS(1,:);

            translationLength = norm(translationVector);
            disp("Distance CCS to RCS: " + num2str(round(translationLength)) + " mm");

            transformationMatrix = eye(4);
            transformationMatrix(1:3, 1:3) = rotationMatrix;
            transformationMatrix(1:2, 4) = translationVector;

            self.transformToRCS = rigidtform3d(transformationMatrix);
        end % extendToRCS

        function targetPointsRCS = apply(self, targetPointsPCS)

            % apply transformation from pcs to ccs and augment the point
            % coordinates
            targetPointsCCS = img2world2d(double(targetPointsPCS), self.extrinsics, self.intrinsics);
            targetPointsCCS(:,4) = 1;

            % apply transformation from ccs to rcs and undo augmentation
            targetPointsRCS = (self.transformToRCS.A * targetPointsCCS')';
            targetPointsRCS = targetPointsRCS(:,1:3);
        end

        function show(self, image)
             offset = 5;
             axisLength = 100;


            % % Display the image and plot the detected corners and axes.
            imshow(image);
            hold on;

            % \\\\\\\\\\\\\\\\\\\\ 1 //////////////////// %
            % draw the pixel coordinate system

            offsetPCS = 2 * offset;

            framePoints = [offsetPCS offsetPCS;
                offsetPCS offsetPCS;
                offsetPCS offsetPCS];
            framePoints(1,1) = framePoints(1,1) + axisLength;
            framePoints(3,2) = framePoints(3,2) + axisLength;
            
            % draw the x axis in red
            line(framePoints(1:2,1), framePoints(1:2,2), 'Color', 'r', 'LineWidth', 2);
            % draw the y axis in green
            line(framePoints(2:3,1), framePoints(2:3,2), 'Color', 'g', 'LineWidth', 2);
            % write the frame name aside the axes
            text(framePoints(2,1) + offsetPCS + 5, framePoints(2,2) + offsetPCS + 5, 'PCS', 'Color', 'b', 'FontSize', 8);

            % \\\\\\\\\\\\\\\\\\\\ 2 //////////////////// %
            % draw the camera coordinate system

            % construct the three points to draw a 2d coordinate frame
            framePoints = [self.cameraParameters.PrincipalPoint;
                self.cameraParameters.PrincipalPoint;
                self.cameraParameters.PrincipalPoint];
            framePoints(1,1) = framePoints(1,1) - axisLength;
            framePoints(3,2) = framePoints(3,2) - axisLength;

            % draw the x axis in red
            line(framePoints(1:2,1), framePoints(1:2,2), 'Color', 'r', 'LineWidth', 2);
            % draw the y axis in green
            line(framePoints(2:3,1), framePoints(2:3,2), 'Color', 'g', 'LineWidth', 2);
            % write the frame name aside the axes
            text(framePoints(2,1) + offset, framePoints(2,2), 'CCS', 'Color', 'b', 'FontSize', 8);

            % \\\\\\\\\\\\\\\\\\\\ 3 //////////////////// %
            % draw the checkerboard coordinate system

            offsetBCS = 2 * offset;
            
            % construct the three points to draw a 2d coordinate frame
            framePoints = [self.pointsPCS(self.dimensions(1) + 1,:); self.pointsPCS(1,:); self.pointsPCS(2,:)];
            
            % draw the x axis in red
            line(framePoints(1:2,1), framePoints(1:2,2), 'Color', 'r', 'LineWidth', 2);
            % draw the y axis in green
            line(framePoints(2:3,1), framePoints(2:3,2), 'Color', 'g', 'LineWidth', 2);
            % write the frame name aside the axes
            text(framePoints(2,1) + offsetBCS, framePoints(2,2), 'BCS', 'Color', 'y', 'FontSize', 8);
            

            % \\\\\\\\\\\\\\\\\\\\ 4 //////////////////// %
            % draw the checkerboard image points

            % draw circles around the points of intersection
            plot(self.pointsPCS(:, 1), self.pointsPCS(:, 2), 'ro');
            
            % label these circles with their index
            for i = 1:size(self.pointsPCS, 1)
                text(self.pointsPCS(i, 1) + offset, self.pointsPCS(i, 2), num2str(i), 'Color', 'y', 'FontSize', 8);
            end

            hold off;
        end % show
    end
    methods (Access = private)
        function pointsCCS = generateInCCS(self)

            % calculate the number of points of intersection in x and y
            % direction
            dimensionX = self.dimensions(2);
            dimensionY = self.dimensions(1);

            % calculate the list length
            n = dimensionX * dimensionY;

            % initialize lists of length n
            pixelDistancesX = eye(n, 1);
            pixelDistancesY = eye(n, 1);

            % get the image square edge lengths as mean pixel distance between the
            % adjacent checkerboard image points
            for i = 1:n
                if i <= (n - dimensionY)
                    to = i + dimensionY;
                    pixelDistancesX(i) = norm(self.pointsPCS(i,:) - self.pointsPCS(to,:));
                else
                    pixelDistancesX(i) = NaN;
                end
                if mod(i, dimensionY) ~= 0
                    pixelDistancesY(i) = norm(self.pointsPCS(i,:) - self.pointsPCS(i+1,:));
                else
                    pixelDistancesY(i) = NaN;
                end
            end

            edgeLengthPCS = mean(cat(1, pixelDistancesX, pixelDistancesY), 'omitnan');

            % calculate a scaling factor from the ratio of the square edge lengths
            scalingFactor = self.edgeLength / edgeLengthPCS;

            pointsCCS = (double(self.cameraParameters.PrincipalPoint) - self.pointsPCS) * scalingFactor;
        end % generateInCCS

        function pointsRCS = generateInRCS(self, boardOriginRCS, boardCornerRCS)

            % calculate the number of points of intersection in x and y
            % direction
            dimensionX = self.dimensions(2);
            dimensionY = self.dimensions(1);

            vectorA = boardCornerRCS - boardOriginRCS;
            vectorA = vectorA / norm(vectorA);

            pointsRCS = zeros(dimensionY, 2);

            for i=1:dimensionY
                pointsRCS(i,:) = boardOriginRCS + (i-1) * self.edgeLength * vectorA;
            end

            vectorA(3) = 0;
            vectorZ = [0 0 1];
            vectorB = cross(-vectorA, vectorZ);
            vectorB = vectorB(1:2) * self.edgeLength;

            for j = 1: (dimensionX - 1)
                newPointsRCS = pointsRCS(1:dimensionY,:) + j * vectorB;
                pointsRCS = cat(1,pointsRCS, newPointsRCS);
            end
        end % generateInRCS

        function score = checkPlausability(self)

            dimensionX = self.dimensions(2) - 1;
            dimensionY = self.dimensions(1) - 1;

            idealDiagonal = sqrt(dimensionX^2 + dimensionY^2) * self.edgeLength;

            diagonalPoints = [self.pointsPCS(1,:); self.pointsPCS((dimensionX + 1)*(dimensionY + 1), :)];
            diagonalWorldPoints = img2world2d(diagonalPoints, self.extrinsics, self.intrinsics);

            approxDiagonal = norm(diagonalWorldPoints(1, :) - diagonalWorldPoints(2, :));

            score = 100 * approxDiagonal / idealDiagonal;
        end % checkPlausability
    end
end