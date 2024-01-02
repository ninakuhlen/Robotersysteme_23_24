classdef Camera3D < handle
    properties
        % camera parameters
        parameters
        % camera pipeline
        pipe
        % camera configuration and align instructions
        configuration
        align
    end
    methods
        function object = Camera3D(cameraParametersPath, imageWidth, imageHeight)

            % load and store camera parameters
            object.parameters = load(cameraParametersPath, cameraParams);

            % set camera resolution to 1280 x 720 px for color and depth camera
            object.configuration = realsense.config();
            object.configuration.enable_stream(realsense.stream.color,-1, imageWidth, imageHeight, realsense.format.rgb8, 0);
            object.configuration.enable_stream(realsense.stream.depth, imageWidth, imageHeight, realsense.format.z16, 0);

            % align depth to color image
            object.align = realsense.align(realsense.stream.depth);

            % create Pipeline object to manage streaming
            object.pipe = realsense.pipeline();

            % start streaming on an arbitrary camera with default settings
            profile = object.pipe.start(object.configuration);

            % get streaming device's name to check connection
            dev = profile.get_device();
            name = dev.get_info(realsense.camera_info.name);
            disp(name);

            % stop streaming
            object.pipe.stop();
        end % Camera3D

        function [colorImage, depth] = snapshot(self)

            % start streaming on an arbitrary camera with default settings
            object.pipe.start(object.configuration);

            % discard the first couple frames to allow the camera time to settle
            for i = 1:5
                self.pipe.wait_for_frames();
            end

            frames = self.pipe.wait_for_frames();
            alignedFrames = self.align.process(frames);
            color = alignedFrames.get_color_frame();
            depth = alignedFrames.get_depth_frame();

            % get actual data and convert into a format imshow can use
            colorData = color.get_data();
            colorImage = permute(reshape(colorData',[3,color.get_width(),color.get_height()]),[3 2 1]);
            depthData = depth.get_data();
            depthImage = permute(reshape(depthData',[3,depth.get_width(),depth.get_height()]),[3 2 1]);

            % stop streaming
            object.pipe.stop();
        end % snapshot

        function result = stream(self, inputFunction)

            % start streaming on an arbitrary camera with default settings
            object.pipe.start(object.configuration);

            % discard the first couple frames to allow the camera time to settle
            for i = 1:5
                self.pipe.wait_for_frames();
            end

            % data stream
            while true
                frames = self.pipe.wait_for_frames();
                alignedFrames = self.align.process(frames);
                color = alignedFrames.get_color_frame();
                depth = alignedFrames.get_depth_frame();

                % get actual data and convert into a format imshow can use
                colorData = color.get_data();
                colorImage = permute(reshape(colorData',[3,color.get_width(),color.get_height()]),[3 2 1]);
                depthData = depth.get_data();
                depthImage = permute(reshape(depthData',[3,depth.get_width(),depth.get_height()]),[3 2 1]);

                [result, terminate] = inputFunction(colorImage, depthImage);

                if terminate
                    break
                end
            end
        end % stream
    end
end