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
        function object = Camera3D(cameraParametersPath)

            % load and store camera parameters
            load(cameraParametersPath);
            object.parameters = cameraParams;

            cameraParams.Intrinsics

            width = cameraParams.Intrinsics.ImageSize(2);
            height = cameraParams.Intrinsics.ImageSize(1);

            clear cameraParams;

            % set camera resolution to 1280 x 720 px for color and depth camera
            object.configuration = realsense.config();
            object.configuration.enable_stream(realsense.stream.color,-1, width, height, realsense.format.rgb8, 0);
            object.configuration.enable_stream(realsense.stream.depth, width, height, realsense.format.z16, 0);

            % align depth to color image
            object.align = realsense.align(realsense.stream.color);

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
            self.pipe.start(self.configuration);

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
            % depthData = depth.get_data();
            % depthImage = permute(reshape(depthData',[3,depth.get_width(),depth.get_height()]),[3 2 1]);

            % stop streaming
            self.pipe.stop();
        end % snapshot

        function result = stream(self, inputFunction)

            % start streaming on an arbitrary camera with default settings
            self.pipe.start(self.configuration);

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
                % depthData = depth.get_data();
                % depthImage = permute(reshape(depthData',[3,depth.get_width(),depth.get_height()]),[3 2 1]);

                [result, terminate] = inputFunction(colorImage, depth);

                if terminate
                    break
                end
            end

            % stop streaming
            self.pipe.stop();
        end % stream
    end
end