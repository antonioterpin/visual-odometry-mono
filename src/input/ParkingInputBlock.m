classdef ParkingInputBlock < InputBlock
    %PARKINGINPUTBLOCK Handles Parking dataset
    
    properties (Access = private)
        ImageNameFormat string
        Path string
    end
    
    methods
        function obj = ParkingInputBlock(path)
            obj.Path = path;
            obj.IsKConstant = true;
        end
        
        function obj = init(obj)
            obj.K = obj.loadIntrinsics();
            obj.ImageNameFormat = strcat(obj.Path, '/images/img_%05d.png');
            obj.NumberOfImages = 598;
        end
    end
    
    methods (Access = protected)
        function image = getImage_(obj, imageIndex)
            image = rgb2gray(imread(...
                sprintf(obj.ImageNameFormat, imageIndex)));
        end
        
        function K = loadIntrinsics(obj)
            %loadIntrinsics Loads the instrinsics parameters
            % we select the first row
            K = load(sprintf('%s/K.txt', obj.Path));
        end
        
        function [pose, rotation, translation] = getTruePose_(obj, poseIndex)
            fid = fopen(sprintf('%s/poses.txt', obj.Path));
            for i = 1:poseIndex
                line = fgetl(fid);
            end
            tmpPose = sscanf(line, '%f');
            translation = [tmpPose(4);
                tmpPose(8);
                tmpPose(12)];
            rotation = [tmpPose(1:3)';
                tmpPose(5:7)';
                tmpPose(9:11)'];
            pose = translation;
            fclose(fid);
        end

    end
end