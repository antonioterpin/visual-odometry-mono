classdef MalagaInputBlock < InputBlock
    %MALAGAINPUTBLOCK Handles Malaga dataset
    
    properties (Access = private)
        LeftImages 
        ImageNameFormat string
        Path string
    end
    
    methods
        function obj = MalagaInputBlock(path)
            obj.Path = path;
            obj.IsKConstant = true;
        end
        
        function obj = init(obj)
            obj.K = obj.loadIntrinsics();
            obj.ImageNameFormat = strcat(obj.Path, ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/%s');
            obj.LeftImages = dir(strcat(obj.Path,...
                '/malaga-urban-dataset-extract-07_rectified_800x600_Images/'));
            obj.LeftImages = obj.LeftImages(3:2:end);
            obj.NumberOfImages = length(obj.LeftImages);
        end
    end
    
    methods (Access = protected)
        function image = getImage_(obj, imageIndex)
            currentImage = obj.LeftImages(imageIndex);
            image = rgb2gray(imread(sprintf(obj.ImageNameFormat, ...
                currentImage.name)));
        end
        
        function K = loadIntrinsics(obj)
            %loadIntrinsics Loads the instrinsics parameters
            % hardcoded 
            K = [621.18428 0 404.0076
                    0 621.18428 309.05989
                    0 0 1];
        end
        
        function [pose, rotation, translation] = getTruePose_(obj, poseIndex)
            fid = fopen(sprintf('%s/malaga-urban-dataset-extract-07_all-sensors_GPS.txt',...
                obj.Path));
            for i = 1:poseIndex+1   %skip first line
                line = fgetl(fid);
            end
            tmpPose = sscanf(line, '%f');
            translation = [-tmpPose(9);
                -tmpPose(11);
                -tmpPose(10)];
            pose = NaN;
            rotation = NaN;
        end
        
    end
end

