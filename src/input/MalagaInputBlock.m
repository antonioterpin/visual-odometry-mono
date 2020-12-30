classdef MalagaInputBlock < InputBlock
    %MALAGAINPUTBLOCK Handles Malaga dataset
    
    properties (Access = private)
        LeftImages 
        ImageNameFormat string
        Path string
        justStartedGTMalaga = true;
        historyPosesMalaga = [];    %use this to store every pose to fit
        discretizedPosesMalaga; %matrix of the correct number of poses 
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
            if obj.justStartedGTMalaga
                obj.justStartedGTMalaga = false;
                %Create collection of all GPS positions
                fid = fopen(sprintf('%s/malaga-urban-dataset-extract-07_all-sensors_GPS.txt',...
                obj.Path));
                line = fgetl(fid);  %skip first line
                for i = 2 : 107
                    line = fgetl(fid);
                    tmpPose = sscanf(line, '%f');
                    pose = [-tmpPose(9);
                        -tmpPose(11);
                        -tmpPose(10)];
                    obj.historyPosesMalaga = [obj.historyPosesMalaga, pose];
                end
                %Fit positions and generate new datapoints
                t = 1 : numel(obj.historyPosesMalaga(1, :));
                xy = [obj.historyPosesMalaga(1, :);
                    obj.historyPosesMalaga(3, :)];
                pp = spline(t, xy);
                tInterp = linspace(1, numel(obj.historyPosesMalaga(1, :)), obj.NumberOfImages);
                xyInterp = ppval(pp, tInterp);
                obj.discretizedPosesMalaga = [xyInterp(1, :);
                                            zeros(1, size(xyInterp, 2));
                                            xyInterp(2, :)];
            end
            
            translation = obj.discretizedPosesMalaga(:, poseIndex);
            pose = translation;
            rotation = NaN;
        end
        
    end
end

