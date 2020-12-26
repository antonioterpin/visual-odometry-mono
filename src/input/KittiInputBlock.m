classdef KittiInputBlock < InputBlock
    %KITTIINPUTBLOCK Handles Kitti dataset
    
    properties (Access = private)
        ImageNameFormat string
        Path string
    end
    
    methods
        function obj = KittiInputBlock(path)
            obj.Path = path;
            obj.IsKConstant = true;
        end
        
        function obj = init(obj)
            obj.K = obj.loadIntrinsics();
            obj.ImageNameFormat = strcat(obj.Path,'/00/image_0/%06d.png');
            obj.NumberOfImages = 4540;
        end
    end
    
    methods (Access = protected)
        function image = getImage_(obj, imageIndex)
            image = imread(sprintf(obj.ImageNameFormat, imageIndex));
        end
        
        function K = loadIntrinsics(obj)
            %loadIntrinsics Loads the instrinsics parameters
            % we select the first row
            fid = fopen(sprintf('%s/00/calib.txt', obj.Path));
            if fid < 0
                InputBlock.calibrationFileNotFound()
            end
            firstLine = fgetl(fid);
            K = sscanf(firstLine(5:end), '%f');
            K = reshape(K([1:3 5:7 9:11]), 3, 3).';
            fclose(fid);
        end
        
        function [pose, rotation, translation] = getTruePose_(obj, poseIndex)
            fid = fopen(sprintf('%s/poses/00.txt', obj.Path));
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
        end
    end
end

