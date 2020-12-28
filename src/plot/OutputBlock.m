classdef OutputBlock < handle
    %OUTPUT Plots the meaningful outputs of the pipeline
    
    properties
        inputHandler
        
        historyData = table([],[],[],[],...
            'VariableNames', {'FrameId', 'Pose', 'PoseGT', 'nLandmarks'});
        
        % Params
        plotGroundTruth = false;
        recentTrajectorySize = 20;
        smooth = false;
        color1 = 'g';
        color2 = 'r';
    end
    
    properties (Constant)
        configurableProps = { 'plotGroundTruth', ...
            'recentTrajectorySize', 'smooth', 'color1', 'color2' }
    end
    
    methods
        function plotCurrentImage(obj, image, p1, p2)
            %PLOTCURRENTIMAGE Plots the upper left part of the output.

            %p1 and p2: [Nx2] [U,V] matrices

            imshow(image);
            hold on;
            if(~isempty(p1))
                scatter(p1(1, :), p1(2, :), 5, obj.color1, 'filled');
            end
            if(~isempty(p2))
                scatter(p2(1, :), p2(2, :), 5, obj.color2);
            end
            hold off;
            title('Current Image');
        end
        
        function plotRecentTrajectory (obj, landmarks, R_CW)
            %PLOTRECENTTRAJECTORY

            % poses is the [Nx3] vector of translation of the camera frame,
            % where each entry is computed as (-R_IC.' * t_IC)'.
            % Thus the three components are x, y and z.

            % landmarks is the [Nx3] matrix of the current set of landmarks
            
            % Last N poses and current landmarks
%             nLandmarks = size(landmarks,2);
            
            if(size(obj.historyData, 1) > obj.recentTrajectorySize)
                poses = obj.historyData.Pose(end-obj.recentTrajectorySize+1 : end, :).';
            else
                poses = obj.historyData.Pose';
            end
            
            %Remove points behind camera
            landmarksC = R_CW * landmarks;
            landmarksCY = landmarksC(3, :);
            landmarks = landmarks(:, landmarksCY > 0);
            
            %Get parameters to adjust axis
            topLan = floor(0.7 * size(landmarks, 2));   %consider 70% of landmarks
            [~, idxX] = sort(abs(landmarks(1, :) - poses(1, end)), 'ascend');
            idxX = idxX(1:topLan);
            [~, idxY] = sort(abs(landmarks(3, :) - poses(3, end)), 'ascend');
            idxY = idxY(1:topLan);
            idxPlot = intersect(idxX, idxY);
            landmarksPlot = landmarks(:, idxPlot);
            xMin = min(landmarksPlot(1, :));
            xMax = max(landmarksPlot(1, :));
            yMin = min(landmarksPlot(3, :));
            yMax = max(landmarksPlot(3, :));
            

            %Get extreme values
            minX = min([poses(1, :), xMin]);
            maxX = max([poses(1, :), xMax]);
            minY = min([poses(3, :), yMin]);
            maxY = max([poses(3, :), yMax]);
            if obj.smooth
                plot(smooth(poses(1, :), 10), smooth(poses(3, :), 10), '-bx', 'MarkerSize', 2)
            else
                plot(poses(1, :), poses(3, :), '-bx', 'MarkerSize', 2)
            end
            hold on;
            scatter(landmarksPlot(1, :), landmarksPlot(3, :), 4, 'k')
        %     set(gcf, 'GraphicsSmoothing', 'on');
            view(0,90);
            hold off;
            axis([minX-5, maxX+5, minY-5, maxY+5])
            title(sprintf('Trajectory of last %d frames and landmarks', obj.recentTrajectorySize));
        end
        
        function plotLandmarksHistory (obj)
            %PLOTLANDMARKSHISTORY Plots the number of the landmarks over the past
            %iterations

            hSize = numel(obj.historyData.nLandmarks);
            idx = max(1, hSize - obj.recentTrajectorySize):hSize;
            
            plot(obj.historyData.FrameId(idx), obj.historyData.nLandmarks(idx), '-k');
            title(sprintf('# tracked landmarks over last %d frames', obj.recentTrajectorySize));
        end
        
        function plotFullTrajectory (obj)
            %PLOTFULLTRAJECTORY
            %Get extreme values
            
            poses = obj.historyData.Pose.';
            minPosesX = Inf; maxPosesX = -Inf;
            minPosesY = Inf; maxPosesY = -Inf;
            hold off;
            
            if obj.plotGroundTruth
                posesGT = obj.historyData.PoseGT.';
                
                poses = alignEstimateToGroundTruth(posesGT, poses);
                
                plot(posesGT(1, :), posesGT(3, :), '-ro','MarkerSize', 2)
                hold on
                
                minPosesX = min(posesGT(1,:));
                maxPosesX = max(posesGT(1,:));
                minPosesY = min(posesGT(3,:));
                maxPosesY = max(posesGT(3,:));
            end
            
            minPosesX = min(minPosesX, min(poses(1, :)));
            maxPosesX = max(maxPosesX, max(poses(1, :)));
            minPosesY = min(minPosesY, min(poses(3, :)));
            maxPosesY = max(maxPosesY, max(poses(3, :)));
            
            if obj.smooth
                poses(1,:) = smooth(poses(1, :), 10);
                poses(3,:) = smooth(poses(3, :), 10);
            end
            
            plot(poses(1, :), poses(3, :), '-bx', 'MarkerSize', 2)
            
            axis ([minPosesX - 5, maxPosesX + 5, minPosesY - 5, maxPosesY + 5])
            title('Full Trajectory');
        end
        
        function plot (obj, imageIdx, p1, p2, R_CW, t_CW, landmarks)
            %PLOT
            % image is the current image
            % p1 and p2: [Nx2] [U,V] matrices
            % poses is the [Nx3] vector of translation of the camera frame,
            %    where each entry is computed as (-R_IC.' * t_IC)'.
            % landmarks is the [Nx3] matrix of the current set of landmarks
            
            plotHeight = 2;
            plotWidth = 4;
            image = obj.inputHandler.getImage(imageIdx);
            gtPose = obj.inputHandler.getTruePose(imageIdx);
            
            % Remove poses after
            % Remark: Output is completely sequential!
            [entriesToRemove, ~] = find(obj.historyData.FrameId >= imageIdx);
            obj.historyData(entriesToRemove, :) = [];
            
            % Add data to history
            obj.historyData = [obj.historyData; 
                table(imageIdx, (-R_CW.' * t_CW).', gtPose.', size(landmarks, 2), ...
                'VariableNames', {'FrameId', 'Pose', 'PoseGT', 'nLandmarks'})];
            
            % Plot
            
            figure(1);
            % Image with keypoints
            subplot(plotHeight,plotWidth,[1,2])
            obj.plotCurrentImage(image, p1, p2)
            
            subplot(plotHeight,plotWidth,[3,4,7,8])
            obj.plotRecentTrajectory(landmarks, R_CW);
            
            % Number of triangulated landmarks
            subplot(plotHeight,plotWidth,5)
            obj.plotLandmarksHistory()

            % Full trajectory
            subplot(plotHeight,plotWidth,6)
            obj.plotFullTrajectory()
        end
        
%         function plotGroundTruth(obj, inputHandler, imageIdx)
%             figure(2)
%             [poses, ~, ~] = inputHandler.getTruePose(imageIdx);
%             
%             if poses(1) < obj.gtHistory(1)
%                 obj.gtHistory(1) = poses(1);
%             end
%             if poses(1) > obj.gtHistory(2)
%                 obj.gtHistory(2) = poses(1);
%             end
%             if poses(3) < obj.gtHistory(3)
%                 obj.gtHistory(3) = poses(3);
%             end
%             if poses(3) > obj.gtHistory(4)
%                 obj.gtHistory(4) = poses(3);
%             end
%             plot(poses(1, :), poses(3, :), '-rx','MarkerSize', 2) % smooth eventually later
%             axis ([obj.gtHistory(1) - 5, obj.gtHistory(2) + 5,...
%                 obj.gtHistory(3) - 5, obj.gtHistory(4) + 5])
%             title('Full Ground-Truth Trajectory')
%             hold on;
%         end
    end
end