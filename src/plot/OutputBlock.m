classdef OutputBlock < handle
    %OUTPUT Plots the meaningful outputs of the pipeline
    
    properties
        inputHandler
        state
        
        historyData = table([],[],[],...
            'VariableNames', {'FrameId', 'PoseGT', 'nLandmarks'});
        
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
            
            if ~isempty(p1) && ~isempty(p2)
                legend({'Tracked keypoints', 'Candidates'}, 'Location', 'southoutside')
            end
        end
        
        function plotRecentTrajectory (obj, landmarks)
            %PLOTRECENTTRAJECTORY

            % poses is the [Nx3] vector of translation of the camera frame,
            % where each entry is computed as (-R_IC.' * t_IC)'.
            % Thus the three components are x, y and z.

            % landmarks is the [Nx3] matrix of the current set of landmarks
            
            % Last N poses and current landmarks
%             nLandmarks = size(landmarks,2);

            poses = obj.state.Poses.Position.';
            
            if(size(poses, 2) > obj.recentTrajectorySize)
                poses = poses(:, end-obj.recentTrajectorySize+1 : end);
            end
            
            %Get parameters to adjust axis
            topLan = floor(0.7 * size(landmarks, 2));   %consider 70% of landmarks
            distances = sqrt((abs(landmarks(1, :)-poses(1, end))).^2 +...
                (abs(landmarks(3, :)-poses(3, end))).^2);
            [~, idxSort] = sort(distances, 'ascend');
            idxSort = idxSort(1:topLan);
            landmarksPlot = landmarks(:, idxSort);
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
                poses(1,:) = smooth(poses(1, :), 10);
                poses(3,:) = smooth(poses(3, :), 10);
            end
            
            plot(poses(1, :), poses(3, :), '-bx', 'MarkerSize', 2)
            hold on;
            scatter(landmarksPlot(1, :), landmarksPlot(3, :), 4, 'k')
        %     set(gcf, 'GraphicsSmoothing', 'on');
            view(0,90);
            hold off;
            axis([minX-0.5, maxX+0.5, minY-0.5, maxY+0.5])
            title(sprintf('Trajectory of last %d frames and landmarks', obj.recentTrajectorySize));
        end
        
        function plotLandmarksHistory (obj)
            %PLOTLANDMARKSHISTORY Plots the number of the landmarks over the past
            %iterations

            hSize = numel(obj.historyData.nLandmarks);
            idx = max(1, hSize - obj.recentTrajectorySize):hSize;
            
            plot(obj.historyData.FrameId(idx), obj.historyData.nLandmarks(idx), '-k');
            axis([min(obj.historyData.FrameId(idx)), max(obj.historyData.FrameId(idx)),...
                0, max(max(obj.historyData.nLandmarks(idx)) + 10,200)]);
            title(sprintf('# tracked landmarks over last %d frames', obj.recentTrajectorySize));
        end
        
        function plotFullTrajectory (obj)
            %PLOTFULLTRAJECTORY
            %Get extreme values
            
            poses = obj.state.Poses.Position.';
            minPosesX = Inf; maxPosesX = -Inf;
            minPosesY = Inf; maxPosesY = -Inf;
            hold off;
            
            if obj.plotGroundTruth
                posesGT = obj.historyData.PoseGT.';
                
                if size(posesGT, 2) > 7 % n min of points to solve equations
                    poses = alignEstimateToGroundTruth(posesGT, poses);
                end
                
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
            
            if obj.plotGroundTruth
                legend({'Ground truth', 'Estimate'}, 'Location', 'southoutside')
            end
        end
        
        function plot (obj, imageIdx, p1, p2, landmarks)
            %PLOT
            % image is the current image
            % p1 and p2: [Nx2] [U,V] matrices
            % poses is the [Nx3] vector of translation of the camera frame,
            %    where each entry is computed as (-R_IC.' * t_IC)'.
            % landmarks is the [Nx3] matrix of the current set of landmarks
            
            plotHeight = 2;
            plotWidth = 4;
            image = obj.inputHandler.getImage(imageIdx);
            
            addHistoryEntry(obj, imageIdx, landmarks)
            
            % Plot
            
            figure(1);
            set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
            
            % Image with keypoints
            subplot(plotHeight,plotWidth,[1,2])
            obj.plotCurrentImage(image, p1, p2)
            
            subplot(plotHeight,plotWidth,[3,4,7,8])
            obj.plotRecentTrajectory(landmarks);
            
            % Number of triangulated landmarks
            subplot(plotHeight,plotWidth,5)
            obj.plotLandmarksHistory()

            % Full trajectory
            subplot(plotHeight,plotWidth,6)
            obj.plotFullTrajectory()
        end
        
        function addHistoryEntry(obj, frameIdx, landmarks)
            % Add data to history
            gtPose = obj.inputHandler.getTruePose(frameIdx);
            
            obj.historyData = [obj.historyData; 
                table(frameIdx, gtPose.', size(landmarks, 2), ...
                'VariableNames', {'FrameId', 'PoseGT', 'nLandmarks'})];
        end
    end
end