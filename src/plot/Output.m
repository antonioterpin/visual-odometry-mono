classdef Output < handle
    %OUTPUT Plots the meaningful outputs of the pipeline
    
    properties
        inputHandler
        posesHistory = [];
        landmarksHistorySize = [];
        frameIdx = [];
        gtHistory = [Inf, -Inf, Inf, -Inf]; %dummy initialization
        superimposeGT = false;
        groundTruth = [];
        recentTrajectorySize = 20;
        landmarksHistorySizePlot = 20;
        smooth = false;
    end
    
    properties (Constant)
        configurableProps = { 'superimposeGT', 'recentTrajectorySize', ...
           'landmarksHistorySizePlot','smooth' }
    end
    
    methods
        function plotCurrentImage (obj, image, inliers, outliers)
            %PLOTCURRENTIMAGE Plots the upper left part of the output.

            %inliers and outliers: [Nx2] [U,V] matrices

            imshow(image);
            hold on;
            if(~isempty(inliers))
                scatter(inliers(1, :), inliers(2, :), 10, 'g', 'filled');
            end
            if(~isempty(outliers))
                scatter(outliers(1, :), outliers(2, :), 10, 'r', 'filled');
            end
            hold off;
            title('Current Image');
        end
        
        function plotRecentTrajectory (obj, poses, landmarks, R_CW)
            %PLOTRECENTTRAJECTORY

            % poses is the [Nx3] vector of translation of the camera frame,
            % where each entry is computed as (-R_IC.' * t_IC)'.
            % Thus the three components are x, y and z.

            % landmarks is the [Nx3] matrix of the current set of landmarks
            
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
            title('Recent Trajectory and Current Landmarks')
        end
        
        function plotLandmarksHistory (obj, landmarksHistory)
            %PLOTLANDMARKSHISTORY Plots the number of the landmarks over the past
            %iterations

            if length(landmarksHistory) < obj.landmarksHistorySizePlot
                plot(obj.frameIdx, landmarksHistory, '-k')
            else
                plot(obj.frameIdx(end-obj.landmarksHistorySizePlot+1 : end),...
                    landmarksHistory(end-obj.landmarksHistorySizePlot+1 : end), '-k')
            end
            title('Landmarks History')
        end
        
        function plotFullTrajectory (obj, poses, imageIdx)
            %PLOTFULLTRAJECTORY
            %Get extreme values
            minPosesX = min(poses(1, :));
            maxPosesX = max(poses(1, :));
            minPosesY = min(poses(3, :));
            maxPosesY = max(poses(3, :));
            if obj.superimposeGT
                
                [posesGT, ~, ~] = obj.inputHandler.getTruePose(imageIdx);
                obj.groundTruth = [obj.groundTruth, posesGT];

                if posesGT(1) < obj.gtHistory(1)
                    obj.gtHistory(1) = posesGT(1);
                end
                if posesGT(1) > obj.gtHistory(2)
                    obj.gtHistory(2) = posesGT(1);
                end
                if posesGT(3) < obj.gtHistory(3)
                    obj.gtHistory(3) = posesGT(3);
                end
                if posesGT(3) > obj.gtHistory(4)
                    obj.gtHistory(4) = posesGT(3);
                end
                
                if obj.smooth
                    plot(smooth(poses(1, :), 10), smooth(poses(3, :), 10), '-bx', 'MarkerSize', 2)
                else
                    plot(poses(1, :), poses(3, :), '-bx', 'MarkerSize', 2)
                end
                hold on
                plot(obj.groundTruth(1, :), obj.groundTruth(3, :), '-rx','MarkerSize', 2)
                hold off
                axis ([obj.gtHistory(1) - 5, obj.gtHistory(2) + 5,...
                    obj.gtHistory(3) - 5, obj.gtHistory(4) + 5])
                title('Full trajectory and Ground-Truth Trajectory')                
            else
                if obj.smooth
                    plot(smooth(poses(1, :), 10), obj.smooth(poses(3, :), 10), '-bx', 'MarkerSize', 2)
                else
                    plot(poses(1, :), poses(3, :), '-bx', 'MarkerSize', 2)
                end
                axis ([minPosesX - 5, maxPosesX + 5, minPosesY - 5, maxPosesY + 5])
                title('Full Trajectory')
            end
            
        end
        
        function plot (obj, imageIdx, inliers, outliers, R_CW, t_CW, landmarks)
            %PLOT
            % image is the current image
            % inliers and outliers: [Nx2] [U,V] matrices
            % poses is the [Nx3] vector of translation of the camera frame,
            %    where each entry is computed as (-R_IC.' * t_IC)'.
            % landmarks is the [Nx3] matrix of the current set of landmarks
            
            %Recover useful data
            image = obj.inputHandler.getImage(imageIdx);
            
            %Build history table
            if any(obj.frameIdx == imageIdx)
                obj.posesHistory = obj.posesHistory(1:imageIdx-1, :);
                obj.posesHistory = obj.landmarksHistorySize(1:imageIdx-1);
            end
                obj.frameIdx = [obj.frameIdx; imageIdx];
                obj.posesHistory = [obj.posesHistory; (-R_CW.' * t_CW)'];
                obj.landmarksHistorySize = [obj.landmarksHistorySize; size(landmarks, 2)];
            
            figure(1)
            % Image with keypoints
            subplot(2,2,1)
            obj.plotCurrentImage(image, inliers, outliers)

            % Last N poses and current landmarks
            if(size(obj.posesHistory, 1) > obj.recentTrajectorySize)
                posesLast = obj.posesHistory(end-obj.recentTrajectorySize+1 : end, :)';
            else
                posesLast = obj.posesHistory';
            end
            subplot(1,2,2)
            obj.plotRecentTrajectory(posesLast, landmarks, R_CW)

            % Number of triangulated landmarks
            subplot(2,4,5)
            obj.plotLandmarksHistory(obj.landmarksHistorySize)

            % Full trajectory
            subplot(2,4,6)
            obj.plotFullTrajectory(obj.posesHistory', imageIdx)
        end
        
        function plotGroundTruth(obj, inputHandler, imageIdx)
            figure(2)
            [poses, ~, ~] = inputHandler.getTruePose(imageIdx);
            
            if poses(1) < obj.gtHistory(1)
                obj.gtHistory(1) = poses(1);
            end
            if poses(1) > obj.gtHistory(2)
                obj.gtHistory(2) = poses(1);
            end
            if poses(3) < obj.gtHistory(3)
                obj.gtHistory(3) = poses(3);
            end
            if poses(3) > obj.gtHistory(4)
                obj.gtHistory(4) = poses(3);
            end
            plot(poses(1, :), poses(3, :), '-rx','MarkerSize', 2) % smooth eventually later
            axis ([obj.gtHistory(1) - 5, obj.gtHistory(2) + 5,...
                obj.gtHistory(3) - 5, obj.gtHistory(4) + 5])
            title('Full Ground-Truth Trajectory')
            hold on;
        end
    end
end