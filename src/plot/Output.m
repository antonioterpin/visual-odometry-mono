classdef Output < handle
    %OUTPUT Plots the meaningful outputs of the pipeline
    
    properties
        inputHandler
        posesHistory = [];
        landmarksHistorySize = [];
        frameIdx = [];
        gtHistory = [1000, -1000, 1000, -1000]; %dummy initialization
        superimposeGT = false;
        groundTruth = [];
    end
    
    properties (Constant)
        configurableProps = { 'superimposeGT' }
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
            size(landmarksC, 2)
            size(landmarks, 2)
            
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

            plot(poses(1, :), poses(3, :), '-x','MarkerSize', 2) % smooth later eventually
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

            %nSkip is the number of skipped frames in the main iteration.
            % For instance, for 1 : 2 : lastImageIndex, nSkip = 2.
            if length(landmarksHistory) < 20
                plot(obj.frameIdx, landmarksHistory, '-k')
            else
                plot(obj.frameIdx(end-19 : end),...
                    landmarksHistory(end-19 : end), '-k')
            end
            title('Landmarks History')
        end
        
        function plotFullTrajectory (obj, poses, imageIdx, ax4)
            %PLOTFULLTRAJECTORY
            %Get extreme values
            minPosesX = min(poses(1, :));
            maxPosesX = max(poses(1, :));
            minPosesY = min(poses(3, :));
            maxPosesY = max(poses(3, :));
            if obj.superimposeGT
                
                [poses, ~, ~] = obj.inputHandler.getTruePose(imageIdx);
                obj.groundTruth = [obj.groundTruth, poses];

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
                %TODO Fix superposition Ground Truth plot
                plot(ax4, poses(1, :), poses(3, :), '-rx', 'MarkerSize', 2); % smooth eventually later
                hold (ax4, 'on')
                %set(ax4, 'nextplot', 'add')
                plot(ax4, obj.groundTruth(1, :), obj.groundTruth(3, :), 'bx','MarkerSize', 2); % smooth eventually later
                hold (ax4, 'off')
                axis ([obj.gtHistory(1) - 5, obj.gtHistory(2) + 5,...
                    obj.gtHistory(3) - 5, obj.gtHistory(4) + 5])
                title('Full trajectory and Ground-Truth Trajectory')                
            else
                plot(poses(1, :), poses(3, :), '-x','MarkerSize', 2); % smooth eventually later
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
            if(size(obj.posesHistory, 1) > 20)
                posesLast = obj.posesHistory(end-19 : end, :)';
            else
                posesLast = obj.posesHistory';
            end
            subplot(1,2,2)
            obj.plotRecentTrajectory(posesLast, landmarks, R_CW)

            % Number of triangulated landmarks
            subplot(2,4,5)
            obj.plotLandmarksHistory(obj.landmarksHistorySize)

            % Full trajectory
            ax4 = subplot(2,4,6);
            obj.plotFullTrajectory(obj.posesHistory', imageIdx, ax4)
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
            plot(poses(1, :), poses(3, :), '-x','MarkerSize', 2) % smooth eventually later
            axis ([obj.gtHistory(1) - 5, obj.gtHistory(2) + 5,...
                obj.gtHistory(3) - 5, obj.gtHistory(4) + 5])
            title('Full Ground-Truth Trajectory')
            hold on;
        end
    end
end