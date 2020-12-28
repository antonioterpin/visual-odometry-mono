classdef Klt < handle
    
    properties
        inputHandler    %TODO make it work properly
        r_T = 15;
        num_iters = 50;
        lambda = 0.1;
        nSkip = 1;  %TODO json
        triangulationSample = 8 ;%8-point-alg   %TUNING
        triangulationIterations = 2000;         %TUNING//MAKE ADAPTIVE
        triangulationTolerance = 1;             %TUNING
    end
    
    methods
        function [delta_keypoint, keep] = trackKLTRobustly(obj,...
                image, image1, keypoints)
            % I_prev: reference image, I: image to track point in, keypoint: point to 
            % track, expressed as [x y]=[col row], r_T: radius of patch to track, 
            % num_iters: amount of iterations to run, lambda: bidirectional error
            % threshold; delta_keypoint: delta by which the keypoint has moved between 
            % images, (2x1), keep: true if the point tracking has passed the
            % bidirectional error test.
            
            W = trackKLT(image1, image, keypoints, obj.r_T, obj.num_iters);
            delta_keypoint = W(:, end);
            Winv = trackKLT(image, image1, (keypoints'+delta_keypoint)', obj.r_T, ...
                obj.num_iters);
            dkpinv = Winv(:, end);
            keep = norm(delta_keypoint + dkpinv) < obj.lambda;

        end
        
        function [kpold, keypoints, keypointsLost, keep] = KltTracker (obj, inputHandler,...
                imageIdx, keypoints)
            %KLTTRACKER
                %TODO Documentation
            %Output:    keypoints: new keypoints
            %           kpold: previous keypoints
            %               OSS: keypoints(:, i) matches with kpold(:, i)
            
            %Get images and reduce resolution
            image1 = inputHandler.getImage(imageIdx-obj.nSkip);
            image = inputHandler.getImage(imageIdx);
            
            %Reduce size keypoints and images
            image1 = imresize(image1, 0.25);
            image = imresize(image, 0.25);
            keypoints = keypoints / 4;
            
            dkp = zeros(size(keypoints));
            keep = true(1, size(keypoints, 2));
            parfor j = 1:size(keypoints, 2)
                [dkp(:,j), keep(j)] = trackKLTRobustly(...
                    image, image1, keypoints(:,j)', obj.r_T, ...
                    obj.num_iters, obj.lambda);
            end
            kpold = keypoints(:, keep);
            keypoints = keypoints + dkp;
            keypointsLost = keypoints(:, ~keep);    %not sure it's gonna be useful
            keypoints = keypoints(:, keep);
            
%             %Visualize matches      TODO: remove plots when it works for sure
%             figure(10)
%             imshow(image);
%             hold on;
%             plotMatches(1:size(keypoints, 2), flipud(keypoints), flipud(kpold));
%             hold off;
%             pause(0.1);
            
            %Expand again keypoint size
            kpold = kpold * 4;
            keypoints = keypoints * 4;
            keypointsLost = keypointsLost * 4;
            
            
        end

    end
end