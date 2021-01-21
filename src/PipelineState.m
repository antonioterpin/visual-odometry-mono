classdef PipelineState < handle
    % PIPELINESTATE Embedds the pipeline state, with the history of poses
    % and 
    % 
    %   Detailed explanation goes here
    
    properties % (Access = private)
        Poses = table([],[],[],[], 'VariableNames', {'Id', 'R_CW', 't_CW', 'Position'})
        Landmarks = table([],[], 'VariableNames', {'Id', 'Position'})
        ObservationGraph = graph % Poses x Landmarks -> (keypoints)
        Candidates
%         K % The intrinsics matrix
        lastLandmark = 0
        lost = true
        descriptorSize = 0
        
        lostBelow = 20
        verbose = true
        alphaTh = 2 % Threshold for triangulation
        maxDistance = 100
        minDistance = 6.5
        pruneOlderThan = 100;
    end
    
    % Config params
    properties (Constant)
        configurableProps = {'lostBelow', 'verbose', ...
            'alphaTh', 'maxDistance', 'minDistance', 'pruneOlderThan'}
    end
    
    methods
        function lost = isLost(state, lost)
            if nargin > 1
                state.lost = lost;
            else
                poseNode = find(state.ObservationGraph.Nodes.PoseId == state.Poses.Id(end));
                state.lost = state.lost || degree(state.ObservationGraph, poseNode) < state.lostBelow;
            end
            
            lost = state.lost;
        end
        
        function [R_CW, t_CW] = getLastPose(state)
            R_CW = reshape(state.Poses.R_CW(end,:), 3, 3);
            t_CW = reshape(state.Poses.t_CW(end,:), 3, 1);
        end
        
        function prevPoseIdx = gotoPrevPose(state)
            prevPoseIdx = [];
            if size(state.Poses, 1) > 1
                prevPoseIdx = state.Poses.Id(end - 1);
                state.resetToPose(prevPoseIdx);
            end
        end
        
        function [R_CW, t_CW] = getPose(state, poseIdx)
            idx = find(state.Poses.Id == poseIdx);
            
            R_CW = reshape(state.Poses.R_CW(idx,:), 3, 3);
            t_CW = reshape(state.Poses.t_CW(idx,:), 3, 1);
        end
        
        function N = getNumberOfLandmarksTrackedAtFrame(state, frameIndex)
            pose_nid = find(state.ObservationGraph.Nodes.PoseId == frameIndex);
            N = degree(state.ObservationGraph, pose_nid);
        end
        
        function resetToPose(state, poseIdx)
            [nidx, ~, ~] = find(state.ObservationGraph.Nodes.PoseId > poseIdx);
            
            state.ObservationGraph = rmnode(state.ObservationGraph, nidx);
            
            % TODO remove all glanding landamrks
        end
        
        function addLandmarksToPose(state, poseIdx, landmarksIdx, keypoints)
            % Take node id
            pose_nid = find(state.ObservationGraph.Nodes.PoseId == poseIdx);
            if isempty(pose_nid)
                verboseDisp(state.verbose, 'Pose not in the observation graph anymore.\n', []);
                return;
            end
            landmarks_nids = find(max(state.ObservationGraph.Nodes.LandmarkId == landmarksIdx.', [], 2));
            % Add edges between pose and landmarks
            nLandmarks = length(landmarksIdx);
            edgeTable = table(...
                [repmat(pose_nid, nLandmarks, 1) landmarks_nids], ...
                keypoints, 'VariableNames', {'EndNodes' 'Keypoints'});
            state.ObservationGraph = addedge(...
                state.ObservationGraph, edgeTable);
        end
        
        function addPose(state, poseIdx, R_CW, t_CW)
            % ADDPOSE Adds a new pose to the observation graph.
            % 
            % TODO DOCUMENT AGAIN!!
            %
            % state = state.ADDPOSE(pose, landmarksIndices, keypoints,
            % descriptors) adds a new pose to the observation graph. The
            % pose is described by the twist vector that embedds the
            % current transformation matrix of the i-th frame T_WC. When
            % adding a pose, the landmarks used to triangulate the pose are
            % required and this are provided through landmarksIndices (Nx1),
            % which is a subset of the ones obtained from state.getLandmarks.
            % keypoints (Nx2 [u v]) and descriptors (NxM) describe the
            % landmarks matched in the frame.
            % 
            % See also addLandmarks.
            
            newPose = table(poseIdx, 0, 'VariableNames', {'PoseId', 'LandmarkId'});
            
            % Add pose node in observation graph
            state.ObservationGraph = addnode(state.ObservationGraph, newPose);
            
            % Add pose to pose table
            state.Poses(end+1,:) = table(poseIdx, ...
                R_CW(:).', t_CW(:).', reshape(-R_CW.' * t_CW, 1, []));
        end
        
        function [landmarks, landmarksIdx, mask] = addLandmarks(state, landmarks)
            % ADDLANDMARKS adds landmarks
            %
            % TODO document again! 3xN
            %
            % state = state.ADDLANDMARKS(landmarks) adds the given pose and
            % the landmarks (Nx3) provided in the observation graph, 
            % connecting them to the
            % newly add pose through the provided keypoints (Nx2 [u v]) and
            % descriptors (NxM).
            %
            % [state, landmarksIndices] = state.ADDLANDMARKS(...)
            % additionally returns the labels of the landamarks, to be used
            % for instance with addPose.
            %
            % See also addPose, getLandmarks.
            
            nLandmarks = size(landmarks, 2);
            
            % TODO eventually filter
            mask = ones(1, nLandmarks) > 0;
            
            landmarksIdx = (state.lastLandmark + 1 : state.lastLandmark + nLandmarks).';
            state.lastLandmark = state.lastLandmark + nLandmarks;
            
            state.Landmarks(end+1:end+nLandmarks,:) ...
                = table(landmarksIdx, landmarks.');
            
            newLandmarks = table(zeros(nLandmarks, 1), landmarksIdx,...
                'VariableNames', {'PoseId', 'LandmarkId'});
            % TODO inf does not work properly on the first column..
            state.ObservationGraph = addnode(state.ObservationGraph, newLandmarks);
        end
        
        function prune(state)
            % Prune older
            [toRemove, ~] = find(...
                state.ObservationGraph.Nodes{:,1} < state.Poses.Id(end) - state.pruneOlderThan...
                & state.ObservationGraph.Nodes{:,1} > 0);
            if nnz(toRemove) > 0
                state.ObservationGraph = rmnode(state.ObservationGraph, toRemove);
                deg = degree(state.ObservationGraph);
                [toRemove, ~] = find(deg == 0);
                if nnz(toRemove) > 0
                    [idx, ~] = find(state.Landmarks.Id == state.ObservationGraph.Nodes{toRemove,2}.');
                    state.Landmarks(idx,:) = [];
                    state.ObservationGraph = rmnode(state.ObservationGraph, toRemove);
                end
            end
        end
        
        function [landmarks, landmarksIdx, keypoints] = getObservations(state, poseId)
            % GETLANDMARKS
            % 
            % TODO COMMENT
            % 
            % See also addLandmarks, addPose.
            
            poseNode = find(state.ObservationGraph.Nodes.PoseId == poseId);
            [eid, landmarksNodesIdx] = outedges(state.ObservationGraph, poseNode);
            landmarksIdx = state.ObservationGraph.Nodes.LandmarkId(landmarksNodesIdx);
            [idx, ~] = find(state.Landmarks.Id == landmarksIdx.');
            landmarks = state.Landmarks.Position(idx, :).';
            keypoints = state.ObservationGraph.Edges.Keypoints(eid, :).';
        end

        function addCandidates(state, frameIdx, candidates)
            % ADDCANDIDATES
            %
            % TODO COMMENT
            %
            % unmatchedKeypoints 2xN
            % unmatchedDescriptors MxN, M assumed to be always the same
            %
            % See also getCandidates, evaluateCandidates, pruneCandidates
            
            if isempty(candidates)
                return;
            end
            
            % Add candidates
            state.Candidates = [state.Candidates; 
                table(repmat(frameIdx, size(candidates, 2), 1),...
                    candidates.', candidates.',...
                    'VariableNames', {'FirstId', 'Keypoint', 'LastSeen'})];
            
            verboseDisp(state.verbose, ...
                'Add %d candidates.\n', size(candidates, 2));
        end

        function candidates = getCandidates(state)
            % GETCANDIDATES Returns candidates that can be used to
            % triangulate new landmarks.
            %
            % TODO update documentation!
            %
            % [candidateLabels, keypoints, descriptors] = state.GETCANDIDATES()
            %
            % candidateLabels is Nx1, the labels of the candidates. These
            % are needed to define which ones have been matched.
            %
            % keypoints is Nx2, the set of keypoints relative to the
            % candidateLabels [u v].
            %
            % descriptors is NxM, the set of descriptors relative to the
            % keypoints.
            % 
            % state.GETCANDIDATES(posesIndices) additionally specify the
            % set of poses indices to consider when selecting the
            % candidates to evaluate.
            %
            % See also evaluateCandidates, addCandidates, pruneCandidates
            
            candidates = [];
            if size(state.Candidates,1) > 0
                candidates = state.Candidates.LastSeen.';
            end
        end
        
        function resetCandidates(state)
            state.Candidates = table([],[],[],...
                'VariableNames', {'FirstId', 'Keypoint', 'LastSeen'});
        end

        function [confirmedKp, confirmedLandmarks, stillCandidates] = evaluateCandidates(state, K, kpcMask, lastSeen)
            % EVALUATECANDIDATES Updates the observation graph and the
            % candidates database given the found matches.
            %
            % TODO update DOCUMENTAION!! 2xN 2xN
            %
            % state = state.EVALUATECANDIDATES(frameIndex, matchLabels,
            % keypoints, descriptors, landmarks, K) Updates the observation 
            % graph and the candidates database given the found matches,
            % which are described by:
            %
            % frameIndex the index of the frame in which the candidates
            % have been matched again (e.g., the current frame).
            %
            % matchLabels (Nx1) the labels of the candidates (see getCandidates)
            % for which a match has been found.
            %
            % keypoints (Nx2, [u v]) and descriptors (NxM) are the 
            % descriptors of the matched candidates in the frameIndex-th 
            % image.
            % 
            % landmarks (Nx3, [X Y Z]) are the triangulated landmarks for
            % the matched candidates.
            
            N = nnz(kpcMask); 
            confirmedKp = [];
            confirmedLandmarks = [];
            stillCandidates = [];
            
            if N == 0
                state.resetCandidates();
                return;
            end
            
            % Filter candidates and update last seen
            state.Candidates(~kpcMask, :) = [];
            state.Candidates.LastSeen = lastSeen.';
            
            [R_2W, t_2W] = state.getLastPose(); % last seen pose
            lsFrameIdx = state.Poses.Id(end); % last seen
            
            confirmedKp = [];
            confirmedLandmarks = [];
            for fsFrameIdx = unique(state.Candidates.FirstId).'
                if fsFrameIdx == lsFrameIdx
                    continue;
                end
                
                idx = state.Candidates.FirstId == fsFrameIdx;
                
                % For all first seen frameIdx
                candidates = state.Candidates.Keypoint(idx, :).';
                lastSeen = state.Candidates.LastSeen(idx, :).';
                
                % Evaluate candidates
                [R_1W, t_1W] = state.getPose(fsFrameIdx);

                T_1W = [R_1W, t_1W; 0, 0, 0, 1];
                T_2W = [R_2W, t_2W; 0, 0, 0, 1];
                T_21 = T_2W / T_1W;
                T_12 = T_1W / T_2W;
                
                % 1. Validation
                N = size(candidates, 2);
%                 bearings1 = normalize(K \ [candidates; ones(1, N)], 'norm');
%                 bearings2 = T_12(1:3,1:3) * normalize(K \ [lastSeen; ones(1, N)], 'norm');
                P_W = triangulateFromPose(...
                    [candidates; ones(1, N)], ...
                    [lastSeen; ones(1, N)], T_21, K, K, T_1W);
                
%                 error = reprojectionError(...
%                     P_W(1:3,:), lastSeen(1:2,:), K, T_2W(1:3,1:3), T_2W(1:3,4));
                
                C1 = -R_1W.'*t_1W;
                C2 = -R_2W.'*t_2W;
%                 C2 = C2*3;
                C2 = C1 + (C2 - C1) * 3;
                
                bearings1 = normalize(P_W(1:3,:) - C1, 'norm');
                bearings2 = normalize(P_W(1:3,:) - C2, 'norm');
                
                cosalpha = dot(bearings1, bearings2);
                valid = abs(rad2deg(acos(cosalpha))) > state.alphaTh;
                
                %RECENTLY ADDED METRIC: remove the XX% farthest and closest
                %landmarks
%                 %FARTHEST and CLOSEST
%                 topLan = floor(0.85 * size(P_W, 2));   %consider XX% of landmarks TOP
%                 bottomLan = ceil(0.05 * size(P_W, 2)); %consider XX% of landmarks BOTTOM
%                 distances = sqrt((abs(P_W(1, :)-C2old(1))).^2 +...
%                     (abs(P_W(3, :)-C2old(3))).^2);
%                 [~, idxSort] = sort(distances, 'ascend');
%                 idxSort = idxSort(bottomLan:topLan);
%                 validPercentage = zeros(size(valid));
%                 validPercentage(idxSort) = 1;
%                 valid = valid .* validPercentage;
%                 valid = valid > 0;
                
                N = nnz(valid);
                if N == 0
%                     verboseDisp(state.verbose, ...
%                         'Triangulated have still low confidence.\n', []);
                    continue;
                end

%                 verboseDisp(state.verbose, ...
%                     '%d points can be triangulated with enough confidence at frame %d.\n', ...
%                     [N, fsFrameIdx]);
                
                stillCandidatesKp = candidates(:,~valid);
                stillCandidatesLs = lastSeen(:,~valid);
                candidates = candidates(:,valid);
                lastSeen = lastSeen(:,valid);
                P_W = P_W(1:3,valid);
                
                % 2. Triangulate landmark
                
%                 P_W = triangulateFromPose(...
%                     [candidates; ones(1, N)], ...
%                     [lastSeen; ones(1, N)], T_21, K, K, T_1W);
                
                % In front of both cameras
                isInFront1 = isInFrontOfCamera(P_W, R_1W, t_1W);
                isInFront2 = isInFrontOfCamera(P_W, R_2W, t_2W);
                isNear = vecnorm(P_W + R_2W.' * t_2W,2) < state.maxDistance;
                isFarEnough = vecnorm(P_W + R_2W.' * t_2W,2) > state.minDistance;   %ADDED minimum distance 01/01
                
                valid = isInFront1 & isInFront2 & isNear & isFarEnough;
                stillCandidatesKp = [stillCandidatesKp, candidates(:,~valid)];
                stillCandidatesLs = [stillCandidatesLs, lastSeen(:,~valid)];
                candidates = candidates(:,valid);
                lastSeen = lastSeen(:,valid);
                P_W = P_W(:,valid);
                
                % TODO reprojection error check
                
                [~, landmarksIdx, mask] = addLandmarks(state, P_W);
                stillCandidatesKp = [stillCandidatesKp, candidates(:,~mask)];
                stillCandidatesLs = [stillCandidatesLs, lastSeen(:,~mask)];
                candidates = candidates(:,mask);
                lastSeen = lastSeen(:,mask);
                
                % 3. Add observation
                state.addLandmarksToPose(fsFrameIdx, landmarksIdx, candidates.');
                state.addLandmarksToPose(lsFrameIdx, landmarksIdx, lastSeen.');
                
                confirmedKp = [confirmedKp, lastSeen];
                confirmedLandmarks = [confirmedLandmarks, P_W];
                stillCandidates = stillCandidatesLs;

                % 4. Remove triangulated candidates
                state.Candidates(idx,:) = [];
                if ~isempty(stillCandidatesLs)
                    state.Candidates = [state.Candidates;
                    table(repmat(fsFrameIdx, size(stillCandidatesLs, 2), 1), ...
                        stillCandidatesKp.', stillCandidatesLs.', ...
                        'VariableNames', {'FirstId', 'Keypoint', 'LastSeen'})];
                end
                
                % Remove all candidates from frames too far away
%                 state.Candidates(...
%                     state.Candidates.FirstId < lsFrameIdx - state.candidateWindow, :) = [];
            end
            
            if state.verbose
                N = state.getNumberOfLandmarksTrackedAtFrame(lsFrameIdx);
                verboseDisp(state.verbose, ...
                    'Tracking %d landmarks at frame %d.\n', [N, lsFrameIdx]);
            end
        end
        
        function [hiddenState, observations, bundleIdx] ...
                = getOptimizationDS(state, bundleSize)
            % GETBUNDLEADJUSTMENTDS Returns a convenient data structure to
            % perform bundle adjustment.
            %
            % TODO update documentation and allow to slide window
            % 
            % [hiddenState, observations] = state.getBundleAdjustmentDS()
            % returns a convenient data structure to perform bundle
            % adjustment from all the currently stored poses.
            %
            % hiddenState is [tau_1.', ..., tau_n.', P_1.', ..., P_m.'],
            % where tau_i is the twist vector, describing the i-th camera 
            % pose and P_i is the 3D position of the i-th landmark.
            %
            % observations is [n, m, O_1.', ..., O_n.'], where n is the
            % number of frames, m is the number of landmarks and O_i is 
            % [k_i, p_i1.', ..., p_ik_i.', li1, ..., lik_i], where k_i is
            % the number landmarks observed at the i-th frame, p_ij is the
            % j-th keypoints found in the i-th frame and lij is the index
            % of the landmark corresponding to the j-th keypoint of the
            % i-th frame.
            %
            % state.getBundleAdjustmentDS(poseIndices) additionally allows
            % to specify which poses to consider when building the
            % data structure.
            
            arguments
                state
                bundleSize = Inf
            end
            
            nPoses = size(state.Poses, 1);
            bundleSize = min(bundleSize, nPoses);
            poseIdx = nPoses-bundleSize+1:nPoses;
            frameIdx = state.Poses.Id(poseIdx);
            
            hiddenState = [];
            observations = [];
            
            [posenids, ~] = find(state.ObservationGraph.Nodes.PoseId == frameIdx.');
            landmarksIdx = [];
            for i = 1:bundleSize
                R_CW = reshape(state.Poses.R_CW(poseIdx(i), :), 3, 3);
                t_WC = reshape(state.Poses.Position(poseIdx(i), :), 3, 1);
                
                T_WC = [R_CW.', t_WC; 0, 0, 0, 1];
                
                tau_i = homogMatrix2twist(T_WC);
                hiddenState = [hiddenState; tau_i];
                
                [leids, lnids] = outedges(state.ObservationGraph, posenids(i));
                landmarksIdx_i = state.ObservationGraph.Nodes.LandmarkId(lnids);
                landmarksIdx = [landmarksIdx; landmarksIdx_i];
                keypoints = state.ObservationGraph.Edges.Keypoints(leids, :).';
                
                observations = [observations; numel(lnids); 
                    reshape(keypoints, [], 1); landmarksIdx_i];
            end
            
            % So that all observations points to the same landmarks
            landmarksIdx = unique(landmarksIdx);
            j = 1;
            for i = 1:bundleSize
                k_i = observations(j);
                landmarksIdx_i = observations(j+2*k_i+1: j+3*k_i);
                [l_i, ~] = find(landmarksIdx == landmarksIdx_i.');
                observations(j+2*k_i+1: j+3*k_i) = l_i;
                
                j = j+3*k_i+1;
            end
            
            [idx, ~] = find(state.Landmarks.Id == landmarksIdx.'); 
            landmarks = state.Landmarks.Position(idx, :).';
            
            hiddenState = [hiddenState; reshape(landmarks, [], 1)];
            observations = [bundleSize; size(landmarks, 2); observations];
            
            bundleIdx = [numel(poseIdx); 
                poseIdx.'; landmarksIdx];
        end
        
        function optimizedBundle(state, hiddenState, bundleIdx)
            nPoses = bundleIdx(1);
            poseIdx = bundleIdx(2:2+nPoses-1);
            landmarksIdx = bundleIdx(2+nPoses:end);
            
            for i = 1:nPoses
                tau_i = hiddenState(6*(i-1)+1:6*i);
                T_WC = twist2HomogMatrix(tau_i);
                R_CW = T_WC(1:3,1:3).';
                t_WC = T_WC(1:3,end);
                t_CW = -R_CW * t_WC;
                
                state.Poses.R_CW(poseIdx(i), :) = reshape(R_CW, 1, []);
                state.Poses.t_CW(poseIdx(i), :) = reshape(t_CW, 1, []);
                state.Poses.Position(poseIdx(i), :) = reshape(t_WC, 1, []);
            end
            
            landmarks = reshape(hiddenState(6*nPoses+1:end), 3, []);
            [idx, ~] = find(state.Landmarks.Id == landmarksIdx.');
            state.Landmarks.Position(idx, :) = landmarks.';
        end
    end
end

