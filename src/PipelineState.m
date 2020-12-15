classdef PipelineState
    % PIPELINESTATE Embedds the pipeline state, with the history of poses
    % and 
    % 
    %   Detailed explanation goes here
    
    properties (Access = private)
        Poses = table() % Id | Pose (Nx12: T_WC(:).') % TODO twist vector
        Landmarks = table() % Id | Position (Mx3: [X, Y, Z])
        ObservationGraph = graph % Poses x Landmarks -> (keypoints, descriptors)
        Candidates = table() % FrameId | Keypoints (Cx2: [u, v]) | Descriptors (CxM)
        LastFrame = 0; % Index of last frame add
        LastLandmark = 0; % Label of last landmark add
        cos_th = 0.1; % Threshold for triangulation TODO make configurable
        K % The intrinsics matrix
    end
    
    methods
        function obj = PipelineState(K)
            obj.K = K;
        end
        
        function poses = getPoses(state)
            % TODO
        end
        
        function state = addPose(state, pose, landmarksIndices, keypoints, descriptors)
            % ADDPOSE Adds a new pose to the observation graph.
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
            
            state.LastFrame = state.LastFrame + 1;
            frameIndex = state.LastFrame;
            newPose = table(frameIndex, Inf, 'VariableNames', {'PoseId', 'LandmarkId'});
            state.ObservationGraph = addnode(state.ObservationGraph, newPose);
            [n, ~, ~] = find(state.ObservationGraph.Nodes.PoseId == frameIndex);
            
            state.Poses = [state.Poses; ...
                table(frameIndex, reshape(pose,1,[]), ...
                'VariableNames', {'Id', 'Pose'})];
            
            % add edges between pose and landmarks
            edgeTable = table(...
                [repmat(n, length(landmarksIndices), 1) landmarksIndices], ...
                keypoints, descriptors, ...
                'VariableNames', {'EndNodes' 'Keypoints' 'Descriptors'});
            state.ObservationGraph = addedge(...
                state.ObservationGraph, edgeTable);
        end
        
        function [state, landmarksIndices] = addLandmarks(state, pose, ...
                landmarks, keypoints, descriptors)
            % ADDLANDMARKS adds landmarks observed at the given pose.
            %
            % state = state.ADDLANDMARKS(pose, landmarks, keypoints,
            % descriptors) adds the given pose and the landmarks (Nx3)
            % provided in the observation graph, connecting them to the
            % newly add pose through the provided keypoints (Nx2 [u v]) and
            % descriptors (NxM).
            %
            % [state, landmarksIndices] = state.ADDLANDMARKS(...)
            % additionally returns the labels of the landamarks, to be used
            % for instance with addPose.
            %
            % See also addPose, getLandmarks.
            
            nLandmarks = size(landmarks, 1);
            landmarksLabels = (state.LastLandmark + 1 : state.LastLandmark + nLandmarks).';
            state.LastLandmark = state.LastLandmark + nLandmarks;
            
            state.Landmarks = [state.Landmarks; ...
                table(landmarksLabels, landmarks, ...
                'VariableNames', {'Id', 'Position'})];
            
            newLandmarks = table(Inf(nLandmarks, 1), landmarksLabels,...
                'VariableNames', {'PoseId', 'LandmarkId'});
            state.ObservationGraph = addnode(state.ObservationGraph, newLandmarks);
            
            [landmarksIndices, ~, ~] = find(state.ObservationGraph.Nodes.LandmarkId == landmarksLabels.');
            state = state.addPose(pose, landmarksIndices, keypoints, descriptors);
        end
        
        function [landmarks, landmarksLabels, descriptors] = getLandmarks(state, poseId)
            % GETLANDMARKS
            %
            % TODO COMMENT
            %
            % See also addLandmarks, addPose.
            
            poseNode = find(state.ObservationGraph.Nodes.PoseId == poseId);
            [eid, landmarksLabels] = outedges(state.ObservationGraph, poseNode);
            [landmarksIdx, ~, ~] = find(state.Landmarks.Id == landmarksLabels.');
            landmarks = state.Landmarks.Position(landmarksIdx, :);
            descriptors = state.ObservationGraph.Edges{eid, 'Descriptors'};
        end
        
        function state = addCandidates(state, frameIndex, keypoints, descriptors)
            % ADDCANDIDATES
            %
            % TODO COMMENT
            %
            % See also getCandidates, evaluateCandidates, pruneCandidates
            
            N = size(keypoints, 1);
            state.Candidates = [state.Candidates; ...
                table(repmat(frameIndex, N, 1), keypoints, descriptors, ...
                'VariableNames', {'PoseId', 'Keypoints', 'Descriptors'})];
        end
        
        function [candidateLabels, keypoints, descriptors] = getCandidates(state, posesIndices)
            % GETCANDIDATES Returns candidates that can be used to
            % triangulate new landmarks.
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
            
            posesIndices = reshape(posesIndices, 1, []);
            [candidateLabels, ~, ~] = find(state.Candidates.PoseId == posesIndices);
            keypoints = state.Candidates.Keypoints(candidateLabels);
            descriptors = state.Candidates.Descriptors(candidateLabels);
        end
        
        function state = evaluateCandidates(state, frameIndex, ...
                matchLabels, keypoints, descriptors, landmarks)
            % EVALUATECANDIDATES Updates the observation graph and the
            % candidates database given the found matches.
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
            
            % 1. Validation
            N = length(matchLabels);
            oldPoses = state.Candidates.PoseId(matchLabels);
            keypointsMatches = state.Candidates.Keypoints(matchLabels);
            bearings1 = normalize(state.K \ transpose([keypointsMatches, ones(N, 1)]), 'norm');
            bearings2 = normalize(state.K \ transpose([keypoints, ones(N, 1)]), 'norm');
            validMatches = reshape(abs(dot(bearings1, bearings2)) < state.cos_th, [], 1);
            oldPoses = oldPoses(validMatches);
            keypointsMatches = keypointsMatches(validMatches);
            descriptorsMatches = state.Candidates.Descriptors(matchLabels(validMatches));
            % 2. Save old landmark
            [state, landmarkLabels] = state.addLandmarks(...
                oldPoses, landmarks, keypointsMatches, descriptorsMatches);
            % 3. Add observation
            state = state.addPose(frameIndex, landmarkLabels, keypoints, descriptors);
        end
        
        function state = pruneCandidates(state, minPoseIndex)
            % PRUNE Prune the candidates table.
            % 
            % state = state.pruneCandidates(minPoseIndex) remove all 
            % candidates from poses associated to an index less or equal to
            % minPoseIndex.
            %
            % See also addCandidates, getCandidates, evaluateCandidates
            % 
            % TODO maxDistance based pruning
            
            % Candidates
            state.Candidates(state.Candidates.PoseId < minPoseIndex, :) = [];
        end

        function state = prune(state, minPoseIndex)
            % PRUNE Prune the observation graph.
            % 
            % state = state.prune(minPoseIndex) remove all poses with an 
            % index less or equal to minPoseIndex. Removes also all
            % landmarks that will remain unobserved, i.e., such that there
            % are no poses stored in which the landmarks have been obseved,
            % i.e., nodes with zero degree in the observation graph.
            % 
            % TODO maxDistance based pruning
            % TODO save on file before pruning
            
            % Remove poses
            posesToRemove = state.Poses.Id(state.Poses.Id < minPoseIndex);
            [poseNodesToRemove, ~, ~] = find(state.ObservationGraph.Nodes.PoseId == posesToRemove.');
            state.ObservationGraph = rmnode(state.ObservationGraph, poseNodesToRemove);
            [poseIdx, ~, ~] = find(state.Poses.Id == posesToRemove.');
            state.Poses(poseIdx, :) = [];
            
            % Remove Landmarks not observed by any pose
            [landmarkNodesToRemove, ~, ~] = find(isfinite(state.ObservationGraph.Nodes.LandmarkId));
            % No observations
            landmarkNodesToRemove = landmarkNodesToRemove(...
                0 == degree(state.ObservationGraph, landmarkNodesToRemove));
            landmarksToRemove = state.ObservationGraph.Nodes.LandmarkId(landmarkNodesToRemove);
            state.ObservationGraph = rmnode(state.ObservationGraph, landmarkNodesToRemove);
            [landmarksIdx, ~, ~] = find(state.Landmarks.Id == landmarksToRemove.');
            state.Landmarks(landmarksIdx, :) = [];
        end
        
        function [hiddenState, observations] = getBundleAdjustmentDS(state, poseIndices)
            % GETBUNDLEADJUSTMENTDS Returns a convenient data structure to
            % perform bundle adjustment.
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
            
            if nargin < 2
                poseIndices = state.Poses.Id;
            end
            poseIndices = reshape(poseIndices, 1, []);
            
            % filter graph
            [nodes, ~, ~] = find(state.ObservationGraph.Nodes.PoseId == poseIndices);
            nodeTable = state.ObservationGraph.Nodes(nodes, :);
            
            [edges, ~, ~] = find(state.ObservationGraph.Edges.EndNodes(:,1) == poseIndices);
            edgeTable = state.ObservationGraph.Edges(edges, :);
            g = graph(edgeTable, nodeTable);
            
            [posesIdx, ~, ~] = find(state.Poses.Id == poseIndices);
            hiddenState = [reshape(state.Poses.Pose(posesIdx, :), 1, []), ...
                reshape(state.Landmarks.Position, 1, [])];
            observations = [length(poseIndices), size(state.Landmarks, 1)];
            for poseIndex = reshape(posesIdx, 1, [])
                [eid, nid] = outedges(g, poseIndex);
                observations = [observations, ...
                    degree(g, poseIndex), ...
                    reshape(g.Edges{eid, 'Keypoints'}.', 1, []), ...
                    g.Nodes.LandmarkId(nid).'];
            end
        end
    end
end

