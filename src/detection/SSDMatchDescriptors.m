function matches = SSDMatchDescriptors(descriptors2, descriptors1, lambda)
% SSDMATCHDESCRIPTORS Matches the given descriptors.
%
% matches = SSDMATCHDESCRIPTORS(descriptors2, descriptors1, lambda) matches
% the descriptors2 (MxN2) to the descriptors1 (MxN1), where M is the
% descriptor dimension. N1 and N2 are the number of descriptors in the
% first and second set respectively.
% matches is a 1xQ matrix, where descriptors2(matches(i)) <->
% descriptors1(i), if matches(i) ~= 0. Otherwise, descriptors1(i) has no
% matches, i.e., the SSD is smaller then lambda*min(SSD). The matches are
% guaranteed to be unique.

arguments
    descriptors2
    descriptors1 {mustBeEqualDimension(descriptors2,descriptors1)}
    lambda {mustBePositive(lambda)}
end

[dists,matches] = pdist2(double(descriptors1.'), double(descriptors2.'), 'euclidean', 'Smallest', 1);

matches(dists >= lambda * min(dists(dists > 0))) = 0;

% remove double matches
tmpMatches = zeros(size(matches));
[~,idx,~] = unique(matches, 'stable');
tmpMatches(idx) = matches(idx);

matches = tmpMatches;

end

function mustBeEqualDimension(a,b)
    if size(a,1) ~= size(b,1)
        eid = 'Size:notEqual';
        msg = 'The dimension of the provided descriptors must be the same.';
        throwAsCaller(MException(eid,msg))
    end
end


