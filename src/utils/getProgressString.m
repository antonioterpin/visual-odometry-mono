function progressString = getProgressString(it, maxIt, size)
%GETPROGRESSSTRING Returns a textual progress bar.
%
%GETPROGRESSSTRING(it, maxIt) Returns a textual progress bar filled at
%it/maxIt*100%
%
%GETPROGRESSSTRING(..., size) Additionally, it is possible to configure the
% size of the progress bar.
%
% Example:
%
% disp(getProgressString(5,10,10));
%  |=====     |
%
% for i = 1:4
%   disp(getProgressString(i,4,8));
% end
% |==      |
% |====    |
% |======  |
% |========|

arguments
    it {mustBeNumeric(it)}
    maxIt {mustBeNumeric(maxIt)}
    size {mustBeNumeric(size)} = 10
end

done = floor(it / maxIt * size);
notDone = size - done;
progressString = ['|' repmat('=', 1, done-1) repmat(' ', 1, notDone) '|'];

end

