function [lines, heads] = plotCoordinateFrame (R_IC, origin, len, colors)
    %PLOTCOORDINATEFRAME Plots a 3D coordinate frames taking as inputs the
    %rotation matric R_IC (from camera to inertial frame), the origin
    %coordinates, a length value len (optional), and colours of the arrows (optional)
    
    if nargin < 3
        len = 1;
    end
    if nargin < 4
        colors = ['r', 'g', 'b'];
    end
    
    %Check proper dimensions
    assert(size(R_IC, 1) == 3)
    assert(size(R_IC, 2) == 3)
    assert(size(origin, 1) == 3)
    assert(size(origin, 2) == 1)
    
    R = R_IC';
    
    [lines, heads] = arrow3d(repmat(origin', 3, 1),...
        repmat(origin', 3, 1) + len * R, 15);
    
    set(lines(1, 1), 'facecolor', colors(1, :));
    set(heads(1, 1), 'facecolor', colors(1, :));
    
    set(lines(1, 2), 'facecolor', colors(2, :));
    set(heads(1, 2), 'facecolor', colors(2, :));
    
    set(lines(1, 3), 'facecolor', colors(3, :));
    set(heads(1, 3), 'facecolor', colors(3, :));
    
end