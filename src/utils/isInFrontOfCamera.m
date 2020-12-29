function isInFront = isInFrontOfCamera(landmark, R_CW, t_CW)
%ISINFRONTOFCAMERA Summary of this function goes here
%   Detailed explanation goes here

arguments
    landmark (3,:)
    R_CW (3,3)
    t_CW (3,1)
end

isInFront = R_CW(3,:) * landmark > -t_CW(3);
            
end

