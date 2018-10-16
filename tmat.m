function TMAT = tmat(theta, d, a, alpha)
    % This function computes the transformation matrix necessary for solving the kinematic problems
    % Returns the homogeneous transformation matrix
    % using standard D-H Convention
    % Angles must be given in degrees but used in radians
    
    theta = theta*pi/180;
    alpha = alpha*pi/180;
    ct = cos(theta);
    st = sin(theta);
    ca = cos(alpha);
    sa = sin(alpha);
    %My Version
    TMAT = [ct -st*ca st*sa a*ct; st ct*ca -ct*sa a*st; 0 sa ca d; 0 0 0 1];
end