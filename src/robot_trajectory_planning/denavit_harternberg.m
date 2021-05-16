function A = denavit_harternberg(theta, d, alpha, a)
% DENAVIT_HARTERNBERG Find the rototranslation between two reference frames
% using the Denavit-Hartenberg (DH) parameters.
%
%   A = DENAVIT_HARTERNBERG(theta, d, alpha, a)
%
%   Input arguments:
%   ------------------
%   theta:  DH parameter 'theta'
%   d:      DH parameter 'd'
%   alpha:  DH parameter 'alpha'
%   a:      DH parameter 'a' (also known as 'r')
%
%   Output arguments:
%   ------------------
%   A:      4x4 roto-translation defined by the DH parameters
%
% See also DIRECT_KIN

    A=[cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha)  a*cos(theta);
       sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
       0           sin(alpha)             cos(alpha)            d;
       0           0                      0                     1];
   
end