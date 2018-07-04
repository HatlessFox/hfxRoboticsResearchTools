function [ np ] = cartProject( p, r, t )
%CARTPROJECT Using rotation r and translation t project the Polar point p
% P has the form [x y]

if size(p,2) == 2
    p = [p 1];
end

rTn = [cos(r) -sin(r) t(1); sin(r) cos(r) t(2); 0 0 1];
np = (rTn*p')';
np(3) = [];

end

