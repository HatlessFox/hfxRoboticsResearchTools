function [ trpt ] = graphicsRobot( pos)
%DISPLAYROBOT Display robot shape

global Opt
scale = Opt.plot.robot_scale;

yaw = pos(3);
pos = [pos(1:2)];

trp = [ -1 -0.5; 1 0; -1 0.5];
trp = (trp) * scale;

rot = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
trpt = [ rot * trp(1,:)' + pos  ...
         rot * trp(2,:)' + pos  ...
         rot * trp(3,:)' + pos ];
end


