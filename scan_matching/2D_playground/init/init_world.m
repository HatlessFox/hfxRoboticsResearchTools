function [world] = init_world(ww)
  segments = ww.walls';

  world.points.id = (1:1);
  world.points.coord = zeros(3, 0);

  world.segments.id    = 1 + (1:size(segments, 2)); 
  world.segments.coord = segments;
  world.surface = [];

  world.lims.xMin = min(segments(1, :));
  world.lims.xMax = max(segments(1, :));
  world.lims.yMin = min(segments(2, :));
  world.lims.yMax = max(segments(2, :));
  world.lims.zMin  = -20; # ?
  world.lims.zMax  = 20; # ?

  # dimensions
  world.dims.l = world.lims.xMax - world.lims.xMin;
  world.dims.w = world.lims.yMax - world.lims.yMin;
  world.dims.h = world.lims.zMax - world.lims.zMin;

  # center
  world.center.xMean   = (world.lims.xMax + world.lims.xMin)/2;
  world.center.yMean   = (world.lims.yMax + world.lims.yMin)/2;
  world.center.zMean   = (world.lims.zMax + world.lims.zMin)/2;
end
