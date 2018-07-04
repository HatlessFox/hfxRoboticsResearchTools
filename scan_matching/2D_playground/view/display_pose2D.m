function [] = display_pose2D(pose, c, scale = 1)
  radius = 0.75 * scale;
  center = pose(1:2);
  drawCircle(center, radius, "color", c, "linewidth", 2);

  dir_end = (createRotation(pose(3)) * [radius; 0; 1])'(1:2) + center;
  line_coords = [center; dir_end];
  line(line_coords(:, 1), line_coords(:, 2), "color", c, "linewidth", 2);

  hold on;
end
