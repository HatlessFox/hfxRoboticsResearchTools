function [] = display_robot2D(scan, pose, c, scale = 1)
  display_scan2D(transform_scan(scan, pose), c);
  display_pose2D(pose, c, scale);
  hold on;
end
