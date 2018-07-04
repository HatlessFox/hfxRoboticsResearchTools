function [] = display_scan(scan, pose, c)
  tscan = transform_scan(scan, pose);
  displayPoints(tscan.cart, c);
  hold on;

  g_rob = graphicsRobot([pose(1) pose(2) pose(3)]);
  fill(g_rob(1, :), g_rob(2, :), c);
  hold on;
end
