function [] = display_scan(scan, pose, c)
  tscan = transform_scan(scan, pose);
  plot(tscan.cart(:, 1), tscan.cart(:, 2), ["." c], "markersize", 6)
  hold on;
end
