function [] = display_scan2D(scan, c)
  plot(scan.cart(:, 1), scan.cart(:, 2), ["." c], "markersize", 6)
end
