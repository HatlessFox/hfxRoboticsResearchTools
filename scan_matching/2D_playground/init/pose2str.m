function s = pose2str(pose)
  s = sprintf("{x: %.2f; y: %.2f; th: %.2f}",
              pose(1), pose(2), rad2deg(pose(3)));
endfunction
