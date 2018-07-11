function f = display_env2D(w, rscan, ref_pose, cscan, true_pose, est_pose,
                          title = "Env2D")
  f = figure("name", title, "numbertitle", "off",
             "menubar", "none", "toolbar", "none");
  axis equal;

  plot(w.walls(1,:), w.walls(2,:), "linewidth", 1, "color", "k");
  hold on;
  grid on;

  # blue - ref
  # green - groud truth (curr)
  # red - corrected
  display_robot2D(rscan, ref_pose, 'b');
  display_robot2D(cscan, est_pose, 'r');
  display_robot2D(cscan, true_pose, 'g');
endfunction
