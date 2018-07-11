function f = display_env2D(w, rscan, ref_pose, cscan, true_pose, est_pose,
                           fig = 0, title = "Env2D")
  if (fig)
    f = gcf();
  else
    f = figure("name", title, "numbertitle", "off",
               "menubar", "none", "toolbar", "none");
  endif

  axis equal;
  hold off;
  grid on;
  plot(w.segments(:, 1), w.segments(:, 2), "linewidth", 1, "color", "k");
  hold on;

  # blue - ref
  # green - groud truth (curr)
  # red - corrected
  display_robot2D(rscan, ref_pose, 'b');
  display_robot2D(cscan, est_pose, 'r');
  display_robot2D(cscan, true_pose, 'g');

  refresh();
endfunction
