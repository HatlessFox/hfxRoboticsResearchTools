function [scan] = get_scan_view(pose, sensor, world)
  robot.state.gt = pose';
  [scan.localCart scan.localPolar] = multibeamComplex2D(robot, world, sensor);
end
