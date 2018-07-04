function [scan] = get_scan_view(pose, sensor, world)
  robot.state.gt = pose';
  [raw _] = simObservation(robot, sensor, world);
  scan.localCart = raw.data.localCart;
  scan.localPolar = raw.data.localPolar;
end
