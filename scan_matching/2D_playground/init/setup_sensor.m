function [sensor] = setup_sensor(dist_noise = 0)
  sensor.fov = 360;
  sensor.beams_nm = 360;

  sensor.min_range = 0.3;
  sensor.max_range = 100;

  sensor.dist_noise = dist_noise;
  sensor.outlier_prob = 0.0;
end
