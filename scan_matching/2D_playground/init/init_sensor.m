function [sensor] = init_sensor()
  sensor.id = 1;
  sensor.name = 'Sonar 2D';
  sensor.type = 'Multibeam2D';
  
  sensor.raw.localCart = sensor.raw.localPolar = [];

  sensor.par.maxWidth = 360; # FoV
  sensor.par.beamWidth = deg2rad(3); # ?
  sensor.par.nBeams = 360; # number of beams (?)
  sensor.par.minRange = 0.3;
  sensor.par.maxRange = 100;
  sensor.par.beamStd = 1*[0.00; 0.000000; 0.000000]; # ?
  sensor.par.i = 0; # ?
  sensor.par.dir = 1; # ?
  sensor.par.dataPerScan = 360; # ? diff ws nBeams?
  sensor.par.outliers = 0.0; # outlier probability
end
