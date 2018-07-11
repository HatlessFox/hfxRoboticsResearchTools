function [scan] = get_scan_view(pose, scnr, world)
  laser_coord = pose(1:2);
  laser_rotation = createRotation(-pose(3))(1:2, 1:2);

  scan.localCart = [];
  scan.localPolar = [];

  skipped = 0;
  beam_step = scnr.fov / scnr.beams_nm;
  min_beam_angle = -scnr.fov / 2;
  for beam_i = 1:scnr.beams_nm
    angle = normalizeAngle(deg2rad(min_beam_angle + beam_i * beam_step), 0);
    beam_end = [cos(angle) sin(angle)] .* [scnr.max_range scnr.max_range] + ...
               laser_coord;
    beam_segment = [laser_coord; beam_end];
    intersections = intersectPolylines(world.segments.coord, beam_segment);
    if (isempty(intersections))
      skipped += 1;
      continue;
    end
    if (1 < size(intersections, 1))
      # find the closest intersection
      # TODO: refactor
      closest_d = inf;
      for i = 1:size(intersections, 1)
        d = distancePoints(intersections(i, :), laser_coord);
        if (d < closest_d)
          obstacle = intersections(i, :);
          closest_d = d;
        end
      end
    else
      obstacle = intersections;
    end
    # TODO: filter by max_range and min_range
    # TODO: outliers
    is_noisy = randn^2 < scnr.outlier_prob;
    if is_noisy
      laser_coord *= randn;
    end

    noise = (obstacle - laser_coord) * 0; # d * stdErr(0, noise)
    obstacle += noise;

    # TODO: refactor
    laser_transform = (laser_rotation * (-laser_coord)')';
    local_cart = (laser_rotation * obstacle')' + laser_transform;

    scan.cart(beam_i - skipped, :) = local_cart;
    scan.polar(beam_i - skipped, :) = cart2pol(local_cart);
  end
end
