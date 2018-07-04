function [hist] = make_angle_histogram(scan, resolution)
  pts_nm = size(scan.cart, 1);
  angles = zeros(pts_nm - 1, 1);
  for i = 1:pts_nm - 1
    d_cart = scan.cart(i, :) - scan.cart(i + 1, :);
    # TODO: filter holes
    ## if (1 < d_cart * d_cart')
    ##   continue;
    ## end
    ang = atan2(d_cart(1), d_cart(2)) + pi; # (0, 2*pi]
    if (2*pi - resolution/2 < ang) # ... | .... | .. -> .. 360=0
      ang = 0;
    end
    angles(i) = ang;
  end

  bin_nm = ceil((2*pi)/resolution);
  hist = zeros(bin_nm, 1);
  for i = 1:size(angles, 1)
    bin_i = round(angles(i) / resolution) + 1;
    bin_i = min(bin_i, bin_nm);
    hist(bin_i) += 1;
  end
end
