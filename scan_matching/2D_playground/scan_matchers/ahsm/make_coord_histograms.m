function [hist_x, hist_y] = make_coord_histograms(scan, bounds, res)
  min_x = bounds(1, 1);
  min_y = bounds(2, 1);
  max_x = bounds(1, 2);
  max_y = bounds(2, 2);

  bin_x_nm = ceil((max_x - min_x) / res);
  bin_y_nm = ceil((max_y - min_y) / res);
  
  hist_x = zeros(bin_x_nm, 1);
  hist_y = zeros(bin_y_nm, 1);
  for pt_i = 1:size(scan.cart,1)
    x_coord = scan.cart(pt_i, 1);
    y_coord = scan.cart(pt_i, 2);
    
    bin_x_i = min(floor((x_coord - min_x) / res) + 1, bin_x_nm);
    bin_y_i = min(floor((y_coord - min_y) / res) + 1, bin_y_nm);

    hist_x(bin_x_i) += 1;
    hist_y(bin_y_i) += 1;
  end
end

