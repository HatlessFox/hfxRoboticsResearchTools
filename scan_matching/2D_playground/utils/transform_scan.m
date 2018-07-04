function [tscan] = transform_scan(scan, pose)
  th = pose(3);
  tr = [pose(1) pose(2)]';
  rot = [cos(th) -sin(th); sin(th) cos(th)];
  for i = 1:size(scan.localCart, 1)
    tscan.cart(i, :) = rot * scan.localCart(i, :)' + tr;
    tscan.polar(i, :) = cart2pol(tscan.cart(i, :));
  end
  tscan.pose = pose;
end
