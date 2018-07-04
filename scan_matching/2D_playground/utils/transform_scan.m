function [tscan] = transform_scan(scan, pose)
  th = pose(3);
  tr = [pose(1) pose(2)]';
  rot = createRotation(th)(1:2, 1:2);
  for i = 1:size(scan.cart, 1)
    tscan.cart(i, :) = rot * scan.cart(i, :)' + tr;
    tscan.polar(i, :) = cart2pol(tscan.cart(i, :));
  end
  tscan.pose = pose;
end
