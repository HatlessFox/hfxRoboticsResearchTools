function [tscan] = transform_scan(scan, pose)
  tr = [pose(1); pose(2)];
  rot = createRotation(pose(3))(1:2, 1:2);
  tscan.cart = (rot * scan.cart' + tr)';
  tscan.polar = cart2pol(tscan.cart);
  tscan.pose = pose;
end
