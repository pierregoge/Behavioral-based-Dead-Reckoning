function [yaw] = yaw_a(bx,by,bz,pitch,roll)

by2 = bz.*sin(roll)-by.*cos(roll);
bz2 = by .* sin(roll) + bz .*cos(roll);
bx3 = bx .* cos(pitch) + bz2.*sin(pitch);
yaw = atan2(by2, bx3);

end

