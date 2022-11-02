function [pitch] = pitch_a(gx,gy,gz)


pitch = atan2(-gx, sqrt(gy.^2+gz.^2));

end

