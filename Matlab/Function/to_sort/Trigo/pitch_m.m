function [pitch] = pitch_m(gx,gy,gz)

pitch = atan2(-gz,gx);

end

