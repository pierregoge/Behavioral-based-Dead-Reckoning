function [roll] = roll_a(gy, gz, varargin)

alpha = 0.025;

    
roll = atan2(gy, gz);

end

