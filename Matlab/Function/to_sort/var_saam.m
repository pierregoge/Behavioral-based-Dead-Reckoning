function [Pi, Ro, He] = var_saam(var,var2)


if isstruct(var)
    Ag = var.Ag ;
    if ~isfield(var.Ag,'data')
        fprintf('var_saam: input must be a proper sensor gravitationnal acceleration structure\n') ;
        return
    end
    if ~strcmp(var.Ag.sampling,'regular')
        fprintf('decdc: input must be a regularly sampled sensor structure\n') ;
        return
    end
    ax = -var.Ag.data(:,1) ;
    ay = -var.Ag.data(:,2) ;
    az = var.Ag.data(:,3) ;
else
    ax = -var(:,1);
    ay = -var(:,2);
    az = var(:,3);
end

if isstruct(var)
    Mg = var.Mg ;
    if ~isfield(var.Mg,'data')
        fprintf('var_saam: input must be a proper sensor gravitationnal acceleration structure\n') ;
        return
    end
    if ~strcmp(var.Mg.sampling,'regular')
        fprintf('decdc: input must be a regularly sampled sensor structure\n') ;
        return
    end
    mx = var.Mg.data(:,1) ;
    my = var.Mg.data(:,2) ;
    mz = var.Mg.data(:,3) ;
end

if nargin > 1
    mx = var2(:,1) ;
    my = var2(:,2) ;
    mz = var2(:,3) ;
end


% % Measures definition (Normalization of magnetic vector meas., Accel vector
% % meas. already normalized)
% norm_acc_meas = sqrt(ax.^2 + ay.^2 + az.^2);
% ax =  ax./norm_acc_meas;
% ay =  ay./norm_acc_meas;
% az =  az./norm_acc_meas;
% norm_magn_meas = sqrt(mx.^2 + my.^2 + mz.^2);
% mx =  mx./(norm_magn_meas);
% my =  my./(norm_magn_meas);
% mz =  mz./(norm_magn_meas);
% 
% md = ax.*mx + ay.*my + az.*mz;
% mn = sqrt(1 - md.^2);
% 
% % % Quaternion calculation
% q0 = -ay.*(mn + mx) + ax.*my;
% q1 = (az - 1).*(mn + mx) + ax.*(md - mz);
% q2 = (az - 1).*my + ay.*(md - mz);
% q3 = az.*md -ax.*mn - mz;
% 
% % Quaternion normalization
% q_SAAM(:,1) = q0 ./ norm([q0; q1; q2; q3]);
% q_SAAM(:,2) = q1 ./ norm([q0; q1; q2; q3]);
% q_SAAM(:,3) = q2 ./ norm([q0; q1; q2; q3]);
% q_SAAM(:,4) = q3 ./ norm([q0; q1; q2; q3]);
% 
% q_SAAM = quaternion(q_SAAM(:,1),q_SAAM(:,2),q_SAAM(:,3),q_SAAM(:,4));
% euler_rad = euler(q_SAAM,'XYZ','frame');

[euler_rad(:,1)] = pitch_a(ax,ay,az);
[euler_rad(:,2)] = roll_a(ay,az);
[euler_rad(:,3)] = yaw_a(mx,my,mz,euler_rad(:,1),euler_rad(:,2));


if isstruct(var)   
    %Pitch
    Pi = var.Ag;
    Pi.data = euler_rad(:,1);
    Pi.name = 'Pi';
    Pi.full_name = 'Pitch';
    Pi.unit = 'rad';
    
    %Roll
    Ro = var.Ag;
    Ro.data = euler_rad(:,2);
    Ro.name = 'Ro';
    Ro.full_name = 'Roll';
    Ro.unit = 'rad';
    
    %Heading
    He = var.Ag;
    He.data = euler_rad(:,3);
    He.name = 'He';
    He.full_name = 'Heading';
    He.unit = 'rad';
        
else
    Pi = euleur_rad(:,1);
    Ro = euleur_rad(:,2);
    He = euleur_rad(:,3);
end

end