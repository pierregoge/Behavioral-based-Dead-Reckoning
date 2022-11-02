function [seq_out] = seq_ptrack2geo(seq,p_start,seq_ref)



if ~isstruct(p_start)

if p_start(1,:) < 2
    if nargin == 3 && isstruct(seq_ref)
        pos_start(1,1) = seq_ref.var.LAT.data(p_start,1);
        pos_start(1,2) = seq_ref.var.LON.data(p_start,1);
    else
        fprintf('seq_ptrack2geo: If p_start is 1x1 need to add seq_ref as structure in input \n') ;
        help seq_ptrack2geo ;
        return
    end
elseif p_start(1,:) == 2
    pos_start(1,1) = p_start(1,1);
    pos_start(1,2) = p_start(1,2);

else    
    fprintf('seq_ptrack2geo: p_start input need to be 1x1 or 1x2 scallar\n') ;
    help seq_ptrack2geo ;
    return
end

else
    pos_start(1,1) = p_start.var.LAT.data(1,1) ;
    pos_start(1,2) = p_start.var.LON.data(1,1) ;
end

if nargin == 3 && ~isstruct(seq_ref)
   fprintf('seq_ptrack2geo: seq_ref need to be a proper structure \n') ;
   help seq_ptrack2geo ;
   return
end

if isstruct(seq)
	X = seq ;
	if ~isfield(seq,'var')
		fprintf('seq_ptrack2geo: input must be a proper structure with var field\n') ;
        help seq_ptrack2geo ;
	end
	
	x = seq.var.X.data ;
    y = seq.var.Y.data;
end

x_start = deg2km(pos_start(1,1));
y_start = deg2km(pos_start(1,2));

% Geolocate traj with the first GPS position of the analysis
lat_traj = km2deg(x_start+(-y/1000));
lon_traj = km2deg(y_start+(-x/1000));

X.var.LAT = X.var.X;
X.var.LAT.data = lat_traj;
X.var.LAT.name = 'LAT';
X.var.LAT.full_name = 'Lattitude';

X.var.LON = X.var.Y;
X.var.LON.data = lon_traj;
X.var.LON.name = 'LON';
X.var.LON.full_name = 'Longitude';

seq_out = X ;


end

