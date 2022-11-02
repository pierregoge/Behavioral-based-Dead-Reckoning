function dives = f_seq2dives(T,seq)

nb_dives = length(T.start);

i=1;
dives(i,1) = seq;
dives(i,1).tstart = 0;
dives(i,1).tend = (T.start(i,1))-(1/dives(i,1).fs_max);
dives(i,1).type = 'dive';

lin_inf = 1;
lin_sup = T.start(i,1)*dives(i,1).fs_max-1;

dives(i,1).var.A.data = dives(i,1).var.A.data(lin_inf:lin_sup,:);
dives(i,1).var.M.data = dives(i,1).var.M.data(lin_inf:lin_sup,:);
dives(i,1).var.G.data = dives(i,1).var.G.data(lin_inf:lin_sup,:);
dives(i,1).var.P.data = dives(i,1).var.P.data(lin_inf:lin_sup,:);
dives(i,1).var.Ag.data = dives(i,1).var.Ag.data(lin_inf:lin_sup,:);
dives(i,1).var.Ad.data = dives(i,1).var.Ad.data(lin_inf:lin_sup,:);
dives(i,1).var.Ro.data = dives(i,1).var.Ro.data(lin_inf:lin_sup,:);
dives(i,1).var.Pi.data = dives(i,1).var.Pi.data(lin_inf:lin_sup,:);
dives(i,1).var.He.data = dives(i,1).var.He.data(lin_inf:lin_sup,:);
dives(i,1).var.O.data = dives(i,1).var.O.data(lin_inf:lin_sup,:);
dives(i,1).var.J.data = dives(i,1).var.J.data(lin_inf:lin_sup,:);

for i=2:nb_dives-1
    
    dives(i,1) = seq;
    dives(i,1).tstart = T.start(i-1,1);
    dives(i,1).tend = (T.start(i,1))-(1/dives(i,1).fs_max);
    dives(i,1).type = 'dive';
    
    lin_inf = T.start(i-1,1)*dives(i,1).fs_max;
    lin_sup = T.start(i,1)*dives(i,1).fs_max-1;
    
    dives(i,1).var.A.data = dives(i,1).var.A.data(lin_inf:lin_sup,:);
    dives(i,1).var.M.data = dives(i,1).var.M.data(lin_inf:lin_sup,:);
    dives(i,1).var.G.data = dives(i,1).var.G.data(lin_inf:lin_sup,:);
    dives(i,1).var.P.data = dives(i,1).var.P.data(lin_inf:lin_sup,:);
    dives(i,1).var.Ag.data = dives(i,1).var.Ag.data(lin_inf:lin_sup,:);
    dives(i,1).var.Ad.data = dives(i,1).var.Ad.data(lin_inf:lin_sup,:);
    dives(i,1).var.Ro.data = dives(i,1).var.Ro.data(lin_inf:lin_sup,:);
    dives(i,1).var.Pi.data = dives(i,1).var.Pi.data(lin_inf:lin_sup,:);
    dives(i,1).var.He.data = dives(i,1).var.He.data(lin_inf:lin_sup,:);
    dives(i,1).var.O.data = dives(i,1).var.O.data(lin_inf:lin_sup,:);
    dives(i,1).var.J.data = dives(i,1).var.J.data(lin_inf:lin_sup,:);
    
end


i=nb_dives;
dives(i,1) = seq;
dives(i,1).tstart = T.start(i-1,1);
dives(i,1).tend = (T.start(i,1))-(1/dives(i,1).fs_max);
dives(i,1).type = 'dive';

lin_inf = T.start(i-1,1)*dives(i-1,1).fs_max;
lin_sup = T.start(i,1)*dives(i,1).fs_max-1;

dives(i,1).var.A.data = dives(i,1).var.A.data(lin_inf:lin_sup,:);
dives(i,1).var.M.data = dives(i,1).var.M.data(lin_inf:lin_sup,:);
dives(i,1).var.G.data = dives(i,1).var.G.data(lin_inf:lin_sup,:);
dives(i,1).var.P.data = dives(i,1).var.P.data(lin_inf:lin_sup,:);
dives(i,1).var.Ag.data = dives(i,1).var.Ag.data(lin_inf:lin_sup,:);
dives(i,1).var.Ad.data = dives(i,1).var.Ad.data(lin_inf:lin_sup,:);
dives(i,1).var.Ro.data = dives(i,1).var.Ro.data(lin_inf:lin_sup,:);
dives(i,1).var.Pi.data = dives(i,1).var.Pi.data(lin_inf:lin_sup,:);
dives(i,1).var.He.data = dives(i,1).var.He.data(lin_inf:lin_sup,:);
dives(i,1).var.O.data = dives(i,1).var.O.data(lin_inf:lin_sup,:);
dives(i,1).var.J.data = dives(i,1).var.J.data(lin_inf:lin_sup,:);


i=nb_dives+1;
dives(i,1) = seq;
dives(i,1).tstart = T.start(i-1,1);
dives(i,1).tend = seq.tend;
dives(i,1).type = 'dive';

lin_inf = T.start(i-1,1)*dives(i-1,1).fs_max;
lin_sup = seq.tend*dives(i-1,1).fs_max-1;

dives(i,1).var.A.data = dives(i,1).var.A.data(lin_inf:lin_sup,:);
dives(i,1).var.M.data = dives(i,1).var.M.data(lin_inf:lin_sup,:);
dives(i,1).var.G.data = dives(i,1).var.G.data(lin_inf:lin_sup,:);
dives(i,1).var.P.data = dives(i,1).var.P.data(lin_inf:lin_sup,:);
dives(i,1).var.Ag.data = dives(i,1).var.Ag.data(lin_inf:lin_sup,:);
dives(i,1).var.Ad.data = dives(i,1).var.Ad.data(lin_inf:lin_sup,:);
dives(i,1).var.Ro.data = dives(i,1).var.Ro.data(lin_inf:lin_sup,:);
dives(i,1).var.Pi.data = dives(i,1).var.Pi.data(lin_inf:lin_sup,:);
dives(i,1).var.He.data = dives(i,1).var.He.data(lin_inf:lin_sup,:);
dives(i,1).var.O.data = dives(i,1).var.O.data(lin_inf:lin_sup,:);
dives(i,1).var.J.data = dives(i,1).var.J.data(lin_inf:lin_sup,:);

end

