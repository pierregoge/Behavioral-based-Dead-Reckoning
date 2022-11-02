function [GEO_POS] = surf_geo_pos(seq_c_track,seq_c_ref)

surf = find_dives(seq_c_track.var.P,0.2);

len = length(surf.start(:,1));

inc = 1;

for i=1:len
    
    t =  surf.start(i,1);
    
    if ~isnan(seq_c_ref.var.X.data(t,1))
        
        GEO_POS(inc,1) = t;
        GEO_POS(inc,2) = seq_c_ref.var.X.data(t,1);
        GEO_POS(inc,3) = seq_c_ref.var.Y.data(t,1);
        inc = inc + 1;
    else
        j=0;
        while j<10   
            if~isnan(seq_c_ref.var.X.data(t+j,1))
                GEO_POS(inc,1) = t;
                GEO_POS(inc,2) = seq_c_ref.var.X.data(t+j,1);
                GEO_POS(inc,3) = seq_c_ref.var.Y.data(t+j,1);
                inc = inc + 1;
                j=10;
      
            else
                if~isnan(seq_c_ref.var.X.data(t-j,1))
                    GEO_POS(inc,1) = t;
                    GEO_POS(inc,2) = seq_c_ref.var.X.data(t-j,1);
                    GEO_POS(inc,3) = seq_c_ref.var.Y.data(t-j,1);
                    inc = inc + 1;
                    j=10;
                end
            end
            j = j+1;
        end
    end
    
end
end

