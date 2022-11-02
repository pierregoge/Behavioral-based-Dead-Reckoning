function behavior_timing(seq)

len = length(seq.var.B2.data(1000:end,1));

bev = seq.var.B2.data(:,:);

inc = zeros(1,6);
inc0 = zeros(1,4);
inc1 = zeros(1,4);
inc3 = zeros(1,3);
inc4 = zeros(1,3);
inc5 = zeros(1,1);
inc6 = zeros(1,2);
inc7 = zeros(1,1);

len_active = 0;

for i=2:len
    
    if (bev(i,1) == 2 || bev(i,1) == 6) && bev(i,2) == 1
        bev(i,2) = 0;
    end
    
    if bev(i,1) == 1
        inc(1,1) = inc(1,1) + 1;
    elseif bev(i,1) == 2
        inc(1,2) = inc(1,2) + 1;
    elseif bev(i,1) == 3
        inc(1,3) = inc(1,3) + 1;
    elseif bev(i,1) == 4
        inc(1,4) = inc(1,4) + 1;
    elseif bev(i,1) == 5
        inc(1,5) = inc(1,5) + 1;
    elseif bev(i,1) == 6
        inc(1,6) = inc(1,6) + 1;
    end
    
    if bev(i,1) == 1 || bev(i,1) == 3 || bev(i,1) == 4
        len_active = len_active+1;
    if bev(i,2) == 1
        inc0(1,1) = inc0(1,1) + 1;
    elseif bev(i,2) == 3
        inc0(1,2) = inc0(1,2) + 1;
    elseif bev(i,2) == 4
        inc0(1,3) = inc0(1,3) + 1;
    elseif bev(i,2) == 0
        inc0(1,4) = inc0(1,4) + 1;
    end
    end
    
    
    if bev(i,1) == 1
        if bev(i,2) == 1
            inc1(1,1) = inc1(1,1) + 1;
        elseif bev(i,2) == 3
            inc1(1,2) = inc1(1,2) + 1;
        elseif bev(i,2) == 4
            inc1(1,3) = inc1(1,3) + 1;
        elseif bev(i,2) == 0
            inc1(1,4) = inc1(1,4) + 1;
        end
    end
    
    if bev(i,1) == 3
        if bev(i,2) == 1
            inc3(1,1) = inc3(1,1) + 1;
        elseif bev(i,2) == 3
            inc3(1,2) = inc3(1,2) + 1;
        elseif bev(i,2) == 0
            inc3(1,3) = inc3(1,3) + 1;
        end
        
    end
    
    if bev(i,1) == 4
        if bev(i,2) == 1
            inc4(1,1) = inc4(1,1) + 1;
        elseif bev(i,2) == 4
            inc4(1,2) = inc4(1,2) + 1;
        elseif bev(i,2) == 0
            inc4(1,3) = inc4(1,3) + 1;
        end   
    end  
    
    if seq.var.Pi.data(i,1)*180/pi > 20 || seq.var.Pi.data(i,1) *180/pi < -20
        inc5 = inc5 + 1;
    end
    
     if bev(i,2) == (3 || 4) && bev(i-1,2) ~= (3 || 4)
         inc6 = inc6+1;
     end
      if bev(i,2) == 3
         inc7 = inc7+1;
      end
       if bev(i,2) == 4
         inc7 = inc7+1;
      end
     
      
end

inc = inc/len*100;
inc0 = inc0/len_active*100;
%inc0 = inc0/len*100
inc1 = inc1/len*100;
inc3 = inc3/len*100;
inc4 = inc4/len*100;
inc5 = inc5/len*100;

