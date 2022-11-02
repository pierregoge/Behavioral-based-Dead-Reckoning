nb = 3;

% Fonction used from tag utils : 
seq_b = downs_seq(seq_buf,100);   %Downsample to 5hz
seq_b.name = 'algo3';
seq_b.type = 'seq';

seq(nb,:) = seq_b;
clear seq_b

[seq(nb,:).var.Ag] = mov_mean_filter(2,seq(nb,:).var.A);  % For 10hz sampling
[seq(nb,:).var.Mg] = mov_mean_filter(2,seq(nb,:).var.M);  % For 10hz sampling
% [seq(nb,:).var.Ag] = mov_mean_filter(4,seq.var.A);  % For 1hz sampling
% [seq(nb,:).var.Mg] = mov_mean_filter(4,seq.var.M);  % For 1hz sampling

% Compute variables
[seq(nb,:).var.Ad] = var_accd(seq(nb,:).var);
[seq(nb,:).var.O]  = var_odba(seq(nb,:).var);
[seq(nb,:).var.J]  = var_jerk(seq(nb,:).var);

[seq(nb,:).var.Pi,seq(nb,:).var.Ro,seq(nb,:).var.He] = var_saam(seq(nb,:).var);

% Correction pitch offset
seq(nb,:).var.Pi.data = seq(nb,:).var.Pi.data -(9/180*pi);

%tic
% Vertical speed
[s_depth_2,v] = ocdr_2(seq(nb,:).var.P.data(:,1),seq(nb,:).var.Pi.data(:,1),1);

%Filtering frequency 
fd = 0.20;



% A = seq(nb,:).var.A.data(:,1:3);
% depth = seq(nb,:).var.P.data(:,1);
% fs = 1;
% fc = 0.25;
% nf = round(4*fs/fc) ;
% 
% A = fir_nodelay(A,nf,fc/(fs/2)) ;
% pitch = a2pr(A)+10/180*pi ;
% pitch(abs(pitch)<0.25/180*pi) = NaN ;
% 
% for i=2:length(depth(:,1))
%     v(i,1) = depth(i,1)-depth(i-1,1);
% 
%     s_depth = v./sin(pitch) ;
% end


% Horizontal speed : Need mean speed. We find by analysing reference speed
mean_s = 0.37;
s_h = sqrt(max(mean_s.^2-v.^2,0));

% ODBA 
odba_threshold = 0.004; % ODBA threshold for no moving phases / old 0.0056
od = odba(seq(nb,:).var.A.data(:,1:3),1,fd);


% for i=1:length(s(:,1)) 
% 
%     if ~isnan(s_depth(i,1))
%         s(i,1) = s_depth(i,1);
%     end
% 
%     if s(i,1) == 0
%         s(i,1) = mean_s;
%     end
%     if od(i,1) < odba_threshold % We Initialy 0.0026
%         s(i,1) = 0;
%     end    
% end
s = zeros(length(s_depth_2(:,1)),1);

s_mean = ones(length(s_depth_2(:,1)),1);

for i=1:length(s(:,1)) 

    %if s(i,1) == 0
        s(i,1) = mean_s;
    %end
%     
    if od(i,1) < odba_threshold % We Initialy 0.0026
        s(i,1) = 0;
    end  
%   
%     if s(i,1) > 1 % We Initialy 0.0026
%         s(i,1) = 1;
%     end 

    if ~isnan(s_depth_2(i,1))
        %s(i,1) = abs(s_depth(i,1));
        s(i,1) = abs(s_depth_2(i,1));
        s_mean(i,1) = 0;
        %s(i,1) = s_h(i,1);
    end


end

%toc



seq(nb,:).var.S.data = s;
seq(nb,:).var.S.sampling = seq_buf.var.A.sampling;
seq(nb,:).var.S.sampling_rate = seq(nb,:).var.A.sampling_rate;
seq(nb,:).var.S.sampling_rate_unit = seq_buf.var.A.sampling_rate_unit;
seq(nb,:).var.S.name = 'S';
seq(nb,:).var.S.full_name = 'Speed';
% 
% nb_behavior = length(seq(1,1).var.B.data(:,1)); % Number of behavior of the sequence
% 
seq(nb,1).var.S.data(:,2) = seq(nb,:).var.S.data(:,1);
seq(nb,1).var.S.data(:,2) = smoothdata(seq(nb,1).var.S.data(:,2),'movmean',10);
% 
% for j=1:nb_behavior
%     
%     c_behavior = seq(1,1).var.B.data(j,1);  % Current behavior
%     lim_inf = seq(1,1).var.B.data(j,2);     % Limit inferior of the behavior
%     lim_sup = seq(1,1).var.B.data(j,3);     % Limit superior of the behavior
%     
%     if c_behavior == 2  % rest behavion
%         
%          seq(nb,1).var.S.data(lim_inf:lim_sup,2) = s_fix(0);
%     end
% end

clear od odba mean_s v odba_threshold nb 