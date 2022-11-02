function dives = var_etho_V1(dives)

%     Caculate ethogram V1 for each dive with tag data.
%
%     Input :
%     -dives is a sctructure array with information of each dive of a analysed sequence. The
%     minimal required fields of dives are:
%    		tstart 	    time in seconds of the start of each dive
%			tend 		time in seconds of the end of each dive
%			var         Structure array with at least data from Depth (P),
%			Accelerometer (A, Ag, Ad), magnetometer (M, Mg), ODBA (O)
%
%
%     Output:
%     -dives is a sctructure array with information of each dive of a analysed sequence The
%		news fields of dives.var is B with:
%    		B.data: 	Behavior data
%           B.name:     Name of the sctucture
%           B.full_name Fulle name of the sctructure
%
%
%     Sctucture of the B.data:
%        Dives.var.B.data(:,1) = Number of the behavior
%        Dives.var.B.data(:,2) = Start time of the behavior
%        Dives.var.B.data(:,3) = End time of the behavior
%
%    Sctucture of the B.data:
%       Swim = 1
%       Rest = 2
%       Up = 3
%       down = 4
%       Surface = 5
%
%
%     pierregogendeau@gmail.com
%     Last modified: 10/03/22

nb_dives = length(dives);
nb_b = 5;


for i=1:nb_dives
    dives(i,1).var.B.name = 'B';
    dives(i,1).var.B.full_name = 'Behavior';
end


% Init algorithm variable
max_depth = 0;
min_depth = 0;
slope_down = 0.01;
slope_up = -0.01;
odba_rest = 0.0015;
surf_depth = 0.2;

%Flag
flag_s = zeros(nb_b, 2); %Start flag for behavior. Column 1 : Number of sample with flag to 1 / 2: Last ith sample of the dive
flag_e = zeros(nb_b, 2); %Ending flag for behavior. Column 1 : Number of sample with flag to 1 / 2: Last ith sample of the dive

% Time used to compare with the flag time
t(1,1) = 5; %Time start assocatied with flag 1 (Swim behavior)
t(1,2) = 5; %Time end assocatied with flag 1   (Swim behavior)
t(2,1) = 5; %Time start assocatied with flag 2 (Rest behavior)
t(2,2) = 5; %Time end assocatied with flag 2  (Rest behavior)
t(3,1) = 5; %Time start assocatied with flag 3 (Up behavior)
t(3,2) = 5; %Time end assocatied with flag 3   (Up behavior)
t(4,1) = 5; %Time start assocatied with flag 4 (Down behavior)
t(4,2) = 5; %Time end assocatied with flag 4   (Down behavior)
t(5,1) = 5; %Time start assocatied with flag 5 (Surface behavior)
t(5,2) = 5; %Time end assocatied with flag 5   (Surface behavior)

t(:,:) = 5;

for i=1:nb_dives %Loop to process each dive
%for i=26:26 %Loop to process each dive
    
    X = dives(i,1).var; %Store var sctructure in a variable for simplicity
    
    inc_etho = 1;
    X.B.data(inc_etho,1) = 4; % Dive start with down behavior
    X.B.data(inc_etho,2) = 1; %Start time of the time
    
    len_dive = length(X.P.data(:,1));
    
    %Slope of depth curve
    clear slope_d
    slope_d(:,1) = zeros(len_dive,1);
    for j=2:len_dive
        slope_d(j,1) = X.P.data(j,1)-X.P.data(j-1,1);
        %depth(j,i) = X.P.data(j,1);
    end
    
    flag_e(:,:) = 0;
    flag_s(:,:) = 0;
    
    
    for j=2:len_dive
        
        
        c_behavior = X.B.data(inc_etho,1); % Current behavior
        
        %Variable calculation
        slope_d(j,1) = X.P.data(j,1)-X.P.data(j-1,1);
        
        %Flag section
        
        %Starting behavior
        %UP : Test if the dive slope is inforior to a fixed value, we use flag_e4
        if slope_d(j,1) < slope_up 
            flag_s(3,1) = flag_s(3,1)+1;
            flag_s(3,2) = j;
        else  % If there
            flag_s(3,1) = 0;
        end
        
        %DOWN :Test if the dive slope is inforior to a fixed value, we use flag_e4
        if slope_d(j,1) > slope_down 
            flag_s(4,1) = flag_s(4,1)+1;
            flag_s(4,2) = j;
        else  % If there
            flag_s(4,1) = 0;
        end
        
        %REST : Test if ODBA inferior to a threshold. We can slope also
        if X.O.data(j,1) < odba_rest  
            flag_s(2,1) = flag_s(2,1)+1;
            flag_s(2,2) = j;
        else  % If there
            flag_s(2,1) = 0;
        end
        
        
        % SURFACE : Test if depth is inferior to a threshold
        if X.P.data(j,1) < surf_depth 
            flag_s(5,1) = flag_s(5,1)+1;
            flag_s(5,2) = j;
        else  % If there
            flag_s(5,1) = 0;
        end
        
        
        %Ending behavior
        %DOWN :Test if the dive slope is inforior to a fixed value, we use flag_e4
        if slope_d(j,1) < slope_down  && c_behavior == 4
            flag_e(4,1) = flag_e(4,1)+1;
            flag_e(4,2) = j;
        else  % If there
            flag_e(4,1) = 0;
        end
        
        %UP : Test if the dive slope is inforior to a fixed value, we use flag_e4
        if slope_d(j,1) > slope_up && c_behavior == 3
            flag_e(3,1) = flag_e(3,1)+1;
            flag_e(3,2) = j;
        else  % If there
            flag_e(3,1) = 0;
        end
        
        %REST : Test if ODBA inferior to a threshold. We can slope also
        if X.O.data(j,1) > odba_rest && c_behavior == 2
            flag_e(2,1) = flag_e(2,1)+1;
            flag_e(2,2) = j;
        else  % If there
            flag_e(2,1) = 0;
        end
        
        
        
        %Change behavior section
        
        
        %DOWN phase -> UP, REST
        if c_behavior == 4
            
            if flag_s(3,1) >= t(3,1)*X.P.sampling_rate  % UP : Compare flag number and sampling (in nb of sample)
                nb_sample_end = flag_s(3,2)-flag_s(3,1);
                X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
                inc_etho = inc_etho+1;
                X.B.data(inc_etho,1) = 3;                         %Start UP behavior
                X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
                flag_s(:,:) = 0;
            elseif flag_s(2,1) >= t(2,1)*X.P.sampling_rate %REST : compare flag number and sampling (in nb of sample)
                nb_sample_end = flag_s(2,2)-flag_s(2,1);
                X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
                inc_etho = inc_etho+1;
                X.B.data(inc_etho,1) = 2;                         %Start REST behavior
                X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
                flag_s(:,:) = 0;
            end
        end
        
        %Rest phase -> UP, DOWN, SWIM-
        if c_behavior == 2
            
            if flag_s(3,1) >= t(3,1)*X.P.sampling_rate  % UP : Compare flag number and sampling (in nb of sample)
                nb_sample_end = flag_s(3,2)-flag_s(3,1);
                X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
                inc_etho = inc_etho+1;
                X.B.data(inc_etho,1) = 3;                         %Start UP behavior
                X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
                flag_s(:,:) = 0;
            elseif flag_s(4,1) >= t(4,1)*X.P.sampling_rate %DOWN : compare flag number and sampling (in nb of sample)
                nb_sample_end = flag_s(4,2)-flag_s(4,1);
                X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
                inc_etho = inc_etho+1;
                X.B.data(inc_etho,1) = 4;                         %Start REST behavior
                X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
                flag_s(:,:) = 0;
            end
        end
        
        %UP phase -> SURFACE, SWIM
        if c_behavior == 3
               
            if flag_s(5,1) >= t(5,1)*X.P.sampling_rate  % Surface: Compare flag number and sampling (in nb of sample)
                nb_sample_end = flag_s(3,2)-flag_s(3,1);
                X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
                inc_etho = inc_etho+1;                            %Increment new behavior
                X.B.data(inc_etho,1) = 5;                         %Start UP behavior
                X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
                flag_s(:,:) = 0;
            end
        end
        
        %Surface phase -> DOWN (If the dive crop is well done nothing
        %after surface
        if c_behavior == 5
            
            if flag_s(4,1) >= t(4,1)*X.P.sampling_rate  % Surface: Compare flag number and sampling (in nb of sample)
                nb_sample_end = flag_s(5,2)-flag_s(5,1);
                X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
                inc_etho = inc_etho+1;                            %Increment new behavior
                X.B.data(inc_etho,1) = 4;                         %Start UP behavior
                X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
                flag_s(:,:) = 0;
            end
        end
        
        %SWIM phase -> DOWN, UP, REST
        if c_behavior == 1 %If the time of end flag for up phase is superior to the threshold
            
            if flag_s(2,1) >= t(2,1)*X.P.sampling_rate  % REST: Compare flag number and sampling (in nb of sample)
                nb_sample_end = flag_s(2,2)-flag_s(2,1);
                X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
                inc_etho = inc_etho+1;
                X.B.data(inc_etho,1) = 2;                         %Start UP behavior
                X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
                flag_s(:,:) = 0;
            elseif flag_s(3,1) >= t(3,1)*X.P.sampling_rate  %UP : compare flag number and sampling (in nb of sample)
                nb_sample_end = flag_s(3,2)-flag_s(3,1);
                X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
                inc_etho = inc_etho+1;
                X.B.data(inc_etho,1) = 3;                         %Start REST behavior
                X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
                flag_s(:,:) = 0;
            elseif flag_s(4,1) >= t(4,1)*X.P.sampling_rate %DOWN: compare flag number and sampling (in nb of sample)
                nb_sample_end = flag_s(4,2)-flag_s(4,1);
                X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
                inc_etho = inc_etho+1;
                X.B.data(inc_etho,1) = 4;                         %Start DOWN behavior
                X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
                flag_s(:,:) = 0;
            end
        end
        
        %SWIM : Swimming phase is hard to describe so we use the end time of the other phase
        %We use end flag of other behavior to go in SWIM phase
        if flag_e(4,1) > t(4,2)*X.P.sampling_rate %If the time of end flag for up phase is superior to the threshold
            nb_sample_end = flag_e(4,2)-flag_e(4,1);
            X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
            inc_etho = inc_etho+1;                            %Increment new behavior
            X.B.data(inc_etho,1) = 1;                         %Start SWIM behavior
            X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
            flag_e(:,:) = 0;
        end
        if flag_e(5,1) > t(5,2)*X.P.sampling_rate %If the time of end flag for up phase is superior to the threshold
            nb_sample_end = flag_e(5,2)-flag_e(5,1);
            X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
            inc_etho = inc_etho+1;                            %Increment new behavior
            X.B.data(inc_etho,1) = 1;                         %Start SWIM behavior
            X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
            flag_e(:,:) = 0;
        end
        if flag_e(3,1) > t(3,2)*X.P.sampling_rate %If the time of end flag for up phase is superior to the threshold
            nb_sample_end = flag_e(3,2)-flag_e(3,1);
            X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
            inc_etho = inc_etho+1;                            %Increment new behavior
            X.B.data(inc_etho,1) = 1;                         %Start SWIM behavior
            X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
            flag_e(:,:) = 0;
        end
        if flag_e(2,1) > t(2,2)*X.P.sampling_rate %If the time of end flag for up phase is superior to the threshold
            nb_sample_end = flag_e(2,2)-flag_e(2,1);
            X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
            inc_etho = inc_etho+1;                            %Increment new behavior
            X.B.data(inc_etho,1) = 1;                         %Start SWIM behavior
            X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
            flag_e(:,:) = 0;
        end
        
    end
    
    X.B.data(end,3) = len_dive;
    dives(i,1).var = X;
    
end




end
