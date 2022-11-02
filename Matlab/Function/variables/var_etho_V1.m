function dives = var_etho_V1(dives,param)

%     Caculate ethogram V1 for each dive with tag data.
%
%     Input :
%     -dives is a sctructure array with information of each dive of a analysed sequence. The
%     minimal required fields of dives are:
%    		tstart 	    time in seconds of the start of each dive
%			tend 		time in seconds of the end of each dive
%			var         Structure array with at least data from Depth (P),
%			Accelerometer (A, Ag, Ad), magnetometer (M, Mg), ODBA (O)
%     -param is a sctructure with ethogram paramater:
%           slope_down  Higher to this value we consider that the DOWN phase end;
%           slope_up    Under this value we consider that the UP phase
%           start
%           odba_rest   Threshold value on ODBA for REST phase;
%           surf_depth  Threshold value on Depth for SURFACE phase;;
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

if ~isstruct(dives)
    fprintf('var_etho_V1 : Dive need to be a sctructure \n');
    help var_etho_V1
end

if nargin < 2  % If no imput param for the ethogram used preset ones
    
    
    if dives(1,1).fs_max == 10
        % Init algorithm variable
        slope_down = 0.01;
        slope_up = -0.003;
        odba_rest = 0.04;
        surf_depth = 0.2;
        surface_limit = 0.4;
    elseif dives(1,1).fs_max == 1
        % Init algorithm variable
        slope_down = 0.1;
        slope_up = -0.1;
        odba_rest = 0.02;
        odba_rest_end = 0.008;
        surf_depth = 0.2;
        surface_limit = 0.4;
    elseif dives(1,1).fs_max == 5
        %Init algorithm variable
        slope_down = 0.03;
        slope_up = -0.02;
        odba_rest = 0.0055;
        surf_depth = 0.2;
        surface_limit = 0.4;
    end
    
    
    
else
    if isstruct(param)
        
        if isfield(param,V1)
            if dives(1,1).fs_max == 10
                slope_down = param.V1.fs10.slope_down;
                slope_up = param.V1.fs10.slope_up;
                odba_rest = param.V1.fs10.odba_rest;
                surf_depth = param.V1.fs10.surf_depth;
                surface_limit = param.V1.fs10.surface_limit;
            elseif dives(1,1).fs_max == 1
                slope_down = param.V1.fs1.slope_down;
                slope_up = param.V1.fs1.slope_up;
                odba_rest = param.V1.fs1.odba_rest;
                surf_depth = param.V1.fs1.surf_depth;
                surface_limit = param.V1.fs1.surface_limit;
            end
        else
            fprintf('var_etho_V1 : Need V1 parameter to the sctructure to use this algorithm\n');
            help var_etho_V1
            return
        end
    else
        fprintf('var_etho_V1 : Parameter need to be a sctructure \n');
        help var_etho_V1
    end
end

nb_dives = length(dives);
nb_b = 5;


for i=1:nb_dives
    dives(i,1).var.B.name = 'B';
    dives(i,1).var.B.full_name = 'Behavior';
end


%Global ethogram variable
short_dive_limit = 70; % in second

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

t(:,:) = 10;
t(2,1) = 7; %Time start assocatied with flag 2 (Rest behavior)
t(3,1) = 15; %Time start assocatied with flag 3 (Up behavior)
t(3,2) = 10; %Time end assocatied with flag 3   (Up behavior)
t(4,1) = 15; %Time end assocatied with flag 4   (Down behavior)
t(4,2) = 15; %Time start assocatied with flag 4   (Down behavior)
t(1,1) = 15; %Time start assocatied with flag 1 (Swim behavior)


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
    
    %Special case of dive sequence with the turtle staying near surface
    depth_max = X.P.data(1:len_dive,1);
    if depth_max < surface_limit
        X.B.data(inc_etho,1) = 5;
    end
    
    %Special case of short dive
    if len_dive < short_dive_limit*X.P.sampling_rate
        X.B.data(inc_etho,1) = 5;
    end
    
        
        
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
        if X.O.data(j,1) > odba_rest_end && c_behavior == 2
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
                flag_e(:,:) = 0;
            elseif flag_s(2,1) >= t(2,1)*X.P.sampling_rate && abs(X.P.data(flag_s(2,2),1)-X.P.data(flag_s(2,2)-flag_s(2,1),1)) < 0.4 %REST : compare flag number and sampling (in nb of sample)
                nb_sample_end = flag_s(2,2)-flag_s(2,1);
                X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
                inc_etho = inc_etho+1;
                X.B.data(inc_etho,1) = 2;                         %Start REST behavior
                X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
                flag_s(:,:) = 0;
                flag_e(:,:) = 0;
            end
        end
        
        %Rest phase -> UP, DOWN,
        if c_behavior == 2
            
            if flag_s(3,1) >= t(3,1)*X.P.sampling_rate  % UP : Compare flag number and sampling (in nb of sample)
                nb_sample_end = flag_s(3,2)-flag_s(3,1);
                X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
                inc_etho = inc_etho+1;
                X.B.data(inc_etho,1) = 3;                         %Start UP behavior
                X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
                flag_s(:,:) = 0;
                flag_e(:,:) = 0;
            elseif flag_s(4,1) >= t(4,1)*X.P.sampling_rate %DOWN : compare flag number and sampling (in nb of sample)
                nb_sample_end = flag_s(4,2)-flag_s(4,1);
                X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
                inc_etho = inc_etho+1;
                X.B.data(inc_etho,1) = 4;                         %Start REST behavior
                X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
                flag_s(:,:) = 0;
                flag_e(:,:) = 0;
            end
        end
        
        %UP phase -> SURFACE
        if c_behavior == 3
            
            if flag_s(5,1) >= t(5,1)*X.P.sampling_rate  % Surface: Compare flag number and sampling (in nb of sample)
                nb_sample_end = flag_s(5,2)-flag_s(5,1); %% CHANGE FLAG 3 to 5
                X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
                inc_etho = inc_etho+1;                            %Increment new behavior
                X.B.data(inc_etho,1) = 5;                         %Start UP behavior
                X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
                flag_s(:,:) = 0;
                flag_e(:,:) = 0;
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
                flag_e(:,:) = 0;
            end
        end
        
        %SWIM phase -> DOWN, UP, REST
        if c_behavior == 1 %If the time of end flag for up phase is superior to the threshold
            
            if flag_s(2,1) >= t(2,1)*X.P.sampling_rate && abs(X.P.data(flag_s(2,2),1)-X.P.data(flag_s(2,2)-flag_s(2,1),1)) < 0.4 % REST: Compare flag number and sampling (in nb of sample)
                nb_sample_end = flag_s(2,2)-flag_s(2,1);
                X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
                inc_etho = inc_etho+1;
                X.B.data(inc_etho,1) = 2;                         %Start UP behavior
                X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
                flag_s(:,:) = 0;
                flag_e(:,:) = 0;
            elseif flag_s(3,1) >= t(3,1)*X.P.sampling_rate  %UP : compare flag number and sampling (in nb of sample)
                nb_sample_end = flag_s(3,2)-flag_s(3,1);
                X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
                inc_etho = inc_etho+1;
                X.B.data(inc_etho,1) = 3;                         %Start REST behavior
                X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
                flag_s(:,:) = 0;
                flag_e(:,:) = 0;
            elseif flag_s(4,1) >= t(4,1)*X.P.sampling_rate %DOWN: compare flag number and sampling (in nb of sample)
                nb_sample_end = flag_s(4,2)-flag_s(4,1);
                X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
                inc_etho = inc_etho+1;
                X.B.data(inc_etho,1) = 4;                         %Start DOWN behavior
                X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
                flag_s(:,:) = 0;
                flag_e(:,:) = 0;
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
            flag_s(:,:) = 0;
        end
        if flag_e(5,1) > t(5,2)*X.P.sampling_rate %If the time of end flag for up phase is superior to the threshold
            nb_sample_end = flag_e(5,2)-flag_e(5,1);
            X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
            inc_etho = inc_etho+1;                            %Increment new behavior
            X.B.data(inc_etho,1) = 1;                         %Start SWIM behavior
            X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
            flag_e(:,:) = 0;
            flag_s(:,:) = 0;
        end
        if flag_e(3,1) > t(3,2)*X.P.sampling_rate %If the time of end flag for up phase is superior to the threshold
            nb_sample_end = flag_e(3,2)-flag_e(3,1);
            X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
            inc_etho = inc_etho+1;                            %Increment new behavior
            X.B.data(inc_etho,1) = 1;                         %Start SWIM behavior
            X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
            flag_e(:,:) = 0;
            flag_s(:,:) = 0;
        end
        if flag_e(2,1) > t(2,2)*X.P.sampling_rate %If the time of end flag for up phase is superior to the threshold
            nb_sample_end = flag_e(2,2)-flag_e(2,1);
            X.B.data(inc_etho,3) = nb_sample_end-1; %End of the behavior
            inc_etho = inc_etho+1;                            %Increment new behavior
            X.B.data(inc_etho,1) = 1;                         %Start SWIM behavior
            X.B.data(inc_etho,2) = nb_sample_end;   %Start time of the new behavior
            flag_e(:,:) = 0;
            flag_s(:,:) = 0;
        end
        
        
        
    end
    
    X.B.data(end,3) = len_dive;
    dives(i,1).var = X;
    
end




end
