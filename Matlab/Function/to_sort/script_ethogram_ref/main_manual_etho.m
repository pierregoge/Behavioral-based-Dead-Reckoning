%% Manual Ethogram
%for STD filter = 3
%1 sequence
lim(1,1) = 580;
lim(1,2) = 1080;
ethogram{1}(:,1) = [0 17 22 30 40 47 74 113 135 147 186 190 225  249 257 279 314 324 343 374 379 390 417 423 435 447 459 463]'+580;
ethogram{1}(:,2) = [17 22 30 40 47 74 113 135 147 186 190 225  249 257 279 314 324 343 374 379 390 417 423 435 447 459 463 500]'+580;
ethogram_name{1}(:,1) = ["s_down"; "s_glide";"s_down"; "s_up"; "s_glide";"s_up"; "swim";"up";"down";"swim";"s_ukn";"swim";"s_up";"s_glide";"swim" ;"up"; "down";"up";"down";"d_glide";"down";"d_swim";"down";"d_glide";"down";"swim";"s_ukn";"swim"];

%2
lim(2,1) = 1360;
lim(2,2) = 2760;

ethogram{2}(:,1) = [0, 16, 22, 33, 43, 60,78 ,250,266,761,797,802,824,829,882,886,904,930,941,950,1020,1042,1082,1093,1184, 1197,1237,1241,1267,1277,1351,1359]'+1360;
ethogram{2}(:,2) = [16, 22, 33, 43, 60,78 ,250,266,761,797,802,824,829,882,886,904,930,941,950,1020,1042,1082,1093,1184,1197,1237,1241,1267,1277,1351,1359,1400]'+1360;
ethogram_name{2}(:,1) = ["swim";"s_glide";"swim";"s_glide";"swim";"p_rest";"rest";"m_rest";"rest";"p_rest";"s_glide"; "swim";"s_glide"; "swim";"s_ukn"; "swim";"s_ukn";"swim";"p_rest";"rest";"m_rest";"rest";"m_rest";"rest";"p_rest";"swim";"s_ukn"; "swim";"s_ukn"; "swim";"s_glide";"up"];

%3
lim(3,1) = 2800;
lim(3,2) = 3210;
ethogram{3}(:,1) = [0 38 81 103 110 126 212 226 313 326 369 388]'+2800;
ethogram{3}(:,2) = [38 81 103 110 126 212 226 313 326 369 388 410]'+2800;
ethogram_name{3}(:,1) = ["down";"swim";"s_ukn";"swim";"s_glide";"swim";"s_glide";"swim";"s_glide";"swim";"s_glide";"swim"];

%4
lim(4,1) = 4550;
lim(4,2) = 5000;
ethogram{4}(:,1) = [0 19 33 53 64 69 100 104 179 185 239 242 369 376 400 407 430 435]'+4550;
ethogram{4}(:,2) = [19 33 53 64 69 100 104 179 186 239 242 369 376 400 407 430 435 450]'+4550;
ethogram_name{4}(:,1) = ["down";"d_glide";"down";"swim";"s_glide";"swim";"s_glide";"swim";"s_glide";"swim";"s_glide";"swim";"s_glide";"swim";"s_glide";"swim";"s_glide";"swim"];

%5
lim(5,1) = 5100;
lim(5,2) = 5630;
ethogram{5}(:,1) = [0 20 40 167 179 223 246 291 308   336 350 396 406 447 452 500]'+5100;
ethogram{5}(:,2) = [20 40 167 179 223 246 291 308   336   350 396 406 447 452 500 530]'+5100;
ethogram_name{5}(:,1) = ["down";"d_glide";"swim";"s_glide";"swim";"s_glide";"s_up";"s_glide";"swim";"s_glide";"swim";"s_glide";"swim";"s_ukn";"swim";"up"];
% Revoir partie entre 5430 et 5600

%6
lim(6,1) = 6101;
lim(6,2) = 6390;
ethogram{6}(:,1) =  [0  36 109 135 150 198 215]'+6101;
ethogram{6}(:,2) =  [36 109 135 150 198 215 290]'+6101;
ethogram_name{6}(:,1) = ["down";"swim";"s_ukn";"p_rest";"rest";"m_rest";"rest"];

%7
lim(7,1) = 7243;
lim(7,2) = 10740; % Old : 10740
ethogram{7}(:,1) = [0,963,978,994,1024,1050,1087,1094,1142,1194,1211,1301,1307,1326,1385,1414,1455,1469,1513,1521,1561,1578,1585,1591,1677,1708,1730,1769,1784,1881,1887,...
    1938,1944,1952,1985,1992,2007,2046,2060,2077,2094,2103,2123,2132,2133,2143,2153,2163,2168,2172,2204,2209,2286,2317,2324,2385,2399,2416,2472,2555,2573,2587,2594,2603,2605,...
    2639,2677,2687,2688,2730,2813,2840,2858,2869,2930,2965,3019,3037,3054,3064,3070,3078,3084,3089,3115,3141,3223,3230,3274,3468]'+7243;
ethogram{7}(:,2) = [963,978,994,1024,1050,1087,1094,1142,1194,1211,1301,1307,1326,1385,1414,1455,1469,1513,1521,1561,1578,1585,1591,1677,1708,1730,1769,1784,1881,1887,...
    1938,1944,1952,1985,1992,2007,2046,2060,2077,2094,2103,2123,2132,2133,2143,2153,2163,2168,2172,2204,2209,2286,2317,2324,2385,2399,2416,2472,2555,2573,2587,2594,2603,2605,...
    2639,2677,2687,2688,2730,2813,2840,2858,2869,2930,2965,3019,3037,3054,3064,3070,3078,3084,3089,3115,3141,3223,3230,3274,3468,3497]'+7243;
ethogram_name{7}(:,1) = ["rest";"m_rest";"rest";"m_rest";"up";"s_ukn";"swim";"up";"down";"d_glide";"swim";"s_ukn";"swim";"up";"up_swim";"up";"up_ukn";"down";"d_glide";"swim";"s_ukn";"swim";"s_glide";...
    "swim";"up";"up_glide";     "down"; 
    "d_glide"; "swim"; "s_glide";"swim";"s_glide";"swim"; "up";"up_glide";"up";"down";  "swim";"up";"up_glide";"down";"swim";"up";"surface";"down";"swim";"up";"down";"up";"down";"d_glide";"swim"; ...
      "d_glide";"s_ukn";"swim";"s_ukn";"swim";"s_ukn";"up";"down";"up";"up_glide";"up";"surface";"down";"up";"up_glide";"surface";"down";"d_swim";"down";"d_glide";"down";"up";"down";"swim";"up";...
      "up_glide";"down";"up";"up_glide";"down";"up";"down";"d_glide";"swim";"s_ukn";"swim";"s_eat";"swim"];      %;"down"     ,"up","down","swim","up","down","swim","up","down","up","down"]';
%

lim(8,1) = 10741;
lim(8,2) = 12752;
ethogram{8}(:,1) =  [0, 1000, 2000 2010 ]'+10741;
ethogram{8}(:,2) =  [1000,2000, 2010 2011]'+10741;
ethogram_name{8}(:,1) = ["swim";"swim";"rest";"swim"];

% Old : 10740    
   % "up";"down";"up";"g_up";"up";"surface";...
   % "down";"up";"surface";"down";"swim";"down";"g_down";"swim";"up";"down";"swim";"up";"g_up";"down";"swim";"g_up";"down";"up";"down";"g_down";"swim";"test"];      %;"down"     ,"up","down","swim","up","down","swim","up","down","up","down"]';

ethogram_b_roi = [];
behavior = [];
for i=1:length(ethogram)
behavior_b1{i}(:,:) = ethogram_name{i}(:,1);
behavior_b{i} = categorical(behavior_b1{i});
roi_table{i}(:,1:2) = [ethogram{1,i}(:,1) ethogram{1,i}(:,2)];
roi_tab = roi_table{i};
behavior_s{i} = behavior_b{i};
behavior = [behavior;behavior_b{i}];
ethogram_s_roi{i}(:,:) = table(roi_tab,behavior_s{i});
ethogram_b_roi = ([ethogram_b_roi;roi_table{i}(:,1:2)]);
end

ethogram_roi = table(ethogram_b_roi,behavior);

% Create of roi strarting from 1 to lenght of sequence for anaylisis
% For 1 and 10Hz
for j=1:length(ethogram)
roi_etho = ethogram_s_roi{1,j}.roi_tab(:,1:2);
bev{j} = string(ethogram_s_roi{1,j}.Var2(:,1));
roi{j} = roi_etho(:,1:2) - roi_etho(1,1);
roi_10{j} = roi_etho(:,1:2)*10 - roi_etho(1,1)*10;
roi{j}(1,1) = 1;
roi_10{j}(1,1) = 1;
end

ethogram_s = ethogram_s_roi;
for i=1:length(ethogram)
ethogram_s{i}.roi_tab_sec(:,:) = ((ethogram_s{i}.roi_tab(:,:)-ethogram_s{i}.roi_tab(1,1)))+1;
ethogram_s{i}.roi_tab(:,:) = ((ethogram_s{i}.roi_tab(:,:)-ethogram_s{i}.roi_tab(1,1))*10)+1;

ethogram_s{i}.roi_tab_sec(end,2) = ethogram_s{i}.roi_tab_sec(end,2)+1;
ethogram_s{i}.roi_tab(end,2) =  ethogram_s{i}.roi_tab(end,2)+1;

end
ethogram_s{6}.roi_tab(7,2) = 2891;
ethogram_s{6}.roi_tab_sec(7,2) = 290;