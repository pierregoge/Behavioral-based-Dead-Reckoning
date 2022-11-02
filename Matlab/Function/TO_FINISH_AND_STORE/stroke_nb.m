function [nb_str, avg_str] = stroke_nb(x)

t_stroke = zero_cross(x(:,1));



if ~isempty(t_stroke)
[p2p_stroke] = stroke_p2p(x(:,1),t_stroke);

nb_str= length(t_stroke(:,1));
avg_str = mean(p2p_stroke);
else
    nb_str= 0;
    avg_str = 0;
end


end

