function [seq_etho,seq_etho_ref] = seq2seq_etho(seq,seq_ref,v)



if isstruct(seq) && nargin < 3
    
    v = seq_ref;
    %seq_etho_ref = struct([]);
    
    if v == 1
        
        len_etho = length(seq.var.B.data(:,1)); % Calculate the number of behavior to process
        
        first_up = 1;
        first_down = 1;
        first_swim = 1;
        first_rest = 1;
        first_surface = 1;
        
        
        for i=1:len_etho
            
            lim_inf = seq.var.B.data(i,2);
            lim_sup = seq.var.B.data(i,3);
            
            c_behavior = seq.var.B.data(i,1);
            
            if c_behavior == 3
                
                if first_up == 1
                    seq_etho.up = crop_seq(seq,lim_inf,lim_sup);
                    first_up = 0;
                else
                    seq_etho_buf.up = crop_seq(seq,lim_inf,lim_sup);
                    seq_etho.up = merge_seq(seq_etho.up, seq_etho_buf.up );
                end
                
                
            end
            
            if c_behavior == 4
                
                if first_down == 1
                    seq_etho.down = crop_seq(seq,lim_inf,lim_sup);
                    first_down = 0;
                else
                    seq_etho_buf.down = crop_seq(seq,lim_inf,lim_sup);
                    seq_etho.down = merge_seq(seq_etho.down, seq_etho_buf.down );
                end
                
                
            end
            
            if c_behavior == 1
                
                if first_swim == 1
                    seq_etho.swim = crop_seq(seq,lim_inf,lim_sup);
                    first_swim = 0;
                else
                    seq_etho_buf.swim = crop_seq(seq,lim_inf,lim_sup);
                    seq_etho.swim = merge_seq(seq_etho.swim, seq_etho_buf.swim );
                end
                
            end
            
        end
    end
end


if nargin > 2
    if isstruct(seq_ref)
        
        if v == 1
            
            len_etho = length(seq.var.B.data(:,1)); % Calculate the number of behavior to process
            
            first_up = 1;
            first_down = 1;
            first_swim = 1;
            first_rest = 1;
            first_surface = 1;
            
            
            for i=1:len_etho
                
                lim_inf = seq.var.B.data(i,2);
                lim_sup = seq.var.B.data(i,3);
                
                c_behavior = seq.var.B.data(i,1);
                
                if c_behavior == 3
                    
                    if first_up == 1
                        seq_etho.up = crop_seq(seq,lim_inf,lim_sup);
                        seq_etho_ref.up = crop_seq(seq_ref,lim_inf,lim_sup);
                        first_up = 0;
                    else
                        seq_etho_buf.up = crop_seq(seq,lim_inf,lim_sup);
                        seq_etho_buf_ref.up = crop_seq(seq_ref,lim_inf,lim_sup);
                        seq_etho.up = merge_seq(seq_etho.up, seq_etho_buf.up );
                        seq_etho_ref.up = merge_seq(seq_etho_ref.up, seq_etho_buf_ref.up );
                    end
                    
                    
                end
                
                if c_behavior == 4
                    
                    if first_down == 1
                        seq_etho.down = crop_seq(seq,lim_inf,lim_sup);
                        seq_etho_ref.down = crop_seq(seq_ref,lim_inf,lim_sup);
                        first_down = 0;
                    else
                        seq_etho_buf.down = crop_seq(seq,lim_inf,lim_sup);
                        seq_etho_buf_ref.down = crop_seq(seq_ref,lim_inf,lim_sup);
                        seq_etho.down = merge_seq(seq_etho.down, seq_etho_buf.down );
                        seq_etho_ref.down = merge_seq(seq_etho_ref.down, seq_etho_buf_ref.down );
                    end
                    
                    
                end
                
                if c_behavior == 1
                    
                    if first_swim == 1
                        seq_etho.swim = crop_seq(seq,lim_inf,lim_sup);
                        seq_etho_ref.swim = crop_seq(seq_ref,lim_inf,lim_sup);
                        first_swim = 0;
                    else
                        seq_etho_buf.swim = crop_seq(seq,lim_inf,lim_sup);
                        seq_etho_buf_ref.swim = crop_seq(seq_ref,lim_inf,lim_sup);
                        seq_etho.swim = merge_seq(seq_etho.swim, seq_etho_buf.swim );
                        seq_etho_ref.swim = merge_seq(seq_etho_ref.swim, seq_etho_buf_ref.swim );
                    end
                    
                    
                end
                
                
                
                
            end
            
            %     if first_swim == 1
            %         seq_etho.swim = struct([]);
            %         seq_etho_ref.swim = struct([]);
            %     end
            %     if first_up == 1
            %         seq_etho.up = struct([]);
            %         seq_etho_ref.up = struct([]);
            %     end
            %
            %     if first_down == 1
            %         seq_etho.down = struct([]);
            %         seq_etho_ref.down = struct([]);
            %     end
            
            
        end
        
        
    end
end





end


