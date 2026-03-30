function [public_vars] = plan_motion(read_only_vars, public_vars)

% public_vars.motion_vector = [0, 0];


%% TASK 5
k = read_only_vars.counter;

if k <= 50
    public_vars.motion_vector = [0.8, 0.8]; 
elseif k <=80
    public_vars.motion_vector = [0.8, 0.8];  
elseif k <= 90
    public_vars.motion_vector = [0.5, 0.8];     % zatacka doprava
elseif k <= 110
    public_vars.motion_vector = [0.8, 0.8];      % rovne    
elseif k <= 130
    public_vars.motion_vector = [0.8, 0.8];      % rovne
elseif k <= 140
    public_vars.motion_vector = [0.5, 0.8]; 
elseif k <= 180
     public_vars.motion_vector = [0.8, 0.8];   
elseif k <= 210
     public_vars.motion_vector = [0.8, 0.8];      % rovne
elseif k <= 220
     public_vars.motion_vector = [0.8, 0.5];   
elseif k <= 245
     public_vars.motion_vector = [0.8, 0.8]; 
elseif k <= 255
     public_vars.motion_vector = [0.8, 0.5]; 
elseif k <= 260
     public_vars.motion_vector = [0.7, 0.9]; 
elseif k <= 265
     public_vars.motion_vector = [0.8, 0.6];
elseif k <= 280
     public_vars.motion_vector = [0.7, 0.7]; 
elseif k <= 285
     public_vars.motion_vector = [0.6, 0.7]; 
elseif k <= 320
    public_vars.motion_vector = [0.7, 0.7];      % rovne
elseif k <= 345
     public_vars.motion_vector = [0.7, 0.7]; 
else
    public_vars.motion_vector = [0, 0];          % stop
end

end

