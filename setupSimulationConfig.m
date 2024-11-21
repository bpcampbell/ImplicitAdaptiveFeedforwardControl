function simConfig = setupSimulationConfig(loadToBase)

    % Initialize simulation parameters
    simConfig.t_run = 1;
    simConfig.sim_time_step = 1/1000;
    simConfig.controller_skips = 1;
    simConfig.ref_sample_time1 = simConfig.sim_time_step;

    % Initialize reference parameters
    simConfig.frequency_vector_1 = 300;
    simConfig.gain_vector_1 = 1;
    simConfig.frequency_vector_2 = 200;
    simConfig.gain_vector_2 = 1.2;

    % Initialize plant dynamics
    s = tf('s');
    P = [3/(60*s + 1) 6*s/(25*s + 1); 
         10*s/(50*s + 1) 2/(40*s + 1)];
    simConfig.P = P;

    % Load feedback controller
    C = load("FeedbackController.mat");
    simConfig.C = C.C; 

    % Build FSDoF controller
    omega = 2000;
    pow = 3;
    F_sub = 1/(s/omega + 1)^pow;
    F = [F_sub, 0; 0, F_sub];
    F_d = c2d(F, simConfig.sim_time_step, 'zoh');
    simConfig.F_d = F_d;

    % Set adaptation parameters
    simConfig.alpha_reference = 0.01; 
    simConfig.alpha_gradient = 0.01; 
    simConfig.buffer_size = 50;
    simConfig.initialWeightsScale = 1;
    simConfig.initialWeightsSeed = 0;

    % Optionally load variables into the base workspace
    if nargin > 0 && loadToBase
        fields = fieldnames(simConfig);
        for i = 1:numel(fields)
            assignin('base', fields{i}, simConfig.(fields{i}));
        end
    end
end

