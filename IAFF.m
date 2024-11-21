%% Define class
classdef IAFF < matlab.System
    % IAFF - Implicit Adaptive Feedforward controller class
    % This class implements an adaptive feedforward control system using
    % two adaptive filters:
    % 1) Reference filter: computes feedforward actuation command based
    %                      on input.
    % 2) Gradient filter: estimates sensitivity (de/du) to adjust the
    %                     reference filter.
    %
    % Inputs:
    %   - in: The current input signal to the reference filter
    %         (e.g., desired state).
    %   - e: The current error signal (e.g., feedback error).
    %
    % Optional Input (enabled based on properties):
    %   - adapt?: (Logical) An optional boolean input that enables or
    %             disables adaptation. If enableAdaptInput is false,
    %             adaptation is always enabled.
    %
    % Outputs:
    %   - out: The computed feedforward actuation command from the
    %          reference filter.
    %
    % Optional Outputs (enabled based on properties):
    %   - dedt_actual: The rate of change of the error signal, de/dt.
    %   - dedt_predicted: Gradient filter's predicted rate of change of
    %                     error.
    %   - dedu: The estimated sensitivity derivatives (de/du) from the
    %           gradient filter weights.
    %   - weights: The weights of the reference filter.
    %
    % Properties:
    %   - alpha_gradient: Learning rate for the gradient filter.
    %   - alpha_reference: Learning rate for the reference filter.
    %   - L: Length of the tapped-delay line filter.
    %   - dt: Sample time for the system.
    %   - N: Number of inputs (e.g., state variables).
    %   - M: Number of outputs (e.g., actuation commands).
    %   - K: Number of error terms used for gradient adaptation.
    %   - variable_type: Specifies variable type ('single' or 'double').
    %   - enableOutputDeDt: If true, outputs de/dt actual and predicted.
    %   - enableOutputDeDu: If true, outputs dedu (subset of gradient
    %                       filter weights).
    %   - enableOutputWeights: If true, outputs reference filter weights.
    %   - enableAdaptInput: Enables an "adapt?" input port if true.
    %   - initialWeightsScale: Value by which to scale the initial weights
    %   - initialWeightsSeed: Seed for random weights initalisation

    %% Initialise non-tuneable, discrete state, and dependent properties
    properties (Nontunable)
        % alpha_gradient Gradient filter learning rate
        alpha_gradient = 0.001;

        % alpha_reference Reference filter learning rate
        alpha_reference = 0.0001;

        % L Tapped-delay line filter length
        L = 50;

        % dt Sample time
        dt = 0.001;

        % N Number of inputs (states)
        N = 2;

        % M Number of outputs (actuations)
        M = 2;

        % K Number of errors
        K = 4;

        % variable_type Type of variable ('single' or 'double')
        variable_type = 'single';

        % enableOutputDeDt Output predicted and true de/dt?
        enableOutputDeDt (1,1) logical = false;

        % enableOutputDeDu Output de/du (gradient-filter weights)?
        enableOutputDeDu (1,1) logical = false;

        % enableOutputWeights Output reference filter weights?
        enableOutputWeights (1,1) logical = false;

        % enableAdaptInput Enable "adapt?" input port
        enableAdaptInput (1,1) logical = false;

        % initialWeightsScale Value to scale the initial weights
        initialWeightsScale = 1;

        % initialWeightsSeed Seed for initial weights (int or 'shuffle')
        initialWeightsSeed = 0;
    end

    properties (Dependent)
        % Dependent properties computed from internal state data
        du_dt_buffer;           % Buffer for feedforward command rate
        reference_buffer;       % Reference input history buffer
        gradient_filter_input;  % Input for gradient filter
    end

    properties (DiscreteState)
        % States that are updated at each time step
        reference_filter_weights;  % Weights for reference filter
        reference_history;         % History buffer of reference inputs
        gradient_filter_weights;   % Weights for gradient filter
        gradient_filter_output;    % Output from gradient filter
        old_output;                % Previous output command
        old_error;                 % Previous error value
        du_dt_history;             % History of change in output commands
    end

    %% Define access methods to get dependent properties
    methods

        function b = get.reference_buffer(obj)
            % Access latest entries in reference history for computation
            b = obj.reference_history(1:obj.L,:);
        end

        function b = get.du_dt_buffer(obj)
            % Access du/dt buffer history for gradient filter updates
            b = obj.du_dt_history(1:obj.L,:);
        end

        function b = get.gradient_filter_input(obj)
            % Constructs combined input for gradient filter
            b = [obj.du_dt_buffer(:); obj.reference_buffer(:)];
        end

        function b = de_du_buffer(obj, from_ff_m, to_e_k)
            % Access du/dt buffer history for gradient filter input
            b = obj.gradient_filter_weights(1+(from_ff_m - 1)...
                *obj.L:from_ff_m*obj.L, to_e_k);
        end
    end

    %% Determine input and output sizes and types
    methods (Access = protected)

        % Determine the number of outputs based on enabled properties
        function num_outputs = getNumOutputsImpl(obj)
            % Calculates total number of outputs based on properties.
            % Add 1 for the main output, 2 for derivative outputs if DeDt
            % is enabled, 1 if DeDu is enabled, and 1 if outputting weights
            num_outputs = 1 + obj.enableOutputDeDt*2 ...
                + obj.enableOutputDeDu + obj.enableOutputWeights;
        end

        % Define the number of inputs based on adaptation input settings
        function num_inputs = getNumInputsImpl(obj)
            % Checks if adaptation input is enabled and adjusts input count
            if obj.enableAdaptInput
                num_inputs = 3; % Inputs: "in", "error", and "adapt?"
            else
                num_inputs = 2; % Inputs: "in" and "error" only
            end
        end

        % Define data types for each output port
        function varargout = getOutputDataTypeImpl(obj)
            % Assigns each output port the specified variable type
            varargout = cell(1, getNumOutputsImpl(obj));
            for i = 1:getNumOutputsImpl(obj)
                varargout{i} = obj.variable_type;
            end
        end

        % Define whether each output is complex or real
        function varargout = isOutputComplexImpl(obj)
            % All outputs are defined as non-complex in this implementation
            varargout = cell(1, getNumOutputsImpl(obj));
            for i = 1:getNumOutputsImpl(obj)
                varargout{i} = false;
            end
        end

        % Define output size for each port based on enabled properties
        function varargout = getOutputSizeImpl(obj)
            % Sets output sizes based on enabled properties
            varargout{1} = [1 obj.M]; % Main output size
            out_ind = 2; % Start index for optional outputs

            % If de/dt output is enabled, add output sizes for gradient
            % filter and de/dt outputs
            if obj.enableOutputDeDt
                varargout{out_ind} = [obj.K 1];
                varargout{out_ind + 1} = [obj.K 1];
                out_ind = out_ind + 2;
            end

            % If DeDu output is enabled, add output size for du/dt output
            if obj.enableOutputDeDu
                varargout{out_ind} = [obj.L 1];
                out_ind = out_ind + 1;
            end

            % If weights output is enabled, add output size for filter
            % weights
            if obj.enableOutputWeights
                varargout{out_ind} = [obj.L 1];
            end
        end

        % Define whether each output port has fixed size
        function varargout = isOutputFixedSizeImpl(obj)
            % Specifies all outputs as fixed-size arrays
            varargout = cell(1, getNumOutputsImpl(obj));
            for i = 1:getNumOutputsImpl(obj)
                varargout{i} = true;
            end
        end

        %% Define discrete state specifications and function to reset and initialise the discrete states
        function [sz, dt, cp] = ...
                getDiscreteStateSpecificationImpl(obj, name)

            % Sets size, data type, and complexity for each discrete state
            % based on name

            if strcmp(name, 'reference_filter_weights')
                % Size and type of reference filter weights matrix
                sz = [obj.L, obj.M*obj.N];
                dt = obj.variable_type;
                cp = false;
            elseif strcmp(name, 'gradient_filter_weights')
                % Size and type of gradient filter weights matrix
                sz = [(obj.M+obj.N)*obj.L, obj.K];
                dt = obj.variable_type;
                cp = false;
            elseif strcmp(name, 'old_output')
                % Size and type for storing the last output state
                sz = [1, obj.M];
                dt = obj.variable_type;
                cp = false;
            elseif strcmp(name, 'old_error')
                % Size and type for storing the last error value
                sz = [obj.K, 1];
                dt = obj.variable_type;
                cp = false;
            elseif strcmp(name, 'du_dt_history')
                % Size and type of du/dt history buffer
                sz = [2*obj.L+1, obj.M];
                dt = obj.variable_type;
                cp = false;
            elseif strcmp(name, 'gradient_filter_output')
                % Size and type of gradient filter output
                sz = [obj.K, 1];
                dt = obj.variable_type;
                cp = false;
            elseif strcmp(name, 'reference_history')
                % Size and type for reference history buffer
                sz = [2*obj.L+1, obj.N];
                dt = obj.variable_type;
                cp = false;
            else
                % Throw an error if the name does not match any state
                error("Error: Incorrect State Name: " + name)
            end
        end

        % Initialize state variables and weights upon system setup
        function setupImpl(obj)

            % Set random number generator seed to reproduce exact results
            rng(obj.initialWeightsSeed);

            % Initializes weight matrices and history buffers for the
            % reference and gradient filters.
            % Randomize reference filter weights across input dimensions
            obj.reference_filter_weights = ...
                obj.initialWeightsScale*(rand(obj.L, obj.M*obj.N, ...
                obj.variable_type))/(obj.L);

            % Initialize reference history to zero with double the buffer
            % length and 1 additional row
            obj.reference_history = ...
                zeros(2*obj.L+1, obj.N, obj.variable_type);

            % Initialize gradient filter weights to zero, sized for input
            % and output counts
            obj.gradient_filter_weights = ...
                zeros((obj.N + obj.M)*obj.L, obj.K, obj.variable_type);

            % Initialize previous outputs and errors, and the output of the
            % gradient filter
            obj.old_output = zeros(1, obj.M, obj.variable_type);
            obj.old_error = zeros(obj.K, 1, obj.variable_type);
            obj.gradient_filter_output = ...
                zeros(obj.K, 1, obj.variable_type);

            % Initialize du/dt history for tracking changes in control
            % output over time
            obj.du_dt_history = zeros(2*obj.L+1, obj.M, obj.variable_type);
        end

        % Reset the internal states of the system object.
        function resetImpl(obj)
            
            % Set random number generator seed to reproduce exact results
            rng(obj.initialWeightsSeed);

            % Initialize or reset adaptive filter weights and history
            % variables
            obj.reference_filter_weights = ...
                obj.initialWeightsScale*(rand(obj.L, obj.M*obj.N, ...
                obj.variable_type))/(obj.L);
            obj.reference_history = ...
                zeros(2*obj.L+1, obj.N, obj.variable_type);
            obj.gradient_filter_weights = ...
                zeros((obj.N + obj.M)*obj.L, obj.K, obj.variable_type);
            obj.old_output = zeros(1, obj.M, obj.variable_type);
            obj.old_error = zeros(obj.K, 1, obj.variable_type);
            obj.gradient_filter_output = ...
                zeros(obj.K, 1, obj.variable_type);
            obj.du_dt_history = zeros(2*obj.L+1, obj.M, obj.variable_type);
        end

        %% Implicit adaptation code that executes each step
        function varargout = stepImpl(obj, in, e, varargin)
            % Determine whether adaptation is enabled based on input or
            % property setting. If `enableAdaptInput` is true, use the
            % input port "adapt?" (varargin{1}), otherwise, set `adapt` to
            % true to allow adaptation at each step.
            if obj.enableAdaptInput
                adapt = varargin{1}; % Use the optional input port "adapt?"
            else
                adapt = true; % Always adapt if no "adapt?" input port
            end

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Adapt gradient filter based on previous feedforward %
            % commands and current error rate                     %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Calculate the rate of change of the error signal, de/dt,
            % difference between the current and previous errors divided by
            % the sample time.
            de_dt = (e - obj.old_error) / obj.dt;

            % Calculate the error between the gradient filter's last
            % prediction and the actual de/dt value. This error will be
            % used to update the filter's weights.
            gradient_filter_error = obj.gradient_filter_output - de_dt;

            % Update the gradient filter weights by adjusting them in the
            % opposite direction of the gradient_filter_error, scaled by
            % `alpha_gradient` (learning rate). Regularization is applied
            % by adding a small value to the norm of the filter input.
            obj.gradient_filter_weights = obj.gradient_filter_weights ...
                - obj.alpha_gradient *obj.gradient_filter_input...
                * gradient_filter_error' ...
                / (norm(obj.gradient_filter_input) + 1);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Calculate output of reference filter %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Shift each element in `reference_history` down by one row for
            % history buffer, and add the newest reference input at the top
            obj.reference_history(2:end, :) = ...
                obj.reference_history(1:end-1, :);
            obj.reference_history(1, :) = in;

            % Initialize the output vector with zeros based on the number
            % of outputs (M).
            out = zeros(1, obj.M, obj.variable_type);

            % For each output (actuation) channel, calculate the filtered
            % reference output by summing the dot products of the reference
            % filter weights with the reference buffer.
            for i = 1:obj.M
                out(i) = sum(dot(obj.reference_filter_weights(:, 1 ...
                    + (i-1)*obj.N : obj.N*i), obj.reference_buffer));
            end

            % Store the main output result in `varargout`, which allows for
            % variable-length outputs.
            varargout{1} = out;
            out_ind = 2; % Initialize index for additional optional outputs

            % Conditionally add optional outputs based on property flags
            if obj.enableOutputDeDt

                % If enabled, output both the gradient filter's predicted
                % de/dt and the actual de/dt calculated earlier
                varargout{out_ind} = obj.gradient_filter_output;
                varargout{out_ind + 1} = de_dt;
                out_ind = out_ind + 2;
            end

            if obj.enableOutputDeDu

                % If enabled, output the estimated gradient (de/du)
                varargout{out_ind} = obj.de_du_buffer(1, 1);
                out_ind = out_ind + 1;
            end

            if obj.enableOutputWeights

                % If enabled, output the weights of the reference filter
                % for debugging or analysis
                varargout{out_ind} = obj.reference_filter_weights(:, 1);
            end

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Predict the next de/dt using the updated gradient filter %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Update the du/dt history by shifting down and adding the
            % current change in output (difference between current and
            % previous outputs) at the top.
            obj.du_dt_history(2:end, :) = obj.du_dt_history(1:end-1, :);
            obj.du_dt_history(1, :) = out - obj.old_output;
            obj.old_output = out; % Update the stored output

            % Predict the next de/dt by calculating the dot product of the
            % gradient filter weights with the input history
            obj.gradient_filter_output = obj.gradient_filter_weights'...
                * obj.gradient_filter_input;

            % If adaptation is enabled, adjust the reference filter weights
            if adapt

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Adapt reference filter weights based on error %
                % and gradient estimates                        %
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                % Iterate over all inputs (N) and outputs (M) to update 
                % each filters weights
                for l = 1:obj.N
                    for j = 1:obj.M

                        % Compute the filter index for the current 
                        % input-output pair
                        f_ind = (j-1) * obj.N + l;

                        % Loop over the filter taps (L) and number of error
                        % terms (K) used in adaptation
                        for i = 1:obj.L
                            for k = 1:obj.K

                                % Update each reference filter weight based
                                % on the error (e) gradient estimate 
                                % (de/du buffer), and reference history,
                                % scaled by the learning rate.
                                % The update is regularized by the norms of
                                % the gradient and reference history.
                                obj.reference_filter_weights(i, f_ind)...
                                    =obj.reference_filter_weights(i,...
                                    f_ind)-obj.alpha_reference * e(k) ...
                                    * dot(obj.de_du_buffer(j, k), ...
                                    obj.reference_history(i+1:i+obj.L,l)...
                                    )/(norm(obj.de_du_buffer(j, k)) ...
                                    * norm(obj.reference_history(i+1:...
                                    i+obj.L, l)) + 1);
                            end
                        end
                    end
                end
            end

            % Store the current error for use in the next time step
            obj.old_error = e;
        end
    end

    %% Customize block icon and dialog display
    methods (Static, Access = protected)
        % Set the title of the block
        function header = getHeaderImpl
            header = matlab.system.display.Header(...
                'lmsSysObj', ...
                'Title', 'C2 Adaptive Filter');
        end

        % Define properties grouped into tabs for better user experience
        function tabs = getPropertyGroupsImpl
            % General settings group
            group1 = matlab.system.display.SectionGroup(...
                'Title','General',...
                'PropertyList',{'alpha_gradient','alpha_reference',...
                'L','N','M','K'});

            % Output settings group, includes toggling for optional outputs
            group2 = matlab.system.display.SectionGroup(...
                'Title', 'Outputs', ...
                'PropertyList',{'enableOutputDeDt','enableOutputDeDu',...
                'enableOutputWeights'});

            % Data type settings group
            group3 = matlab.system.display.SectionGroup(...
                'Title', 'Data Type', ...
                'PropertyList',{'variable_type'});

            % Input settings group, includes checkbox for input enables
            group4 = matlab.system.display.SectionGroup(...
                'Title', 'Inputs', ...
                'PropertyList',{'enableAdaptInput'});

            % Initialization settings group
            group5 = matlab.system.display.SectionGroup(...
                'Title', 'Initialize', ...
                'PropertyList',{'initialWeightsScale',...
                'initialWeightsSeed'});

            % Adding a "Visualize" button to the Outputs group
            group2.Actions = ...
                matlab.system.display.Action(@(actionData,obj)...
                visualize(obj,actionData),'Label','Visualize');

            % Combine all groups into tabs
            tabs = [group1, group2, group3, group4, group5];
        end
    end

    methods (Access = protected)
        % Define block icon text
        function icon = getIconImpl(~)
            icon = sprintf('Implicit Adaptive\n Feedforward controller');
        end

        % Define input port names
        function [in1name, in2name, in3name] = getInputNamesImpl(~)
            in1name = 'in';         % Main input signal
            in2name = 'error';      % Error signal input
            in3name = 'adapt?';     % Adaptive input enable signal
        end

        % Define output port names dynamically based on enabled outputs
        function varargout = getOutputNamesImpl(obj)

            varargout{1} = 'Out';   % Primary output
            out_ind = 2;            % Start indexing for optional outputs

            if obj.enableOutputDeDt

                % Predicted rate of change of error
                varargout{out_ind} = 'Predicted de/dt'; 

                % Actual rate of change of error
                varargout{out_ind + 1} = 'Actual de/dt';  
                out_ind = out_ind + 2;
            end

            if obj.enableOutputDeDu

                % Estimated derivative of error w.r.t input
                varargout{out_ind} = 'Estimated de/du';   
                out_ind = out_ind + 1;
            end

            if obj.enableOutputWeights

                % Reference filter weights
                varargout{out_ind} = 'Filter weights'; 
            end
        end
    end
end
