function ConfigureOptimization(mode)
% Configure optimization parameters for different performance/accuracy trade-offs
% Input: mode - 'fast', 'balanced', 'accurate'

global step_s;
global step_v;

switch lower(mode)
    case 'fast'
        % Fast mode: ~100x speedup, good for testing
        step_s = 10;    % 10m spatial resolution
        step_v = 0.2;   % 0.2 m/s speed resolution
        fprintf('Fast mode: step_s=%.0fm, step_v=%.1fm/s\n', step_s, step_v);
        
    case 'balanced'
        % Balanced mode: ~25x speedup, good accuracy
        step_s = 5;     % 5m spatial resolution  
        step_v = 0.1;   % 0.1 m/s speed resolution
        fprintf('Balanced mode: step_s=%.0fm, step_v=%.1fm/s\n', step_s, step_v);
        
    case 'accurate'
        % Accurate mode: High precision, slower
        step_s = 2;     % 2m spatial resolution
        step_v = 0.05;  % 0.05 m/s speed resolution
        fprintf('Accurate mode: step_s=%.0fm, step_v=%.2fm/s\n', step_s, step_v);
        
    case 'original'
        % Original high-precision mode
        step_s = 1;     % 1m spatial resolution
        step_v = 0.01;  % 0.01 m/s speed resolution
        fprintf('Original mode: step_s=%.0fm, step_v=%.2fm/s (VERY SLOW!)\n', step_s, step_v);
        
    otherwise
        error('Mode must be: fast, balanced, accurate, or original');
end

% Recalculate dependent parameters
global Speed_N;
global MaxSpeed;
global N;
global start_pos;
global end_pos;

Speed_N = ceil(MaxSpeed/3.6/step_v);
N = ceil(abs(start_pos-end_pos)/step_s);

% Display performance estimate
matrix_size = Speed_N + 1;
memory_per_iter = matrix_size^2 * 8 / 1024^2; % MB
total_iterations = N - 1;

fprintf('\nPerformance Estimate:\n');
fprintf('  Spatial points: %d\n', N);
fprintf('  Speed states: %d\n', Speed_N+1);
fprintf('  Matrix size: %dx%d\n', matrix_size, matrix_size);
fprintf('  Memory per iteration: %.1f MB\n', memory_per_iter);
fprintf('  Total iterations: %d\n', total_iterations);

if memory_per_iter > 100
    fprintf('  WARNING: Large memory usage! Consider faster mode.\n');
elseif memory_per_iter < 1
    fprintf('  Excellent: Low memory usage, fast execution expected.\n');
else
    fprintf('  Good: Moderate memory usage, reasonable speed.\n');
end

fprintf('\n');
end