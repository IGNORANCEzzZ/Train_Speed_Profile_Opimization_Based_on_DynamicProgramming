% Performance comparison script
clc; clear;

fprintf('=== Dynamic Programming Performance Comparison ===\n\n');

modes = {'fast', 'balanced', 'accurate'};
results = [];

for i = 1:length(modes)
    mode = modes{i};
    fprintf('Testing %s mode...\n', mode);
    
    try
        % Load and configure
        Global;
        ConfigureOptimization(mode);
        [wj,~,~]=GetAddResistance();
        [Dis_Space,MaxCapacityV]=MaxCapacityCurve(1);
        
        % Run optimization
        lambda = 632858.8509;
        tic;
        [s,v,F,T_real,E,Matrix_Jmin,Et,Eb,T1] = DynamicProgram(lambda);
        elapsed_time = toc;
        
        % Store results
        results = [results; {mode, elapsed_time, T_real, E/1000, N, Speed_N+1}];
        
        fprintf('  Time: %.2f s, Runtime: %.1f s, Energy: %.0f kJ\n', ...
                elapsed_time, T_real, E/1000);
        
    catch ME
        fprintf('  ERROR: %s\n', ME.message);
        results = [results; {mode, NaN, NaN, NaN, N, Speed_N+1}];
    end
    
    fprintf('\n');
end

% Display comparison table
fprintf('=== Performance Comparison Results ===\n');
fprintf('%-10s %-12s %-12s %-12s %-8s %-8s\n', ...
        'Mode', 'Calc Time(s)', 'Runtime(s)', 'Energy(kJ)', 'N', 'Speed_N');
fprintf('%-10s %-12s %-12s %-12s %-8s %-8s\n', ...
        '----', '--------', '--------', '--------', '---', '-------');

for i = 1:size(results, 1)
    row = results(i, :);
    if isnan(row{2})
        fprintf('%-10s %-12s %-12s %-12s %-8d %-8d\n', ...
                row{1}, 'FAILED', '-', '-', row{5}, row{6});
    else
        fprintf('%-10s %-12.2f %-12.1f %-12.0f %-8d %-8d\n', ...
                row{1}, row{2}, row{3}, row{4}, row{5}, row{6});
    end
end

% Calculate speedup ratios
if size(results, 1) >= 2 && ~isnan(results{end, 2})
    fastest_time = results{1, 2};
    for i = 2:size(results, 1)
        if ~isnan(results{i, 2})
            speedup = results{i, 2} / fastest_time;
            fprintf('\n%s mode is %.1fx slower than fast mode\n', ...
                    results{i, 1}, speedup);
        end
    end
end

fprintf('\n=== Recommendations ===\n');
fprintf('• Use FAST mode for testing and development\n');
fprintf('• Use BALANCED mode for production runs\n');
fprintf('• Use ACCURATE mode only when highest precision needed\n');
fprintf('• Avoid ORIGINAL mode unless absolutely necessary\n');