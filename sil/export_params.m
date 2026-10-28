% Evaluate the Simulink parameter scripts (Octave or MATLAB) and emit JSON.
here = fileparts(mfilename('fullpath'));
run(fullfile(here, '..', 'Simulink', 'SystemParameters.m'));
run(fullfile(here, '..', 'Simulink', 'SampleTime.m'));
printf('{"C1": %.17g, "C2": %.17g, "R1": %.17g, "R2": %.17g, "num": %.17g, "den": [%.17g, %.17g, %.17g], "sample_time": %.17g}\n', ...
       C1, C2, R1, R2, num, den(1), den(2), den(3), sample_time);
