function VERIFY_ACTUATOR_UNCERTAINTY_RESULTS
root=fileparts(mfilename('fullpath'));
s=load(fullfile(root,'Working Results','actuator_uncertainty','sweep.mat'));
assert(height(s.results)==9 && all(s.results.terminal=="COMPLETE"));
assert(all(s.results.passes) && all(s.results.limitSamples==0));
fine=load(fullfile(root,'Working Results','roll_turn_surge', ...
    'plane_45_heading_45_dt_0.025_timeout_15','actuator_combined','mission.mat'));
coarse=s.missions{9};
assert(fine.result.passes);
assert(abs(fine.result.finalHeadingErrorDeg-coarse.finalHeadingErrorDeg)<.01);
assert(abs(fine.result.maxSurgePlaneErrorDeg-coarse.maxSurgePlaneErrorDeg)<.01);
assert(abs(fine.result.finalSpeed-coarse.finalSpeed)<.001);
assert(abs(fine.result.finalProgress-coarse.finalProgress)<.01);
fprintf('Combined half-step heading/plane differences %.6f/%.6f degrees\n', ...
    abs(fine.result.finalHeadingErrorDeg-coarse.finalHeadingErrorDeg), ...
    abs(fine.result.maxSurgePlaneErrorDeg-coarse.maxSurgePlaneErrorDeg));
disp('All nine actuator scenarios and combined-case refinement passed.');
end
