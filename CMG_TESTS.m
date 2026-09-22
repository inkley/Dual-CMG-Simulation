function results=CMG_TESTS(suite)
%CMG_TESTS Run isolated verification scripts/functions without full study sweeps.
% CMG_TESTS('unit') is the default. 'saved' needs existing Working Results.
% 'all' runs both. No production simulations or full publication batch here.
if nargin<1, suite='unit'; end
CMG_SETUP;
assert(any(strcmp(suite,{'unit','saved','all'})),'Use unit, saved, or all.');
unit={'VERIFY_CMG_TORQUE_SIGNS','VERIFY_CMG_MOMENTUM_EXCHANGE', ...
    'VERIFY_DUAL_CMG_CONDITIONING','VERIFY_MASS_PROPERTY_ASSEMBLY', ...
    'VERIFY_CMG_ESTIMATION','VERIFY_CMG_MECHANICAL_ENERGY', ...
    'VERIFY_AFT_PROPULSION','VERIFY_VORTEX_RING_THRUSTERS','VERIFY_THRUSTER_ALLOCATION', ...
    'VERIFY_ROLL_TO_PLANE_COMMAND','VERIFY_ROLL_TURN_SURGE', ...
    'VERIFY_INSTALLED_MASS_UNCERTAINTY','VERIFY_ACTUATOR_UNCERTAINTY', ...
    'VERIFY_PLANE_PITCH_CORRECTION','VERIFY_ROLL_FEASIBILITY_DIAGNOSTIC','VERIFY_REPOSITORY_LAYOUT'};
saved={'VERIFY_ROLL_TURN_SURGE_RESULTS','VERIFY_INSTALLED_MASS_RESULTS', ...
    'VERIFY_ACTUATOR_UNCERTAINTY_RESULTS','VERIFY_FLYWHEEL_GEOMETRY', ...
    'VERIFY_MATCHED_ENERGY','VERIFY_MISSION_ESTIMATION_RESULTS', ...
    'VERIFY_PLANE_PITCH_CORRECTION_RESULTS','VERIFY_DISTURBANCE_MISMATCH_RESULTS', ...
    'VERIFY_ROLL_DISTURBANCE_SETUP','VERIFY_ALLOCATION_SCOPE_DECISION', ...
    'REPORT_INERTIA_ESTIMATION_BOUNDS'};
names={};
if any(strcmp(suite,{'unit','all'})), names=[names,unit]; end
if any(strcmp(suite,{'saved','all'})), names=[names,saved]; end
passed=false(numel(names),1); messages=strings(numel(names),1);
for k=1:numel(names)
    fprintf('\n--- %s ---\n',names{k});
    try
        runIsolated(names{k}); passed(k)=true;
    catch problem
        messages(k)=string(problem.message);
        fprintf(2,'FAILED: %s\n',problem.message);
    end
end
results=table(string(names(:)),passed,messages,'VariableNames',{'test','passed','message'});
disp(results);
assert(all(passed),'CMG:RegressionFailure','One or more checks failed; inspect the table.');
end
function runIsolated(name)
% Legacy verification scripts use clearvars; keep them out of the runner scope.
eval([name,';']);
end
