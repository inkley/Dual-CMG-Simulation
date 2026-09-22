function VERIFY_MISSION_ESTIMATION_ABLATION
root=CMG_ROOT(); cases=MISSION_ESTIMATION_CASES();
s=load(fullfile(root,'Working Results','mission_estimation_ablation','ablation.mat'));
assert(height(s.results)==9);
for k=10:13
    c=cases(k);
    if k<=11, assert(all(c.inertiaError==0)); else, assert(all(c.bias==0)); end
    assert(c.bias(1)==-c.bias(2) && c.inertiaError(1)==-c.inertiaError(2));
end
for k=[10,12]
    folder=['estimation_',char(cases(k).name)];
    a=load(fullfile(root,'Working Results','roll_turn_surge', ...
        'plane_45_heading_45_dt_0.05_timeout_15',folder,'mission.mat'));
    f=load(fullfile(root,'Working Results','roll_turn_surge', ...
        'plane_45_heading_45_dt_0.025_timeout_15',folder,'mission.mat'));
    assert(a.result.terminal==f.result.terminal);
    assert(abs(a.result.finalHeadingErrorDeg-f.result.finalHeadingErrorDeg)<.02);
    if a.result.terminal=="ABORT"
        assert(~any(a.history.phase=="SURGE") && all(a.history.propulsionForce==0));
    end
    fprintf('Isolated case%d: %s, finer-step final heading difference %.6f deg\n', ...
        k,a.result.terminal,abs(a.result.finalHeadingErrorDeg-f.result.finalHeadingErrorDeg));
    fprintf('Strict screen coarse/fine %d/%d, roll-infeasible samples %d/%d\n', ...
        a.result.passes,f.result.passes,a.result.rollInfeasibleSamples,f.result.rollInfeasibleSamples);
    idx=find(~f.history.rollFeasible);
    if ~isempty(idx), disp(f.history(idx,{'time','phase','headingError','planeError'})); end
end
disp('Single-factor isolation and representative refinement checks passed.');
end
