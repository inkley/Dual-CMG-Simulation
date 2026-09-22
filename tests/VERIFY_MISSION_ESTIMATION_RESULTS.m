function VERIFY_MISSION_ESTIMATION_RESULTS
root=CMG_ROOT();
s=load(fullfile(root,'Working Results','mission_estimation','sweep.mat'));
assert(height(s.results)==9);
for k=1:9
    folder=['estimation_',char(s.cases(k).name)];
    a=load(fullfile(root,'Working Results','roll_turn_surge', ...
        'plane_45_heading_45_dt_0.05_timeout_15',folder,'mission.mat'));
    assert(all(isfinite(a.history.state),'all'));
    assert(norm(a.cfg.estimation.speedScaleBias-s.cases(k).bias)<1e-12);
    trueI=[a.b.gyro1.I,a.b.gyro2.I];
    assert(norm(trueI./a.cfg.estimation.rotorInertia-1-s.cases(k).inertiaError)<1e-12);
    assert(max(abs(abs(a.history.state(:,[14,16]))-1200*2*pi/60),[],'all')<1e-10);
    if a.result.terminal=="ABORT"
        assert(~any(a.history.phase=="SURGE") && all(a.history.propulsionForce==0));
        j=find(a.history.phase=="TURN",1,'last');
        fprintf('Case%d last TURN heading/plane/rate: %.4f / %.4f deg / %.4f deg/s\n', ...
            k,rad2deg(a.history.headingError(j)),rad2deg(a.history.planeError(j)), ...
            rad2deg(a.history.angularRate(j)));
    end
    if ismember(k,[6,9])
        f=load(fullfile(root,'Working Results','roll_turn_surge', ...
            'plane_45_heading_45_dt_0.025_timeout_15',folder,'mission.mat'));
        assert(a.result.terminal==f.result.terminal && a.result.passes==f.result.passes);
        assert(abs(a.result.finalHeadingErrorDeg-f.result.finalHeadingErrorDeg)<.02);
        if isfinite(a.result.maxSurgePlaneErrorDeg)
            assert(abs(a.result.maxSurgePlaneErrorDeg-f.result.maxSurgePlaneErrorDeg)<.02);
        end
        fprintf('Case%d refined outcome %s, pass%d; heading difference %.6f deg\n', ...
            k,f.result.terminal,f.result.passes, ...
            abs(a.result.finalHeadingErrorDeg-f.result.finalHeadingErrorDeg));
    end
end
fprintf('%d/9 completed; %d/9 passed the unchanged screen.\n', ...
    nnz(s.results.terminal=="COMPLETE"),nnz(s.results.passes));
disp('Mission estimator truth separation, spin-state and refinement checks passed.');
end
