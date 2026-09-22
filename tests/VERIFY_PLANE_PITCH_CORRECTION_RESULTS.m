function VERIFY_PLANE_PITCH_CORRECTION_RESULTS
root=CMG_ROOT(); cases=MISSION_ESTIMATION_CASES();
s=load(fullfile(root,'Working Results','plane_pitch_correction','sweep.mat'));
assert(height(s.results)==13 && all(s.results.terminal=="COMPLETE"));
peakPitch=0; postCaptureFlags=0;
for k=1:13
    base=fullfile(root,'Working Results','roll_turn_surge', ...
        'plane_45_heading_45_dt_0.05_timeout_15',['estimation_',char(cases(k).name)]);
    legacy=load(fullfile(base,'mission.mat'));
    a=load(fullfile(base,'plane_pitch_correction','mission.mat'));
    for name=["rollCaptureAngle","rollCaptureRate","rollDwell","rollTimeout", ...
            "turnDuration","turnTimeout","headingCaptureAngle","planeCaptureAngle", ...
            "turnCaptureRate","turnDwell","headingAbortAngle","planeAbortAngle"]
        assert(isequal(a.m.(name),legacy.m.(name)));
    end
    assert(isequaln(a.b,legacy.b),'Plant must remain unchanged.');
    assert(isequaln(a.cfg.limits,legacy.cfg.limits));
    assert(all(abs(a.history.requestedCmgMoment(:,2))<=.02+eps));
    roll=a.history.phase=="ROLL";
    assert(all(a.history.requestedCmgMoment(roll,2)==0));
    assert(a.result.preSurgeForce==0 && ~any(a.result.limitedSamples));
    peakPitch=max(peakPitch,max(abs(a.history.requestedCmgMoment(:,2))));
    postCaptureFlags=postCaptureFlags+nnz(~a.history.rollFeasible & ~roll);
    if ismember(k,[6,9,10,12,13])
        f=load(fullfile(root,'Working Results','roll_turn_surge', ...
            'plane_45_heading_45_dt_0.025_timeout_15', ...
            ['estimation_',char(cases(k).name)],'plane_pitch_correction','mission.mat'));
        assert(f.result.terminal=="COMPLETE");
        assert(abs(f.result.finalHeadingErrorDeg-a.result.finalHeadingErrorDeg)<.01);
        assert(abs(f.result.maxSurgePlaneErrorDeg-a.result.maxSurgePlaneErrorDeg)<.02);
        fprintf('Case%d strict screen coarse/fine %d/%d\n',k,a.result.passes,f.result.passes);
    end
end
fprintf('Complete %d/13; strict screen %d/13; post-ROLL infeasible samples %d; peak pitch request %.6g Nm\n', ...
    nnz(s.results.terminal=="COMPLETE"),nnz(s.results.passes),postCaptureFlags,peakPitch);
assert(postCaptureFlags==0);
for angles=[-45,-45;0,30].'
    a=load(fullfile(root,'Working Results','roll_turn_surge', ...
        sprintf('plane_%g_heading_%g_dt_0.05_timeout_15',angles), ...
        'plane_pitch_correction','mission.mat'));
    assert(a.result.passes);
end
disp('Unchanged criteria/plant, bounded correction, capture recovery and nominal regressions passed.');
end
