function VERIFY_ROLL_TURN_SURGE_RESULTS()
% Verify the saved positive/mirrored/horizontal missions and refinement.
root=fullfile(CMG_ROOT(),'Working Results','roll_turn_surge');
names={'plane_45_heading_45_dt_0.05_timeout_15', ...
    'plane_-45_heading_-45_dt_0.05_timeout_15', ...
    'plane_0_heading_30_dt_0.05_timeout_15'};
for j=1:numel(names)
    a=load(fullfile(root,names{j},'mission.mat'));
    assert(a.result.passes && a.result.terminal=="COMPLETE");
    assert(isequal(a.result.events.phase,["ROLL";"TURN";"SURGE";"COMPLETE"]));
    pre=a.history.phase=="ROLL" | a.history.phase=="TURN";
    assert(all(a.history.propulsionCommand(pre)==0) && all(a.history.propulsionForce(pre)==0));
    first=find(a.history.phase=="SURGE",1);
    assert(a.history.headingError(first)<=a.m.headingCaptureAngle ...
        && a.history.planeError(first)<=a.m.planeCaptureAngle ...
        && a.history.angularRate(first)<=a.m.turnCaptureRate);
    fprintf('%s passed.\n',names{j});
end
coarse=load(fullfile(root,names{1},'mission.mat'));
fine=load(fullfile(root,'plane_45_heading_45_dt_0.025_timeout_15','mission.mat'));
assert(fine.result.passes);
assert(abs(coarse.result.finalHeadingErrorDeg-fine.result.finalHeadingErrorDeg)<.01);
assert(abs(coarse.result.finalSpeed-fine.result.finalSpeed)<.001);
assert(abs(coarse.result.finalProgress-fine.result.finalProgress)<.01);
assert(abs(coarse.result.peakAftForce-fine.result.peakAftForce)<.03);
abort=load(fullfile(root,'plane_45_heading_45_dt_0.05_timeout_0.1','mission.mat'));
assert(abort.result.terminal=="ABORT" && ~any(abort.history.phase=="SURGE"));
assert(all(abort.history.propulsionCommand==0) && all(abort.history.propulsionForce==0));
fprintf('Half-step, transition guards, and no-surge timeout checks passed.\n');
end
