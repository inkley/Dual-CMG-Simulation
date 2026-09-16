function VERIFY_INSTALLED_MASS_RESULTS
root=fileparts(mfilename('fullpath'));
out=fullfile(root,'Working Results','installed_mass_uncertainty');
s=load(fullfile(out,'sweep.mat'));
assert(height(s.results)==9 && all(isfinite(s.results.massKg)));
for k=1:9
    assert(all(isfinite(s.rolls{k}.x),'all'));
    assert(abs(s.rolls{k}.auv.m-s.b.auv.m*s.scales(k,1))<1e-10);
    assert(s.rolls{k}.params.m==s.rolls{k}.auv.m);
end
for k=2:3
    assert(max(abs(s.rolls{k}.x(:,4)-s.rolls{1}.x(:,4)))<1e-8, ...
        'Mass alone should not change the symmetric roll-only baseline.');
end
fprintf('Roll screen: %d/9; mission screen: %d/9\n', ...
    nnz(s.results.rollPass),nnz(s.results.missionPass));
fprintf('Worst roll settling %.3fs; worst roll final error %.3fdeg\n', ...
    max(s.results.rollSettlingTime),max(abs(s.results.rollFinalErrorDeg)));
disp(s.results(:,{'caseIndex','missionTerminal','missionPass', ...
    'finalHeadingErrorDeg','maxPlaneErrorDeg','outOfPlane','peakAftForce','limitSamples'}));
fig=figure('Visible','off'); tiledlayout(1,2);
nexttile; bar(s.results.caseIndex,s.results.rollSettlingTime); yline(4,'--');
xlabel('Case index (see summary.csv)'); ylabel('Roll settling time (s)'); grid on;
nexttile; bar(s.results.caseIndex,[s.results.finalHeadingErrorDeg,s.results.maxPlaneErrorDeg]);
yline(1,'--'); xlabel('Case index'); ylabel('Mission error (deg)');
legend('Final heading','Peak surge plane'); grid on;
exportgraphics(fig,fullfile(out,'MASS_UNCERTAINTY.png'),'Resolution',180); close(fig);
disp('Installed-mass sweep integrity and roll decoupling checks passed.');
fine=load(fullfile(root,'Working Results','roll_turn_surge', ...
    'plane_45_heading_45_dt_0.025_timeout_15','mass_1.1_roll_1.2_transverse_1.2','mission.mat'));
coarse=s.missions{9};
assert(fine.result.passes==coarse.passes);
assert(abs(fine.result.finalHeadingErrorDeg-coarse.finalHeadingErrorDeg)<.01);
assert(abs(fine.result.maxSurgePlaneErrorDeg-coarse.maxSurgePlaneErrorDeg)<.01);
assert(abs(fine.result.finalSpeed-coarse.finalSpeed)<.001);
assert(abs(fine.result.finalProgress-coarse.finalProgress)<.01);
fprintf('High-corner half-step heading/plane differences %.6f/%.6f deg\n', ...
    abs(fine.result.finalHeadingErrorDeg-coarse.finalHeadingErrorDeg), ...
    abs(fine.result.maxSurgePlaneErrorDeg-coarse.maxSurgePlaneErrorDeg));
end
