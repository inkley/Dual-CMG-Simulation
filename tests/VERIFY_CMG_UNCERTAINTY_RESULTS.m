function VERIFY_CMG_UNCERTAINTY_RESULTS
root=CMG_ROOT();
out=fullfile(root,'Working Results','estimation_uncertainty');
a=load(fullfile(out,'sweep_dt_0.01.mat'));
b=load(fullfile(out,'sweep_dt_0.005.mat'));
assert(height(a.results)==36 && height(b.results)==36);
assert(all(a.results.finalTime==5) && all(b.results.finalTime==5));
assert(isequal(a.results.rollPass,b.results.rollPass));
for field=["finalRollErrorDeg","peakPitchDeg","peakYawDeg","peakGimbalDeg"]
    difference=max(abs(a.results.(field)-b.results.(field)));
    assert(difference<.01,'Step sensitivity exceeded .01 degrees: %s',field);
    fprintf('%s half-step difference: %.6g deg\n',field,difference);
end
for mode=["single","dual"]
    r=a.results(a.results.mode==mode,:);
    fprintf('%s: %d/%d roll passes; worst settling %.3fs, error %.4fdeg, pitch/yaw %.4f/%.4fdeg, angle %.3fdeg, speed %.2frpm\n', ...
        mode,nnz(r.rollPass),height(r),max(r.settlingTime),max(abs(r.finalRollErrorDeg)), ...
        max(r.peakPitchDeg),max(r.peakYawDeg),max(r.peakGimbalDeg),max(r.peakSpeedRPM));
end
% Time-weighted physical inverse error includes estimator mismatch and damping,
% but excludes finite-rate actuator tracking loss (available separately).
inverse=zeros(36,1);
for k=1:36
    h=a.histories{k}; inverse(k)=sqrt(trapz(h.t,h.control.allocationError(:,1).^2)/5);
end
fprintf('Worst physical unconstrained roll-inverse RMSE: %.6f N m\n',max(inverse));
fig=figure('Visible','off'); tiledlayout(1,2);
nexttile; scatter(1:36,abs(a.results.finalRollErrorDeg),'filled');
ylabel('Final |roll error| (deg)'); xlabel('Case index'); grid on;
nexttile; plot(1:36,[a.results.peakPitchDeg,a.results.peakYawDeg]);
ylabel('Peak attitude (deg)'); xlabel('Case index'); legend('Pitch','Yaw'); grid on;
exportgraphics(fig,fullfile(out,'UNCERTAINTY_SUMMARY.png'),'Resolution',180); close(fig);
disp('Uncertainty sweep completeness and half-step checks passed.');
end
