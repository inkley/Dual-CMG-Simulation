function REPORT_ROLL_DISTURBANCE_TESTS
% Verify applied loads and visualize saved feedback/passive experiments.
root=CMG_ROOT();
out=fullfile(root,'Working Results','roll_disturbance_tests');
plan=ROLL_DISTURBANCE_TEST_PLAN;
f=figure('Visible','off','Color','w','Position',[100 100 1100 850]);
cleanup=onCleanup(@() close(f));
tiledlayout(2,2);
metrics=struct([]);
for pair=1:4
    nexttile; hold on;
    for j=1:2
        k=2+(pair-1)*2+j-1;
        a=load(fullfile(out,char(plan.cases(k).name)+".mat"));
        expected=SINUSOIDAL_ROLL_LOAD(a.t,a.c.external.sinusoidalRoll);
        assert(max(abs(a.h.rollDisturbance(:)-expected(:)))<1e-12);
        assert(max(abs(expected))<=plan.cases(k).amplitude+1e-12);
        assert(all(a.h.externalDumpMoment==0));
        % Adding a load must not leak perfect load knowledge into feedback.
        c0=a.c; c0.external.sinusoidalRoll.amplitude=0;
        ti=plan.load.startTime+plan.cases(k).period/4;
        [withLoad,~]=CMG_ALLOCATE(ti,a.x(1,:).',a.b.gains,a.b.gyro1,a.b.gyro2,a.d,a.loop,a.c);
        [noLoad,~]=CMG_ALLOCATE(ti,a.x(1,:).',a.b.gains,a.b.gyro1,a.b.gyro2,a.d,a.loop,c0);
        assert(withLoad.KD==noLoad.KD && withLoad.MD==noLoad.MD);
        assert(withLoad.KDisturbance>0 && noLoad.KDisturbance==0);
        if j==1, yyaxis left; else, yyaxis right; end
        plot(a.t,rad2deg(a.x(:,4)-a.plan.holdAngle),'LineWidth',1.2);
        if j==1
            ylabel('Feedback roll error (deg)');
            yline(plan.peakRollErrorDeg,':','HandleVisibility','off');
            yline(-plan.peakRollErrorDeg,':','HandleVisibility','off');
        else
            ylabel('No-feedback unwrapped error (deg)');
        end
        H=a.b.gyro1.I*a.x(:,14).*sin(a.x(:,13))+a.b.gyro2.I*a.x(:,16).*sin(a.x(:,15));
        m.name=plan.cases(k).name;
        m.peakGimbalDeg=rad2deg(max(abs(a.x(:,[13,15])),[],'all'));
        m.finalRotorHxChange=H(end)-H(1);
        m.externalRollImpulse=trapz(a.t,expected);
        if isempty(metrics), metrics=m; else, metrics(end+1)=m; end %#ok<AGROW>
    end
    grid on; xlabel('Time (s)');
    title(sprintf('A = %.2f N m, T = %g s',plan.cases(k).amplitude,plan.cases(k).period));
    legend('Feedback','No attitude feedback','Location','best');
end
sgtitle('Synthetic roll loads: initialized 45-degree hold');
exportgraphics(f,fullfile(out,'roll_disturbance_comparison.png'),'Resolution',180);
writetable(struct2table(metrics),fullfile(out,'load_diagnostics.csv'));
disp(struct2table(metrics));
disp('Applied-load bounds, replay consistency, no feedforward and no unloading checks passed.');
end
