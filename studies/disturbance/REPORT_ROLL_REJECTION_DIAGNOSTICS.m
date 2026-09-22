function REPORT_ROLL_REJECTION_DIAGNOSTICS
% Compare resolution and momentum histories without changing pass criteria.
root=CMG_ROOT();
out=fullfile(root,'Working Results','roll_disturbance_tests');
b=load(fullfile(out,'A0.03_T15_feedback1.mat'));
fine=load(fullfile(out,'fine_step.mat'));
fineOnBase=interp1(fine.t,fine.x,b.t);
rollDifferenceDeg=max(abs(rad2deg(fineOnBase(:,4)-b.x(:,4))));
momentum=@(a) a.b.gyro1.I*a.x(:,14).*sin(a.x(:,13))+a.b.gyro2.I*a.x(:,16).*sin(a.x(:,15));
momentumDifference=max(abs(interp1(fine.t,momentum(fine),b.t)-momentum(b)));
fprintf('Fine/coarse max roll difference: %.9g deg; rotor Hx difference %.9g N m s\n',rollDifferenceDeg,momentumDifference);
assert(rollDifferenceDeg<.001 && momentumDifference<1e-5,'Resolution check failed.');
f=figure('Visible','off','Color','w','Position',[100 100 1050 850]);
cleanup=onCleanup(@() close(f));
names=["extended_zero_mean","extended_bias_0p003Nm"];
tiledlayout(3,1);
for row=1:3
    nexttile; hold on;
    for name=names
        a=load(fullfile(out,name+".mat"));
        assert(a.analysis.peakBalanceResidual<1e-5,'Scalar momentum balance failed.');
        switch row
            case 1, y=rad2deg(a.x(:,4)-a.plan.holdAngle); label='Roll error (deg)';
            case 2, H=momentum(a); y=H-H(1); label='Rotor H_x change (N m s)';
            case 3, y=rad2deg(max(abs(a.x(:,[13,15])),[],2)); label='Max |gimbal angle| (deg)';
        end
        plot(a.t,y,'LineWidth',1.2);
    end
    grid on; ylabel(label); xlabel('Time (s)');
    legend('Zero-mean sinusoid','Sinusoid + 0.003 N m bias','Location','best');
end
sgtitle('Finite-duration roll rejection and momentum storage (no unloading)');
exportgraphics(f,fullfile(out,'roll_momentum_diagnostics.png'),'Resolution',180);
end
