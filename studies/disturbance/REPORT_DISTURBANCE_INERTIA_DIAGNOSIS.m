function REPORT_DISTURBANCE_INERTIA_DIAGNOSIS
root=CMG_ROOT();
out=fullfile(root,'Working Results','roll_disturbance_envelope','unequal_inertia');
a=load(fullfile(out,'case_1.mat'));
d=load(fullfile(out,'allocation_diagnosis.mat'));
e=load(fullfile(out,'exact_knowledge_diagnostic.mat'));
[~,k]=max(abs(a.x(:,4)-a.env.criteria.holdAngle));
x=a.x(k,:); hs=[a.a.b.gyro1.I*x(14),a.a.b.gyro2.I*x(16)];
B=-[hs.*cos(x([13,15]));hs.*sin(x([13,15]))];
rhs=a.h.requestedMoment(k,1:2).'-B*[x(12);x(12)];
capacity=ROLL_FEASIBILITY_DIAGNOSTIC(B,rhs,a.c.limits.maxGimbalRate);
fprintf('At error peak: commanded gimbal rates %.6f / %.6f rad/s; true exact K/M rates %.6f / %.6f rad/s\n',a.h.commandedAlphadot(k,:),capacity.exactRates);
fprintf('Static true roll margin %.6f N m; bounded K/M residual %.9g N m\n',capacity.rollMargin,capacity.boundedResidual);
f=figure('Visible','off','Color','w','Position',[100 100 1050 850]); cleanup=onCleanup(@() close(f));
tiledlayout(3,1);
nexttile; plot(a.t,rad2deg(a.x(:,4)-a.env.criteria.holdAngle),e.t,rad2deg(e.x(:,4)-a.env.criteria.holdAngle),'LineWidth',1.2);
yline(2,':'); yline(-2,':'); ylabel('Roll error (deg)'); legend('Nominal inertia estimate','Exact inertia estimate','Location','best');
grid on; xlim([175,182]);
nexttile; plot(a.t,[a.h.requestedMoment(:,1),a.h.achievedMoment(:,1)],'LineWidth',1.2);
ylabel('Roll moment (N m)'); legend('Requested','Achieved','Location','best'); grid on; xlim([175,182]);
nexttile; plot(a.t,[d.allocationResidual(:,1),d.estimationResidual(:,1),d.servoResidual(:,1)],'LineWidth',1.2);
ylabel('K residual contribution (N m)'); xlabel('Time (s)'); grid on; xlim([175,182]);
legend('Damped allocation','Inertia estimation','Servo dynamics','Location','best');
sgtitle('Unequal rotor inertia: late-cycle disturbance-tracking failure');
exportgraphics(f,fullfile(out,'inertia_failure_diagnosis.png'),'Resolution',180);
save(fullfile(out,'peak_feasibility.mat'),'capacity','k');
end
