function VERIFY_OBLIQUE_SHAPING_RESULTS()
% Check saved candidate trajectories and compare the half-step reruns.
root=CMG_ROOT();
out=fullfile(root,'Working Results','oblique_preload_shaping');
s=load(fullfile(out,'sweep.mat')); fine=load(fullfile(out,'refinement','sweep.mat'));
assert(height(s.results)==20 && height(fine.results)==2);
assert(all(s.results.passes([10,20])) && all(fine.results.passes));
assert(max(abs(s.results.finalHeadingErrorDeg([10,20])- ...
    fine.results.finalHeadingErrorDeg))<.001,'Heading refinement check failed');
assert(max(abs(s.results.maxGimbalDeg([10,20])-fine.results.maxGimbalDeg))<.001);
% Independent exact-inverse feasibility check at sampled candidate states.
% Finding a rate-bounded exact solution is sufficient for instantaneous
% feasibility; it does not establish global reachability or servo feasibility.
for j=[10,20]
    z=s.histories{j}; peakRequired=0; maxResidual=0;
    for k=1:numel(z.time)
        x=z.state(k,:); h=[s.b.gyro1.I*x(14),s.b.gyro2.I*x(16)];
        B=-[h.*cos(x([13,15]));h.*sin(x([13,15]))];
        rhs=[z.moments(k,1);0]-B*[x(12);x(12)];
        rates=pinv(B)*rhs;
        peakRequired=max(peakRequired,max(abs(rates)));
        maxResidual=max(maxResidual,norm(B*rates-rhs));
    end
    assert(peakRequired<=s.b.cmgConfig.limits.maxGimbalRate && maxResidual<1e-8);
    fprintf('Case %d: exact feasible rate peak %.6f rad/s; residual %.3e N m\n',j,peakRequired,maxResidual);
end
fprintf('Both shaped candidates pass half-step and sampled roll-authority checks.\n');

fig=figure('Visible','off','Position',[100 100 1250 850]); tiledlayout(2,2);
colors=lines(2);
for index=1:2
    j=[10,20]; z=s.histories{j(index)};
    nexttile(index); plot(z.time,z.shapedAngle,'k--','LineWidth',1.5); hold on;
    actual=zeros(size(z.time)); lateral=z.desired.hybrid.lateralDirectionNED;
    for k=1:numel(z.time)
        theta=z.state(k,5); psi=z.state(k,6);
        nose=[cos(psi)*cos(theta);sin(psi)*cos(theta);-sin(theta)];
        actual(k)=rad2deg(atan2(dot(nose,lateral),nose(1)));
    end
    plot(z.time,actual,'LineWidth',1.7,'Color',colors(index,:));
    grid on; xlabel('Time (s)'); ylabel('In-plane heading (deg)');
    title(sprintf('%+g deg plane and heading',s.results.planeDeg(j(index))));
    legend('Shaped reference','Achieved projected heading','Location','best');
end
nexttile(3); hold on;
for index=1:2
    js=[10,20]; z=s.histories{js(index)};
    plot(z.time,z.gate,'Color',colors(index,:),'LineWidth',1.5, ...
        'DisplayName',sprintf('%+g deg',s.results.planeDeg(js(index))));
end
ylim([0 1.05]); grid on; xlabel('Time (s)'); ylabel('Thrust-enable multiplier');
title('Unchanged thrust gating'); legend('Location','best');
nexttile(4); hold on;
for index=1:2
    js=[10,20]; z=s.histories{js(index)};
    plot(z.time,max(abs(rad2deg(z.state(:,[13,15]))),[],2), ...
        'Color',colors(index,:),'LineWidth',1.5, ...
        'DisplayName',sprintf('%+g deg',s.results.planeDeg(js(index))));
end
yline(100,'k--','DisplayName','Angle limit'); grid on; xlabel('Time (s)');
ylabel('Maximum absolute gimbal angle (deg)'); title('Remaining gimbal travel');
legend('Location','best');
exportgraphics(fig,fullfile(out,'SHAPED_OBLIQUE_TURNS.png'),'Resolution',200); close(fig);
end
