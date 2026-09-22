function decision=VERIFY_ALLOCATION_SCOPE_DECISION
% Evidence gate for retaining the existing allocator in the bounded study.
% Not a test of a new uncertainty-aware allocator or a continuum guarantee.
root=CMG_ROOT();
out=fullfile(root,'Working Results','inertia_estimation_bounds');
[e1,e2]=ndgrid([-.05,0,.05]);
pairs=[e1(:),e2(:),repmat(-.0005,9,1); ...
    -.05,-.05,.0005;-.05,.05,.0005;.05,-.05,.0005;.05,.05,.0005;0,0,.0005];
peak=zeros(14,1); rmsError=peak; momentum=peak; damping=peak;
for k=1:14
    tag=sprintf('e1_%+.6f_e2_%+.6f_bias_%+.6f_step_0.01.mat',pairs(k,:));
    a=load(fullfile(out,tag));
    assert(a.s.pass && a.s.completed && a.s.limitedSamples==0);
    assert(abs(a.s.error1Percent-100*pairs(k,1))<1e-10);
    assert(abs(a.s.error2Percent-100*pairs(k,2))<1e-10);
    assert(a.s.bias==pairs(k,3));
    assert(a.a.env.criteria.peakRollErrorDeg==2 && a.a.env.criteria.rmsRollErrorDeg==1 ...
        && a.a.env.criteria.finalRecoveryErrorDeg==.5);
    assert(strcmp(a.c.dualController,'constant_speed'));
    assert(isequal(a.c.allocator,a.a.c.allocator));
    peak(k)=a.s.peakErrorDeg; rmsError(k)=a.s.rmsErrorDeg;
    momentum(k)=a.s.peakMomentumExcursion; damping(k)=a.c.allocator.damping;
end
assert(all(damping==damping(1)));
challenge=load(fullfile(out,'e1_-0.100000_e2_+0.100000_bias_-0.000500_step_0.005.mat'));
assert(~challenge.s.pass && challenge.s.peakErrorDeg>2);
decision.retainExistingAllocator=true;
decision.testedCases=14;
decision.minimumPeakErrorMarginDeg=2-max(peak);
decision.minimumRmsErrorMarginDeg=1-max(rmsError);
decision.maximumMomentumExcursion=max(momentum);
decision.damping=damping(1);
decision.outsideTargetFailurePeakDeg=challenge.s.peakErrorDeg;
decision.scope="Selected fixed-plant 200-second disturbed holds; not general robustness.";
disp(decision);
save(fullfile(out,'allocation_scope_decision.mat'),'decision','pairs');
end
