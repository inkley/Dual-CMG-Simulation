function VERIFY_ROLL_FEASIBILITY_DIAGNOSTIC
d=ROLL_FEASIBILITY_DIAGNOSTIC(eye(2),[1;2],3);
assert(d.legacyFeasible && d.boundedResidual<1e-12);
d=ROLL_FEASIBILITY_DIAGNOSTIC(eye(2),[4;0],3);
assert(~d.legacyFeasible && abs(d.boundedResidual-1)<1e-12);
d=ROLL_FEASIBILITY_DIAGNOSTIC([1,1;0,0],[2;0],1);
assert(d.boundedResidual<1e-12);
d=ROLL_FEASIBILITY_DIAGNOSTIC([1,1;0,0],[0;1],1);
assert(abs(d.boundedResidual-1)<1e-12);
d=ROLL_FEASIBILITY_DIAGNOSTIC([1,1;1,1],[1;0],1);
assert(d.rollOnlyFeasible && ~d.boundedToleranceFeasible);
assert(abs([1,1]*d.rollPriorityRates-1)<1e-12);
assert(abs(d.rollPriorityPitchResidual-1)<1e-12);
d=ROLL_FEASIBILITY_DIAGNOSTIC(eye(2),[1;.5],2);
assert(d.rollOnlyFeasible && abs(d.rollPriorityPitchResidual)<1e-12);
assert(norm(d.rollPriorityRates-[1;.5])<1e-12);
root=CMG_ROOT();
s=load(fullfile(root,'Working Results','roll_feasibility_diagnostic','diagnostic.mat'));
assert(height(s.results)==3 && s.results.localIntegration(3));
assert(all(s.results.denseFlagCount>0));
assert(s.results.maxBoundedResidual(3)>1e-7);
assert(s.results.rollOnlyInfeasibleSamples(3)==0);
assert(s.results.peakRollPriorityPitchNm(3)>1e-7);
disp('Bounded feasibility tests and local-integration confirmation passed.');
end
