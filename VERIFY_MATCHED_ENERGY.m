function VERIFY_MATCHED_ENERGY
root=fileparts(mfilename('fullpath')); out=fullfile(root,'Working Results','matched_energy');
a=load(fullfile(out,'comparison_0.01.mat')); b=load(fullfile(out,'comparison_0.005.mat'));
assert(height(a.results)==4 && isequal(a.results.passes,b.results.passes));
for k=[1,3]
    for name=["initialSpinJ","totalRotorMass","totalSpinInertia","installedMass"]
        assert(abs(a.results.(name)(k)-a.results.(name)(k+1))<1e-10);
    end
end
assert(max(abs(a.results.finalErrorDeg-b.results.finalErrorDeg))<.01);
assert(max(abs(a.results.gimbalDeg-b.results.gimbalDeg))<.01);
assert(max(abs(a.results.rollGrossJ-b.results.rollGrossJ))<1e-4);
disp('Matched initial energy/mass/inertia and half-step checks passed.');
end
