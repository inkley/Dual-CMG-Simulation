function VERIFY_INSTALLED_MASS_UNCERTAINTY
root=fileparts(mfilename('fullpath'));
b=load(fullfile(root,'Working Results','dual','symmetric_spin','VFR','simulation_result.mat'));
[a,p]=APPLY_INSTALLED_MASS_UNCERTAINTY(b.auv,b.params,[1,1,1]);
assert(a.m==b.auv.m && p.Ix==b.params.Ix && p.Iy==b.params.Iy && p.Iz==b.params.Iz);
[a,p]=APPLY_INSTALLED_MASS_UNCERTAINTY(b.auv,b.params,[1.1,1.2,.8]);
assert(abs(a.m/b.auv.m-1.1)<1e-12 && p.m==a.m);
assert(abs(p.Ix/b.params.Ix-1.2)<1e-12 && abs(p.Iy/b.params.Iy-.8)<1e-12);
assert(p.xg==b.params.xg && p.yg==b.params.yg && p.zg==b.params.zg);
failed=false;
try, APPLY_INSTALLED_MASS_UNCERTAINTY(b.auv,b.params,[-1,1,1]); catch, failed=true; end
assert(failed);
disp('Installed mass scaling, consistency and invalid-input checks passed.');
end
