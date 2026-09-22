function [auv,params] = APPLY_INSTALLED_MASS_UNCERTAINTY(auv,params,scales)
% True assembled rigid-body properties only: [mass, roll inertia, pitch/yaw inertia].
% Rotor spin inertia, added mass, drag, geometry and controller estimates stay fixed.
% Not a physical reassembly or CG/product-of-inertia uncertainty model.
assert(numel(scales)==3 && all(isfinite(scales)) && all(scales>0));
assert(norm([params.xg,params.yg,params.zg])<1e-10, ...
    'This sensitivity model requires the zero-CG baseline.');
auv.m=auv.m*scales(1); params.m=auv.m;
auv.W=auv.m*auv.g;
params.Ix=params.Ix*scales(2);
params.Iy=params.Iy*scales(3); params.Iz=params.Iz*scales(3);
moments=[params.Ix,params.Iy,params.Iz];
assert(all(moments>0) && 2*max(moments)<=sum(moments), ...
    'Principal moments must satisfy the rigid-body triangle inequalities.');
% W is kept consistent for bookkeeping; the current model omits restoring loads.
end
