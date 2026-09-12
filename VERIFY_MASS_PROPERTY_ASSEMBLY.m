% VERIFY_MASS_PROPERTY_ASSEMBLY.m
% Regression checks for center-of-mass and parallel-axis assembly.

clearvars;
baseMass = 10;
baseCG = [0;0;0];
baseInertia = diag([1,2,3]);
moduleMasses = [2;2];
modulePositions = [-0.5,0,0;0.5,0,0];
moduleInertias = repmat(diag([0.1,0.2,0.3]),1,1,2);
[mass,cg,inertia] = ASSEMBLE_VEHICLE_MASS_PROPERTIES( ...
    baseMass,baseCG,baseInertia,moduleMasses,modulePositions,moduleInertias);

expected = diag([1.2,3.4,4.6]);
assert(abs(mass-14) < eps);
assert(norm(cg,inf) < eps);
assert(norm(inertia-expected,inf) < 1e-14);
fprintf('Mass-property assembly verification passed.\n');
fprintf('  Total mass %.3f kg | CG [%g %g %g] m\n',mass,cg);
fprintf('  Inertia diag [%g %g %g] kg m^2\n',diag(inertia));
