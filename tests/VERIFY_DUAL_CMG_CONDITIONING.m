% VERIFY_DUAL_CMG_CONDITIONING.m
% Regression checks for the analytic dual-CMG singularity geometry.

clearvars;
I = 1.03e-3;
Omega = 40*pi;
h = I*Omega;
matrixFor = @(a1,a2) -[h*cos(a1), -h*cos(a2); ...
                        h*sin(a1), -h*sin(a2)];

Borthogonal = matrixFor(-pi/4, pi/4);
Bparallel = matrixFor(0, 0);
Bantiparallel = matrixFor(-pi/2, pi/2);

assert(abs(cond(Borthogonal)-1) < 1e-12, ...
    'Orthogonal steering columns should have condition number one.');
assert(abs(det(Bparallel)) < 1e-15, ...
    'Parallel steering columns must be singular.');
assert(abs(det(Bantiparallel)) < 1e-15, ...
    'Antiparallel steering columns must be singular.');

angles = linspace(deg2rad(0.1), pi/2, 500);
numericalCondition = zeros(size(angles));
analyticCondition = sqrt((1+abs(cos(angles))) ...
    ./(1-abs(cos(angles))));
for index = 1:numel(angles)
    numericalCondition(index) = cond(matrixFor(-angles(index)/2, ...
        angles(index)/2));
end
assert(max(abs(numericalCondition-analyticCondition)) < 1e-8, ...
    'Numerical and analytic equal-momentum condition numbers disagree.');
fprintf('Dual-CMG conditioning verification passed.\n');
fprintf('  Orthogonal-column condition number: %.3f\n', cond(Borthogonal));
fprintf('  Parallel determinant: %.3e\n', det(Bparallel));
