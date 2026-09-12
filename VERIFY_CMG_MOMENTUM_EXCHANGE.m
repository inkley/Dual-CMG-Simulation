% VERIFY_CMG_MOMENTUM_EXCHANGE.m
% Algebraic regression test for CMG/vehicle angular-momentum exchange.
% In an isolated system, CMG.m must return the exact negative inertial
% derivative of rotor angular momentum, so rotor and vehicle momentum-rate
% contributions cancel before external moments are added.

clearvars;
rng(42);
sampleCount = 1000;
maximumReactionError = 0;

gyro1.I = 2.91e-5;
gyro2.I = 3.17e-5;
for index = 1:sampleCount
    state = zeros(16,1);
    state(10:12) = 4*(rand(3,1)-0.5);
    state(13) = 2*pi*(rand-0.5);
    state(14) = 400*(rand-0.5);
    state(15) = 2*pi*(rand-0.5);
    state(16) = 400*(rand-0.5);
    contpar.alphadot1 = 40*(rand-0.5);
    contpar.Omegadot1 = 1000*(rand-0.5);
    contpar.alphadot2 = 40*(rand-0.5);
    contpar.Omegadot2 = 1000*(rand-0.5);

    [tau1, tau2] = CMG(gyro1, gyro2, contpar, state);
    omegaBody = state(10:12);
    rotorRate1 = rotorMomentumRate(gyro1.I, state(13), state(14), ...
        contpar.alphadot1, contpar.Omegadot1, omegaBody);
    rotorRate2 = rotorMomentumRate(gyro2.I, state(15), state(16), ...
        contpar.alphadot2, contpar.Omegadot2, omegaBody);
    vehicleReaction = [tau1.K+tau2.K; tau1.M+tau2.M; tau1.N+tau2.N];
    maximumReactionError = max(maximumReactionError, ...
        norm(vehicleReaction+rotorRate1+rotorRate2, inf));
end

fprintf('CMG momentum-exchange regression: %d randomized states\n', sampleCount);
fprintf('Maximum |tau_vehicle + dH_rotor/dt_N|: %.3e N m\n', ...
    maximumReactionError);
assert(maximumReactionError < 1e-12, ...
    'CMG action/reaction momentum identity failed.');

function Hdot = rotorMomentumRate(I, alpha, Omega, alphadot, Omegadot, omega)
    spinAxis = [sin(alpha); -cos(alpha); 0];
    spinAxisDerivative = [cos(alpha); sin(alpha); 0];
    H = I*Omega*spinAxis;
    Hdot = I*Omegadot*spinAxis ...
        + I*Omega*alphadot*spinAxisDerivative + cross(omega,H);
end
