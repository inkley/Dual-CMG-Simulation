function env=ROLL_DISTURBANCE_ENVELOPE
% Provisional synthetic-load operating specification, NOT a sea-state rating.
% Applies only to the saved symmetric dual-CMG initialized 45-degree hold.
env.maxAmplitudeNm=.006;
env.frequencyHz=[1/15,1/5];
env.maxAbsBiasNm=.0005;
env.maxTotalDuration=200; % Bias acts throughout, including recovery.
env.loadStart=10; env.loadDuration=180; env.rampTime=2;
env.initialPhase=0;
env.momentumReserveFraction=.20;
env.diagnosticGimbalAngle=deg2rad(85);
env.criteria=ROLL_DISTURBANCE_TEST_PLAN;
env.periods=[5,10,15];
% Four amplitude/frequency/bias corners plus a zero-bias midpoint.
env.testPeriods=[5,5,15,15,10];
env.testBias=[-.0005,.0005,-.0005,.0005,0];
env.claim="Provisional finite-duration envelope; discrete nonlinear screening only.";
end
