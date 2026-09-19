function m = ROLL_TURN_SURGE_DEFAULTS()
% Finite roll-turn-cruise demonstration, not waypoint arrival or stopping.
m.planeDeg=45; m.headingDeg=45;
m.sampleTime=.05; m.maxStep=.01;
m.rollCaptureAngle=deg2rad(.5); m.rollCaptureRate=deg2rad(.5);
m.rollDwell=.5; m.rollTimeout=15;
m.turnDuration=24; m.turnTimeout=50;
m.headingCaptureAngle=deg2rad(1); m.planeCaptureAngle=deg2rad(.5);
m.turnCaptureRate=deg2rad(.5); m.turnDwell=1;
m.surgeSpeed=.5; m.speedRampTime=10; m.surgeDuration=30;
m.headingAbortAngle=deg2rad(3); m.planeAbortAngle=deg2rad(2);
m.angularRateAbort=deg2rad(5); m.abortCoastTime=5;
m.speed.Kp=12; m.speed.Ki=2;
m.speed.dragCoefficient=1.62; % matches current REMUS surge drag
m.speed.effectiveMass=NaN; % assigned from saved vehicle mass + surge added mass
m.installedMassScales=[1,1,1]; % true mass, Ix, Iy/Iz; controller retains nominal mass
m.actuatorCase=0; % 0: unchanged baseline; 1:9: ACTUATOR_UNCERTAINTY_CASES
m.estimationCase=0; % 0 unchanged; 1:9 corners, 10:13 isolated-error ablations
m.exactRotorInertiaKnowledge=false; % diagnostic: same true plant, correct allocator I
m.planePitchCorrection=false; % candidate bounded CMG pitch correction after roll capture
end
