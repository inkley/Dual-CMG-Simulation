# Complete roll-turn-surge simulation

## Scope

`RUN_ROLL_TURN_SURGE` now connects CMG roll capture, a shaped in-plane heading
turn, turn-complete qualification, and a speed-controlled forward cruise segment.
It is a separate mission runner using the existing saved symmetric dual VFR
baseline. It does not modify the baseline driver, overwrite baseline results,
or require changes to the production CMG allocator or hybrid attitude controller.

The modeled mission ends while cruising. COMPLETE means the 30-second cruise
observation interval has ended; the speed command remains 0.5 m/s. It does not
mean waypoint arrival, propulsion shutdown, station keeping, or zero speed.
No return to nominal roll is included.

## Sequence and safety logic

The supervisor is updated outside the ODE at 0.05 s intervals. Its memory is
passed explicitly rather than stored in persistent variables inside the RHS.
Commands are held during each integration interval (maximum ODE step 0.01 s).

1. ROLL: command the plane alignment with aft thrust disabled. Require roll
   error <=0.5 degree and body angular-rate norm <=0.5 degree/s continuously
   across qualifying samples for 0.5 s. An interrupted capture resets the dwell.
2. TURN: start the 24 s quintic in-plane heading reference upon roll capture.
   Continue the existing CMG and differential-thruster control. Aft thrust
   remains disabled. After the reference finishes, require full nose-vector
   heading error <=1 degree, fixed-plane error <=0.5 degree, and angular-rate
   norm <=0.5 degree/s for a one-second sampled dwell.
3. SURGE: ramp speed to 0.5 m/s over 10 s and observe 30 s of forward travel.
   Continue heading and plane control. Switch lateral regulation to a
   cross-track direction orthogonal to the target heading within the maneuver
   plane; holding the previous inertial lateral coordinate would oppose travel.
4. COMPLETE: end recording after the cruise segment, with the vehicle moving.

Roll and turn timeouts are 15 and 50 s. During surge, heading error >3 degrees,
plane error >2 degrees, or angular-rate norm >5 degrees/s triggers ABORT.
ABORT commands zero aft thrust, freezes the desired nose direction at the
observed heading, and records five seconds of coasting with attitude control.
Actual propulsion force decays through the actuator model; it is not removed
instantaneously. Qualification/inhibits are sampled checks, not proofs of
continuous-time safety. A terminal ODE event stops simulation at the nominal
gimbal-angle bound because hard-stop dynamics remain unvalidated.

## Surge control

`SURGE_SPEED_CONTROL` uses forward-only PI control with conditional anti-windup:

`Fraw = meff * uref_dot + 1.62 * uref * abs(uref) + Kp*(uref-u) + Ki*integral`.

`Fcmd = clamp(Fraw,0,10 N)`, with `Kp=12`, `Ki=2`, and
`meff = vehicle mass + 0.93 kg` matching the implemented surge added mass.
Feedforward coefficients match the current REMUS model, not experimental
calibration. Integral action freezes when integration would drive command
saturation further, and resets while propulsion is inhibited. The existing
0.5 s thrust lag and 10 N/s rate bound remain in force. Reverse thrust and
active propulsion braking are not available.

## Finite-mission screening criteria

These are development screening criteria, not preregistered validation:
COMPLETE reached; heading and plane errors <=1 degree during surge; cross-track
and out-of-plane displacement each <=0.05 m; final speed within 0.025 m/s of
command; progress >=5 m; zero aft force before surge; no configured actuator
limit flags; and sampled rate-bounded roll allocation feasible. The 5 cm path
tolerance is a new cruise-segment criterion, not a relaxation/relabeling of
the earlier stationary maneuver's 15 mm criterion.

The independent allocation check finds a pitch-neutral exact gimbal-rate
solution including body yaw-rate bias. It proves sampled instantaneous
feasibility where a solution is found, not arbitrary momentum reachability,
long-term controllability, or finite-servo torque tracking by itself.

## Results

All three finite nominal missions passed the screening criteria.

| Quantity | +45 plane / +45 turn | -45 plane / -45 turn | 0 plane / +30 turn |
| --- | ---: | ---: | ---: |
| TURN begins (s) | 5.25 | 5.25 | 0.50 |
| SURGE begins (s) | 38.10 | 37.75 | 25.85 |
| Cruise segment ends (s) | 68.10 | 67.75 | 55.85 |
| Final full heading error (deg) | 0.422 | 0.483 | 0.020 |
| Maximum surge plane error (deg) | 0.585 | 0.670 | 0.022 |
| Final speed (m/s) | 0.4996 | 0.4996 | 0.4996 |
| Along-path progress (m) | 12.501 | 12.501 | 12.501 |
| Maximum in-plane cross-track error (mm) | 0.285 | 0.335 | 0.513 |
| Maximum out-of-plane displacement (mm) | approximately 15.0 | approximately 23.1 | approximately 2.5 |
| Peak aft force (N) | 3.522 | 3.522 | 3.522 |
| Maximum absolute gimbal angle (deg) | 62.10 | 63.61 | 45.10 |

Peak speed is about 0.5314 m/s (approximately 6.3% overshoot), so this is not
perfect trajectory tracking. Heading/plane transients during the initial turn
remain larger than the cruise errors. No aft thrust occurs during ROLL or TURN;
no reported actuator-limit or sampled roll-allocation failure occurs in these
three missions. Small cross-track numbers arise from this deterministic,
undisturbed model and do not establish real-world navigation precision.

## Verification and reproduction

```matlab
VERIFY_ROLL_TURN_SURGE
RUN_ROLL_TURN_SURGE
RUN_ROLL_TURN_SURGE(struct('planeDeg',-45,'headingDeg',-45))
RUN_ROLL_TURN_SURGE(struct('planeDeg',0,'headingDeg',30))
RUN_ROLL_TURN_SURGE(struct('sampleTime',.025,'maxStep',.005))
RUN_ROLL_TURN_SURGE(struct('rollTimeout',.1))
VERIFY_ROLL_TURN_SURGE_RESULTS
```

The unit tests cover dwell reset, stage ordering, reference-completion gating,
turn qualification, surge alignment inhibition, roll timeout, terminal cruise
semantics, and upper/lower command anti-windup. The integrated short-timeout
run never enters SURGE and keeps actual/commanded aft force zero. A forced
misalignment tests the supervisor's inhibit as a unit test; an integrated
disturbance-recovery study is not included.

The refined positive mission halves both supervisor and maximum ODE steps.
It retains the same transition times, changes final heading error by less
than 0.001 degree and final speed by less than 0.00001 m/s. Peak aft force
changes from 3.522 to 3.512 N. This is a numerical sensitivity check, not a
general stability proof.

Outputs reside under `Working Results/roll_turn_surge/` with a folder per
plane, heading, sample time and roll timeout. Each contains `mission.mat`,
`events.csv`, and `ROLL_TURN_SURGE.png`. The MAT stores the baseline snapshot,
mission settings and sampled state/control histories. Rerunning the same
output folder overwrites that scenario; archive results before changing other
settings. Bounds are provisional, and the model still omits propeller
shaft-reaction torque/inflow physics and uses simplified hydrodynamics.

## Trello update

Completed: implement and demonstrate finite roll-turn-surge sequencing with
bounded surge-speed control and sustained turn-complete qualification.

Keep open: broader uncertainty/disturbance testing, physical gimbal-stop
behavior, faster-turn controller coordination, propeller roll-reaction
assumptions, and waypoint arrival/stopping if retained in the paper's scope.
