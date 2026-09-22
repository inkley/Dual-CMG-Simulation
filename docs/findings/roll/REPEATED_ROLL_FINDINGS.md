# Repeated roll and momentum accumulation

Two four-segment sequences were evaluated with six seconds per segment:

- Reciprocal: 90, 0, 90, 0 degrees.
- Cumulative: 90, 180, 270, 360 degrees.

The original +/-45-degree gimbal preload was directionally biased. It
supported repeated positive rolls but did not contain enough rotor-momentum
range for a full negative 90-degree return. The gimbals then approached their
angle limits. This was a momentum-envelope limitation, not evidence that every
repeated maneuver necessarily accumulates momentum.

The dual baseline now uses a +/-15-degree preload at +/-1200 rpm. This provides
adequate positive and negative roll-momentum range. Both repeated sequences
complete without gimbal-rate, gimbal-acceleration, flywheel, or mechanical
angle limiting.

After the reciprocal sequence, rotor Hx differs from its initial value by
-1.14e-3 N m s and each gimbal is within 0.26 degrees of its initial angle.
After four same-direction increments, rotor Hx has drifted -4.08e-3 N m s and
each gimbal has drifted 0.93 degrees. The latter is a small but systematic
accumulation caused by hydrodynamic roll drag: the surrounding fluid is an
external angular-momentum sink, so a sequence of rolls in only one direction
cannot return the internal momentum state exactly. Alternating roll direction
largely cancels that impulse.

The +/-15-degree geometry passes through column alignment during portions of
the maneuver. The full two-axis K/M steering matrix is rank deficient at that
instant, but its remaining controllable direction is roll and the requested
pitch moment is zero. Consequently, the pitch-neutral roll-capacity margin
remains greater than ten times the active command and no roll-rate
infeasibility occurs. This is acceptable for the present roll-only CMG role,
but it would not support simultaneous arbitrary CMG pitch control.

The relevant publication distinction is between a formal full-matrix
singularity and a task-direction singularity. Reciprocal rest-to-rest roll is
nearly momentum-neutral, while repeated same-direction roll slowly walks the
momentum state because of external drag. Long-duration operation therefore
requires monitoring and, eventually, momentum unloading. Sustained external
roll disturbance should be examined with the later wave-disturbance cases.
