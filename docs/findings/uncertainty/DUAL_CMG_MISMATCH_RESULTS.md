# Dual-CMG mismatch sensitivity

The accepted 1200 rpm dual-CMG baseline was evaluated over 49 combinations
of relative flywheel-speed mismatch and CMG #2 rate-servo time-constant
mismatch. Both quantities ranged from -20% to +20%. Mean flywheel speed was
held at 1200 rpm, and the allocator used the actual rotor speeds.

All 49 cases completed the 90-degree maneuver without rate or acceleration
limiting. Worst-case settling time was 2.885 s and worst final error magnitude
was 0.774 degrees. Maximum condition number was 12.20, closest singularity
distance was 9.56 degrees, and minimum active pitch-neutral roll-capacity
margin was 6.99 times the requested moment. No tested case became singular or
directionally infeasible.

Flywheel-speed mismatch dominates cross-axis sensitivity. With matched servo
dynamics, 5%, 10%, and 20% speed mismatch produced peak pitch responses of
0.166, 0.333, and 0.664 degrees and peak yaw responses of 0.083, 0.166, and
0.334 degrees, respectively. The response is approximately symmetric with
respect to which rotor is faster.

Servo mismatch alone has little effect over the tested range. At +/-20% servo
time-constant mismatch with matched flywheel speeds, peak pitch remains below
0.002 degrees and peak yaw below 0.008 degrees. The combined +20% speed and
+20% servo case gives the largest observed pitch/yaw response: 0.667 and
0.345 degrees.

The symmetric cancellation is therefore robust enough to preserve the roll
maneuver across the tested envelope, but it is not exact when rotor momentum
magnitudes differ. Closed-loop flywheel-speed regulation or calibrated
momentum balancing should be retained in a hardware design if sub-degree
cross-axis motion is required. This sweep assumes the allocator knows the
actual speeds; sensor bias and unmodeled inertia error require a separate
robustness test.
