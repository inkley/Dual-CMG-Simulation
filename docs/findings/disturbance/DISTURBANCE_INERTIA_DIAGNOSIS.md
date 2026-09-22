# Unequal-inertia disturbance-tracking failure: diagnosis

## Conclusion

The selected failure is caused primarily by inertia-estimation error amplified
by a poorly conditioned, strongly cancelling two-CMG steering configuration.
Finite-servo lag and damped-allocation error also contribute. It is not a
recorded rate/acceleration saturation, exhausted aggregate momentum budget, or
integration-step artifact. No production controller or acceptance criterion was
changed in this investigation.

## Evidence

The same true +10%/-10% unequal-inertia plant receives a 0.006 Nm, 15-second
sinusoid and -0.0005 Nm constant bias over the existing 200-second test.
Its allocator initially assumes nominal inertias.

- Peak roll error: 2.039360 degrees at 179.16 s (criterion: 2 degrees).
- Recorded above-threshold interval: 179.08–179.27 s, approximately 0.20 s by
  sampled integration. No acceptance tolerance was added for its brevity.
- At the error peak the steering-column separation from singularity is 1.5613
  degrees and the true smallest singular value is 0.0024558.
- Requested roll moment is -0.010910 Nm; achieved moment is -0.030736 Nm.
- The achieved-minus-requested difference decomposes at that instant into
  +0.0007375 Nm damped-allocation residual, -0.018692 Nm inertia-estimation
  residual, and -0.0018716 Nm servo residual.
- At that instant commanded gimbal rates are -0.8299/-0.7534 rad/s. A static
  true-model solve for the requested roll/pitch moments gives -1.2177/-1.4033
  rad/s, inside the 20 rad/s bounds, with bounded residual 1.04e-16 Nm. The
  desired moment pair is statically feasible there; this does not establish
  instantaneous finite-servo reachability or feasibility throughout the run.

The trajectory passes even closer to a formal steering singularity at 147 s
(sampled distance 0.00412 degrees), without its largest roll error there.
Conditioning alone does not identify the failure: demand direction, opposing
rotor contributions, model error, and the evolving state matter together.
The torque snapshot at peak error describes the contemporaneous correction;
the plotted preceding history is needed to interpret the evolving error.

The decomposition evaluates the same commanded rates through the estimated and
true steering matrices. It closes algebraically to the achieved-minus-requested
moment. Its components are not independent energy terms or independent causal
experiments. The counterfactual below supplies additional causal evidence.

## Same-plant counterfactual

Changing **only allocator inertia knowledge** to the true unequal values gives:

| Metric | Nominal inertia knowledge | Exact inertia knowledge |
|---|---|---|
| Peak roll error | 2.03936 deg | 1.2412 deg |
| Load-window RMS | 0.81143 deg | 0.7923 deg |
| Final-second error | 0.12176 deg | 0.0955 deg |
| Complete 200-second horizon | Yes | Yes |
| Recorded actuator-limit flags | 0 | 0 |
| Original roll/momentum screen | Fail | Pass |

The true rotors, installed vehicle properties, load, initial state, gains,
servo dynamics, damping, pitch correction, and thresholds remain unchanged.
Peak rotor-x momentum excursion remains approximately 0.100 N m s.
Exact knowledge is a diagnostic idealization, not an implemented estimator or
evidence that all other mismatch cases would pass.

An earlier half-step repeat reproduced the original failure with a common-grid
roll change of only 1.06e-9 degrees. Numerical refinement does not remove it.

## Next development decision

The diagnosis is complete. Keep the original failure visible. A targeted next
step is to bound acceptable rotor-inertia estimation error around the same
unequal plant and test whether an uncertainty-aware allocation strategy is
needed near poorly conditioned steering. Do not increase the 2-degree criterion
or silently replace uncertain estimates with truth in production.

## Reproduction

```matlab
DIAGNOSE_DISTURBANCE_INERTIA_FAILURE
REPORT_DISTURBANCE_INERTIA_DIAGNOSIS
```

Saved tables, histories, exact-knowledge test, and PNG are under
`Working Results/roll_disturbance_envelope/unequal_inertia`.
