# Changelog

All notable changes to this addon are documented here.

## [Unreleased]

### Added

- Gameplay hand-off: `ElectromagneticBypassTag` IComponentData — when present on an EM source entity (`PermanentMagnet`, `Electromagnet`, or `WireSegment`), both `WriteSourceContributionsSystem` AND `ComputeReceiverForcesSystem` skip the entity for the substep. The source neither writes its field into the grid nor samples force from the field. Intended for gameplay systems that temporarily own a source's behaviour (e.g. a grav-gun-carried magnet driving its own gameplay-curve force directly while not itself being yanked by other world fields). Adding/removing the tag is a clean hand-off — the source rejoins the simulation next substep with no teardown.
- Session 4 (gameplay tuning): `ElectromagnetismSettings.maxLinearAcceleration` — per-body ceiling (m/s²) on the linear acceleration the magnetic force can produce in a single substep, applied inside the receiver jobs after `globalForceScale`. 0 = disabled (no clamp, no change to existing scenes). Regularizes the `1/r⁴` gradient singularity at near-contact distances so a touching magnet/ferromagnet pair sticks instead of flying apart via Anna's collision-flip + impulse overshoot. Uses `RigidBody.inverseMass` so heavy and light bodies see the same acceleration ceiling. Skipped for infinite-mass bodies.
- Session 4 (gameplay tuning): `ElectromagnetismSettings.globalTorqueScale` — multiplier on `τ = m × B` at impulse-write, decoupled from `globalForceScale`. Lets a designer dial down rotational response (magnets whipping into uncontrolled spin) without losing translational pull. Default 1.0 = previous combined-scale behaviour. New slider on `ElectromagnetismSettingsAuthoring` under "Gameplay tuning".
- Tier 2 Phase C: `PropagatePermeabilitySystem` — μ_r-weighted Jacobi diffusion of the B field, runs between source-write and receiver-forces. Each iteration neighbour-averages B with weights = `sqrt(μ_r[c] · μ_r[n])` so high-μ_r geometry (iron / steel walls authored via the substance catalog) channels flux through itself. New `ElectromagneticField.Btemp` scratch buffer for ping-pong; `ElectromagnetismSettings.propagationIterations` (default 0 = disabled) and `propagationBlend` (default 0.7) knobs on the settings authoring. Skipped entirely at iterations=0 so existing scenes pay zero cost until the designer opts in.
- Tier 2: `WireSegment` source component — straight current-carrying wire with body-local endpoints. Field contribution computed via closed-form Biot-Savart (no numerical integration) on every grid cell within `InfluenceRadius.radius` of the segment. New `WireSegmentAuthoring` MonoBehaviour, static-by-default (no `AnnaRigidBodyAuthoring` required). Gameplay controls current via `SetComponentData<WireSegment>` — sign flips the field direction. Wires are emit-only in Tier 2 (no Lorentz F = I·L×B on the wire itself, spec §5.3).
- `EMMath.WireSegmentField` — closed-form Biot-Savart helper for an arbitrary straight segment.
- Tier 2: `Electromagnet` source component — coil-driven dipole with `m = N·I·A·n̂` derived each substep from runtime-writeable `currentAmps`. New `ElectromagnetAuthoring` MonoBehaviour for scene authoring. Gameplay code controls current via `SetComponentData<Electromagnet>` — sign of current flips the pole, zero collapses the field.

### Changed

- Lifted `influenceRadius` off `PermanentMagnet` into a shared `InfluenceRadius` component. Both `PermanentMagnetAuthoring` and `ElectromagnetAuthoring` add it; the receiver and self-subtraction paths now query the shared component, so adding a third dipole source type (wire segments, T2) only requires a new write job — the receiver pass picks it up for free.
- Renamed `PermanentReceiverJob` → `DipoleSourceReceiverJob` to reflect that it now handles any entity with a dipole moment + influence radius (permanent magnet, electromagnet, future).
- `WriteSourceContributionsSystem` now schedules a permanent-magnet job and an electromagnet job, chained on `state.Dependency` (both write the same `B` array). Shared per-cell dipole-accumulation loop extracted as a static helper.
- Dropped `[RequireComponent(typeof(AnnaRigidBodyAuthoring))]` from `PermanentMagnetAuthoring` and `ElectromagnetAuthoring`. Authoring a static (wall-mounted, embedded, etc.) source is now just authoring the source component without a rigid body — the field-emit path runs regardless; the receiver path naturally skips bodies without `RigidBody`+`AddImpulse`.

### Fixed

- Tier 1.5: subtract a source's own grid contribution from its B and ∇B samples before computing F/τ. Removes the spurious self-force a source-also-receiver felt from its own deposited field. Added `EMMath.SampleSelfDipoleContribution` and `EMMath.GradientSelfDipoleContribution` helpers.

### Added

- Tier 1 vertical slice: analytical permanent-magnet sources, ferromagnetic receivers, dense cell-centered B grid, force computation via `F = ∇(m·B)` and `τ = m × B`, Anna integration via `AddImpulse`.
- `ElectromagneticField` ICollectionComponent on `sceneBlackboardEntity` (persistent allocator, disposed on scene transition).
- `PermanentMagnet`, `Ferromagnet`, `MagneticDipoleMoment`, `EMEffectImmuneTag` components.
- `ElectromagnetismSettings` IComponentData with grid bounds, cell size, and `globalForceScale` tuning.
- `ElectromagnetismSuperSystem` orchestrating the per-substep update phases.
- `ElectromagnetismBootstrap.InstallElectromagnetism(world)` installer.
