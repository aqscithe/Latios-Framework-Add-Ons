# Changelog

All notable changes to this addon are documented here.

## [Unreleased]

### Added

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
