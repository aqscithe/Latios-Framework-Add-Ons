# Changelog

All notable changes to this addon are documented here.

## [Unreleased]

### Fixed

- Tier 1.5: subtract a permanent magnet's own grid contribution from its B and ∇B samples before computing F/τ. Removes the spurious self-force a source-also-receiver felt from its own deposited field. Added `EMMath.SampleSelfDipoleContribution` and `EMMath.GradientSelfDipoleContribution` helpers and tightened `PermanentReceiverJob` to require `PermanentMagnet` (needed for the source's `influenceRadius`).

### Added

- Tier 1 vertical slice: analytical permanent-magnet sources, ferromagnetic receivers, dense cell-centered B grid, force computation via `F = ∇(m·B)` and `τ = m × B`, Anna integration via `AddImpulse`.
- `ElectromagneticField` ICollectionComponent on `sceneBlackboardEntity` (persistent allocator, disposed on scene transition).
- `PermanentMagnet`, `Ferromagnet`, `MagneticDipoleMoment`, `EMEffectImmuneTag` components.
- `ElectromagnetismSettings` IComponentData with grid bounds, cell size, and `globalForceScale` tuning.
- `ElectromagnetismSuperSystem` orchestrating the per-substep update phases.
- `ElectromagnetismBootstrap.InstallElectromagnetism(world)` installer.
