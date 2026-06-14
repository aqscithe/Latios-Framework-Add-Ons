# Electromagnetism — Anna Addon

A quasi-static electromagnetic field simulation built on top of Anna Physics. Magnets and ferromagnetic objects exchange force/torque through a shared voxel field grid. Magnetic forces combine with gravity and contact forces via Anna's solver — no special integration required.

## Scope

Analytical dipole sources writing into a dense cell-centered B grid, with ferromagnetic receivers sampling the grid and applying force `F = ∇(m·B)` and torque `τ = m × B` via Anna's `AddImpulse` buffer.

| Component | Status |
|---|---|
| Field grid | Dense vec3 B at cell centers, fixed scene-anchored bounds |
| Sources | `PermanentMagnet` (constant dipole), `Electromagnet` (m = N·I·A·n̂, runtime-controllable current), `WireSegment` (closed-form Biot-Savart, runtime-controllable current) |
| Receivers | `Ferromagnet` (induced dipole, gets attracted) |
| Force | F=∇(m·B), τ=m×B (with self-contribution subtracted at sample time) |
| Anna integration | `AddImpulse` (field-flavor linear + axial-flavor angular) |
| Bake / induction / GPU viz | Deferred |

Full architecture and tiered scope is documented in `electromagnetism_implementation_plan.md` at the project root.

## Getting Started

**Scripting Defines:** `LATIOS_ADDON_ANNA`, `LATIOS_ADDON_ANNA_ELECTROMAGNETISM`

**Requirements:**

- Requires Latios Framework 0.15.0 or newer
- Requires Latios.Anna addon installed and bootstrapped first
- Requires QVVS Transforms

### Installing

In your `LatiosBootstrap`, install electromagnetism after Anna:

```csharp
var anna = Latios.Anna.AnnaBootstrap.InstallAnna(world);
Latios.Anna.Electromagnetism.ElectromagnetismBootstrap.InstallElectromagnetism(world);
```

### Authoring a scene

1. Add an `ElectromagnetismSettingsAuthoring` to any GameObject in the scene's subscene. This defines the grid's bounds, cell size, and global tuning. Without it, no grid is allocated and no EM systems run.
2. Add one of the source authoring components to any GameObject:
   - `PermanentMagnetAuthoring` — constant dipole. Set magnitude and local-axis direction.
   - `ElectromagnetAuthoring` — coil-driven dipole. Set turns × area for coil construction, initial current, and the coil-normal direction. Gameplay code controls current at runtime via `SystemAPI.GetComponentRW<Electromagnet>(entity).ValueRW.currentAmps = …` (or `EntityManager.SetComponentData`).
   - `WireSegmentAuthoring` — straight current-carrying wire. Set both endpoints in body-local space and the initial current; gameplay code dials current at runtime by writing `WireSegment.currentAmps`. Wires don't act as receivers in Tier 2 (no F = I·L×B), so they're emit-only.

   Pair a dipole source (permanent magnet / electromagnet) with `AnnaRigidBodyAuthoring` if you want it to also be a *receiver* (handheld magnets, magnetic crane arms — anything that should feel reaction force). Omit `AnnaRigidBodyAuthoring` for *static* sources (wall-mounted holding magnets, magnetic door coils, conveyor pads) — they still emit field normally, they just don't feel reaction force. Wires default to static.
3. Add `FerromagnetAuthoring` to any GameObject with `AnnaRigidBodyAuthoring`. Set `μ_r` (relative permeability — iron ~200) and the body's approximate volume.

### Acceptance tests

- A `PermanentMagnetAuthoring` item near a `FerromagnetAuthoring` item: the ferromagnet should accelerate toward the magnet and orient its long axis to align with the local field. Two permanent magnets should attract / repel based on relative orientation and rotate to align.
- An `ElectromagnetAuthoring` item near a `FerromagnetAuthoring` item with positive initial current: behaves like a permanent magnet. Toggle the current sign at runtime and the pole flips; set to zero and the field collapses.

## Math notes

- Reference frame: world. The grid is anchored to the scene; movement of bodies through the grid does the source/receiver work.
- Units: B in Tesla, m in A·m², F in Newtons, τ in N·m. Impulse `= force · dt`, angular impulse `= torque · dt` — Anna's `AddImpulse` expects impulses, not forces, so the conversion happens just before append.
- μ₀ = 4π × 10⁻⁷ T·m/A. Dipole field formula: `B(r) = (μ₀/4π) · [(3·(m·r̂)·r̂ − m) / r³]`.
- Force on a dipole: `F = ∇(m·B)`. Computed via central differences on the grid.
- Torque on a dipole: `τ = m × B(centerPos)`. Aligns the dipole with the local field.
- A `globalForceScale` setting allows tuning the magnitude for gameplay feel. Realistic permanent magnets have very small `|m|` (~0.1–10 A·m²) and short influence radii at game scale; the scale knob exists to make small magnets feel impactful without authoring artificially huge moments.

## Bigger picture

See `electromagnetism_simulation_spec.md` and `electromagnetism_implementation_plan.md` at the project root for the multi-tier roadmap (Tier 2: electromagnets, wires, GPU viz, sparse grid; Tier 3: offline bake, Faraday induction, eddy currents).

**Author:** aqscithe / The Facility

**Support:** Project-internal for now. If this matures into something general-purpose, the natural upstream is the Latios Framework addons repository.
