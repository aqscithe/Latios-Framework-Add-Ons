using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Latios.Anna.Electromagnetism.Authoring
{
    /// <summary>
    /// Configures the per-scene electromagnetism grid. Place on any GameObject
    /// in the subscene (the AnnaSettings GameObject is a natural spot). Without
    /// this authoring component, no EM systems run.
    ///
    /// Bakes <see cref="ElectromagnetismSettings"/> as a regular IComponentData
    /// on the resulting entity. <see cref="CoreExtensions.GetElectromagnetismSettings"/>
    /// will locate it on the scene or world blackboard at runtime.
    /// </summary>
    [AddComponentMenu("Latios/Anna/Electromagnetism Settings")]
    public class ElectromagnetismSettingsAuthoring : MonoBehaviour
    {
        [Header("Grid bounds (world space)")]
        [Tooltip("World position of the grid's (0,0,0) corner.")]
        public Vector3 gridOrigin = new Vector3(-8f, -4f, -8f);

        [Tooltip("Cell counts along x, y, z. Multiply by Cell Size for total " +
                 "world coverage. Tier 1 recommendation: keep below ~4 million " +
                 "cells (e.g. 160x80x160 at 0.1m = ~16x8x16 m).")]
        public Vector3Int gridResolution = new Vector3Int(160, 80, 160);

        [Header("Cell size (meters)")]
        [Tooltip("Spec default is 0.1m. Larger cells trade detail for memory + " +
                 "compute. Tier 2's chunked/hierarchical storage will let us " +
                 "keep 0.1m at scene scale.")]
        [Min(0.001f)]
        public float cellSize = 0.1f;

        [Header("Gameplay tuning")]
        [Tooltip("Multiplies all magnetic forces and torques at the final-write " +
                 "step. 1.0 = realistic. Realistic permanent magnets at game " +
                 "scale produce very weak forces (~μN at 1m), so this knob " +
                 "exists to make a 1 A·m² hand-magnet feel impactful without " +
                 "authoring physically-unrealistic dipole moments.")]
        [Min(0f)]
        public float globalForceScale = 1f;

        void OnDrawGizmosSelected()
        {
            // Visualize the grid bounds so the designer can see what region
            // of the world is "EM-aware" before play.
            Vector3 size   = new Vector3(gridResolution.x, gridResolution.y, gridResolution.z) * cellSize;
            Vector3 center = (Vector3)gridOrigin + size * 0.5f;
            Gizmos.color = new Color(0.4f, 0.7f, 1.0f, 0.85f);
            Gizmos.DrawWireCube(center, size);
        }
    }

    public class ElectromagnetismSettingsBaker : Baker<ElectromagnetismSettingsAuthoring>
    {
        public override void Bake(ElectromagnetismSettingsAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.None);
            AddComponent(entity, new ElectromagnetismSettings
            {
                gridOrigin       = (float3)(Vector3)authoring.gridOrigin,
                gridResolution   = new int3(authoring.gridResolution.x, authoring.gridResolution.y, authoring.gridResolution.z),
                cellSize         = authoring.cellSize,
                globalForceScale = authoring.globalForceScale,
            });
        }
    }
}
