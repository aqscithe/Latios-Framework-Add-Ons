using Latios.Anna.Authoring;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Latios.Anna.Electromagnetism.Authoring
{
    /// <summary>
    /// Marks a body as a permanent magnet.
    ///
    /// Pair with <see cref="AnnaRigidBodyAuthoring"/> for a *dynamic* magnet
    /// (handheld neodymium, magnetic ammo, anything that needs to feel
    /// reaction force from other sources). Omit it for a *static* magnet
    /// (a fixed bar magnet glued to a wall, a magnetic strip embedded in
    /// the floor) — the field-emit path doesn't need a rigid body, only the
    /// reaction-force path does. Static magnets still drive every other
    /// receiver in the scene normally.
    /// </summary>
    [AddComponentMenu("Latios/Anna/Permanent Magnet")]
    public class PermanentMagnetAuthoring : MonoBehaviour
    {
        [Header("Dipole moment (body-local)")]
        [Tooltip("Direction of the north-pole face in body-local space. Will be " +
                 "normalized at bake time; magnitude is read from Strength.")]
        public Vector3 localAxis = new Vector3(0f, 1f, 0f);

        [Tooltip("Dipole moment magnitude, units A·m². Realistic permanent " +
                 "magnets: a small ferrite ~0.1, a strong neodymium hand magnet " +
                 "~10. The Global Force Scale on ElectromagnetismSettings lets " +
                 "you dial up the impact without authoring unrealistic values.")]
        [Min(0f)]
        public float strength = 5f;

        [Header("Influence")]
        [Tooltip("World-space radius beyond which this magnet's contribution " +
                 "to the field grid is skipped. Cell-count cost per source " +
                 "scales as radius³, so keep this tight. A 1m radius at 0.1m " +
                 "cell size = ~4k cells per substep.")]
        [Min(0.05f)]
        public float influenceRadius = 1.5f;

        [Header("Thermal")]
        [Range(0f, 1f)]
        [Tooltip("Curie demagnetization scale: 1 = full strength, 0 = fully " +
                 "demagnetized (above Curie temperature). Tier 1 leaves this " +
                 "as a designer-authored value; Tier 2 drives it from thermal " +
                 "state.")]
        public float curieScale = 1f;

        void OnDrawGizmosSelected()
        {
            // Visualize the dipole axis (north → red arrow tip; south → blue).
            Vector3 worldAxis = transform.TransformDirection(localAxis.normalized);
            Vector3 origin    = transform.position;
            float   length    = Mathf.Max(0.1f, Mathf.Pow(strength, 1f / 3f) * 0.2f);

            Gizmos.color = Color.red;
            Gizmos.DrawLine(origin, origin + worldAxis * length);
            Gizmos.DrawWireSphere(origin + worldAxis * length, length * 0.1f);

            Gizmos.color = Color.blue;
            Gizmos.DrawLine(origin, origin - worldAxis * length);

            Gizmos.color = new Color(1f, 0.5f, 0f, 0.3f);
            Gizmos.DrawWireSphere(origin, influenceRadius);
        }
    }

    public class PermanentMagnetBaker : Baker<PermanentMagnetAuthoring>
    {
        public override void Bake(PermanentMagnetAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);

            Vector3 axis = authoring.localAxis;
            if (axis.sqrMagnitude < 1e-6f)
                axis = Vector3.up;
            float3 localMoment = math.normalize(new float3(axis.x, axis.y, axis.z)) * authoring.strength;

            AddComponent(entity, new PermanentMagnet
            {
                localMoment     = localMoment,
                curieScale      = (half)authoring.curieScale,
            });
            AddComponent(entity, new InfluenceRadius
            {
                radius = authoring.influenceRadius,
            });

            // Permanents are also receivers (they feel each other's fields),
            // and downstream debug viz expects every magnetic body to carry
            // MagneticDipoleMoment so the world moment is queryable.
            AddComponent(entity, new MagneticDipoleMoment
            {
                worldMoment = float3.zero,
            });
            // Telemetry channel: ComputeReceiverForcesSystem writes the
            // last-substep F and τ here so debug viz can draw arrows
            // without recomputing.
            AddComponent(entity, default(MagneticFeedback));
        }
    }
}
