using Latios.Anna.Authoring;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Latios.Anna.Electromagnetism.Authoring
{
    /// <summary>
    /// A straight current-carrying wire segment. Field is computed via
    /// closed-form Biot-Savart on grid cells within
    /// <see cref="influenceRadius"/> of the segment.
    ///
    /// Wires are typically static — they're conduits running along walls,
    /// embedded in floors, threaded through a piece of machinery. Authoring
    /// does *not* require <see cref="AnnaRigidBodyAuthoring"/>; if you want
    /// the wire to move with a rigid body (handheld railgun, magnetic crane
    /// arm) add it explicitly. The wire is emit-only either way — wires don't
    /// feel reaction force in Tier 2 (we don't model F = I·L×B yet, spec §5.3).
    ///
    /// Gameplay controls the current at runtime via
    /// <c>SystemAPI.GetComponentRW&lt;WireSegment&gt;(entity).ValueRW.currentAmps = …</c>.
    /// Zero current → no field (the source-write pass early-outs).
    /// </summary>
    [AddComponentMenu("Latios/Anna/Wire Segment")]
    public class WireSegmentAuthoring : MonoBehaviour
    {
        [Header("Geometry (body-local)")]
        [Tooltip("Start endpoint in body-local space. The GameObject's transform " +
                 "is treated as the body origin; this offset is rotated/translated " +
                 "to world each substep.")]
        public Vector3 startLocal = new Vector3(0f, 0f, -0.5f);

        [Tooltip("End endpoint in body-local space. Current flows start → end " +
                 "for positive currentAmps (right-hand rule for B).")]
        public Vector3 endLocal = new Vector3(0f, 0f, 0.5f);

        [Header("Drive")]
        [Tooltip("Initial current in amperes. Gameplay code can rewrite at " +
                 "runtime via SetComponentData<WireSegment>. Sign determines " +
                 "the direction of the B field via the right-hand rule.")]
        public float initialCurrentAmps = 10f;

        [Header("Influence")]
        [Tooltip("World-space radius beyond which this wire's contribution to " +
                 "the field grid is skipped. Per-cell cost scales as " +
                 "(segmentLength + 2·radius)·radius². A wire's field falls as " +
                 "1/d (slower than a dipole's 1/d³), so meaningful influence " +
                 "extends further than for a magnet of comparable strength.")]
        [Min(0.05f)]
        public float influenceRadius = 1.0f;

        void OnDrawGizmosSelected()
        {
            Vector3 startWorld = transform.TransformPoint(startLocal);
            Vector3 endWorld   = transform.TransformPoint(endLocal);

            // Current direction arrow.
            float poleSign = Mathf.Sign(initialCurrentAmps);
            if (poleSign == 0f) poleSign = 1f;

            Vector3 fromPt = poleSign > 0 ? startWorld : endWorld;
            Vector3 toPt   = poleSign > 0 ? endWorld   : startWorld;

            Gizmos.color = new Color(1f, 0.85f, 0.2f, 0.9f);
            Gizmos.DrawLine(startWorld, endWorld);
            Gizmos.DrawSphere(fromPt, 0.04f);

            // Arrowhead on the current-positive end.
            Vector3 dir = (toPt - fromPt);
            float   len = dir.magnitude;
            if (len > 1e-3f)
            {
                Vector3 unit  = dir / len;
                Vector3 ortho = Vector3.Cross(unit, Vector3.up);
                if (ortho.sqrMagnitude < 1e-6f)
                    ortho = Vector3.Cross(unit, Vector3.right);
                ortho.Normalize();
                float head = Mathf.Min(0.15f, len * 0.25f);
                Gizmos.DrawLine(toPt, toPt - unit * head + ortho * head * 0.5f);
                Gizmos.DrawLine(toPt, toPt - unit * head - ortho * head * 0.5f);
            }

            // Influence "tube" — two endpoint spheres bracket a capsule's
            // ends well enough for designer intuition without a real capsule
            // gizmo (Unity has no built-in DrawWireCapsule).
            Gizmos.color = new Color(0.4f, 0.7f, 1f, 0.25f);
            Gizmos.DrawWireSphere(startWorld, influenceRadius);
            Gizmos.DrawWireSphere(endWorld,   influenceRadius);
        }
    }

    public class WireSegmentBaker : Baker<WireSegmentAuthoring>
    {
        public override void Bake(WireSegmentAuthoring authoring)
        {
            // Renderable is enough for a static wire (just need WorldTransform).
            // If the user also added AnnaRigidBodyAuthoring, that baker upgrades
            // the flags to Dynamic — no conflict.
            var entity = GetEntity(TransformUsageFlags.Renderable);

            AddComponent(entity, new WireSegment
            {
                startLocal   = new float3(authoring.startLocal.x,   authoring.startLocal.y,   authoring.startLocal.z),
                endLocal     = new float3(authoring.endLocal.x,     authoring.endLocal.y,     authoring.endLocal.z),
                currentAmps  = authoring.initialCurrentAmps,
            });
            AddComponent(entity, new InfluenceRadius
            {
                radius = authoring.influenceRadius,
            });
        }
    }
}
