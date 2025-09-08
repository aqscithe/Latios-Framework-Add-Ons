using Latios.Psyshock;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Latios.Anna.Authoring
{
    public class AnnaRigidBodyAuthoring : MonoBehaviour
    {
        [Header("Simulation properties")]
        public float mass                     = 1f;
        public float coefficientOfFriction    = 0.3f;
        public float coefficientOfRestitution = 0.3f;

        [Header("Constraints")]
        public bool3 lockPositionAxes    = false;
        public bool3 lockRotationAxes    = false;
        public bool  forceLockAxesExists = false;

        [Header("Initial State")]
        public float3 initialVelocity = float3.zero;

        [Header("Features")]
        public bool supportExternalForces = true;

        [Tooltip("Enables the custom gravity properties below")]
        public bool supportCustomGravity  = true;

        [Header("Custom Gravity Properties")]

        [Tooltip("Override Anna physics settings gravity")]
        public bool   useGravityOverride = false;

        [Tooltip("Scales Anna physics settings gravity | Scales override instead if enabled")]
        public bool   useGravityScaling  = false;

        public float3 gravityOverride    = new float3(0f, -9.81f, 0f);
        public float  gravityScale       = 1f;


    }

    public class AnnaRigidBodyAuthoringBaker : Baker<AnnaRigidBodyAuthoring>
    {
        public override void Bake(AnnaRigidBodyAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(entity, new RigidBody
            {
                inverseMass              = 1f / authoring.mass,
                coefficientOfFriction    = (half)authoring.coefficientOfFriction,
                coefficientOfRestitution = (half)authoring.coefficientOfRestitution,
                velocity                 = new UnitySim.Velocity
                {
                    linear  = authoring.initialVelocity,
                    angular = float3.zero
                }
            });
            AddComponent<CollisionWorldAabb>( entity);
            AddComponent<CollisionWorldIndex>(entity);
            if (math.any(authoring.lockPositionAxes | authoring.lockRotationAxes) || authoring.forceLockAxesExists)
            {
                AddComponent(entity, new LockWorldAxesFlags
                {
                    packedFlags = (byte)(math.bitmask(new bool4(authoring.lockPositionAxes, false)) | (math.bitmask(new bool4(authoring.lockRotationAxes, false)) << 3))
                });
            }
            if (authoring.supportExternalForces)
            {
                AddBuffer<AddImpulse>(entity);
            }
            if (authoring.supportCustomGravity)
            {
                AddComponent(entity, new CustomGravity {
                    useGravityScaling = authoring.useGravityScaling,
                    useGravityOverride = authoring.useGravityOverride,
                    gravityScale = authoring.gravityScale,
                    gravityOverride = authoring.gravityOverride
                });
            }
        }
    }

}

