using Latios.Transforms.Abstract;
using Unity.Collections;
using Unity.Entities;

namespace Latios.FlowFieldNavigation
{
    public struct FlowGoalTypeHandles
    {
        [ReadOnly] internal WorldTransformReadOnlyAspect.TypeHandle WorldTransform;
        [ReadOnly] internal ComponentTypeHandle<FlowField.Goal> GoalTypeHandle;
        [ReadOnly] internal EntityTypeHandle Entity;

        /// <summary>
        /// Constructs the FlowGoal type handles using a managed system
        /// </summary>
        public FlowGoalTypeHandles(SystemBase system)
        {
            GoalTypeHandle = system.GetComponentTypeHandle<FlowField.Goal>(true);
            WorldTransform = new WorldTransformReadOnlyAspect.TypeHandle(ref system.CheckedStateRef);
            Entity = system.GetEntityTypeHandle();
        }

        /// <summary>
        /// Constructs the FlowGoal type handles using a SystemState
        /// </summary>
        public FlowGoalTypeHandles(ref SystemState system)
        {
            GoalTypeHandle = system.GetComponentTypeHandle<FlowField.Goal>(true);
            WorldTransform = new WorldTransformReadOnlyAspect.TypeHandle(ref system);
            Entity = system.GetEntityTypeHandle();
        }

        /// <summary>
        /// Updates the type handles using a managed system
        /// </summary>
        public void Update(SystemBase system)
        {
            GoalTypeHandle.Update(system);
            WorldTransform.Update(ref system.CheckedStateRef);
            Entity.Update(system);
        }

        /// <summary>
        /// Updates the type handles using a SystemState
        /// </summary>
        public void Update(ref SystemState system)
        {
            GoalTypeHandle.Update(ref system);
            WorldTransform.Update(ref system);
            Entity.Update(ref system);
        }
    }
}