using Unity.Entities;
using UnityEngine;

namespace Latios.FlowFieldNavigation.Hybrid
{
    public class FlowFieldGoalAuthoring : MonoBehaviour
    {
        class Baker : Baker<FlowFieldGoalAuthoring>
        {
            public override void Bake(FlowFieldGoalAuthoring authoring)
            {
                var entity = GetEntity(authoring, TransformUsageFlags.Dynamic);
                AddComponent<FlowField.Goal>(entity);
            }
        }
    }
}