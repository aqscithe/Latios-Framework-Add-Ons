using Unity.Entities;
using UnityEngine;

namespace Latios.FlowFieldNavigation.Hybrid
{
    public class FlowFieldGoalAuthoring : MonoBehaviour
    {
        [Range(1, FlowSettings.MaxFootprintSize)]
        public int FootprintSize = 3;
        
        class Baker : Baker<FlowFieldGoalAuthoring>
        {
            public override void Bake(FlowFieldGoalAuthoring authoring)
            {
                var entity = GetEntity(authoring, TransformUsageFlags.Dynamic);
                AddComponent(entity, new FlowField.Goal { FootprintSize = authoring.FootprintSize });
            }
        }
    }
}