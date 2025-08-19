using Unity.Entities;
using UnityEngine;

namespace Latios.FlowFieldNavigation.Hybrid
{
    public class FlowFieldAgentAuthoring : MonoBehaviour
    {
        [Range(1, FlowSettings.MaxFootprintSize)]
        public int FootprintSize = 3;

        class Baker : Baker<FlowFieldAgentAuthoring>
        {
            public override void Bake(FlowFieldAgentAuthoring authoring)
            {
                var entity = GetEntity(authoring, TransformUsageFlags.Dynamic);
                AddComponent<FlowField.AgentDirection>(entity);
                AddComponent<FlowField.PrevPosition>(entity);
                AddComponent<FlowField.Velocity>(entity);
                AddComponent(entity, new FlowField.AgentFootprint { Size = authoring.FootprintSize });
            }
        }
    }
}