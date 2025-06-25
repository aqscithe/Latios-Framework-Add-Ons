using Latios.Navigator.Components;
using Unity.Entities;
using NavMeshAgent = UnityEngine.AI.NavMeshAgent;

namespace Latios.Navigator.Authoring
{
    [DisableAutoCreation]
    public class NavMeshAgentBaker : Baker<NavMeshAgent>
    {
        public override void Bake(NavMeshAgent authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent<NavmeshAgentTag>(entity);
            AddComponent(entity, new Components.NavMeshAgent
            {
                Radius = authoring.radius
            });

            AddComponent(entity, new AgentDestination
            {
                Position = authoring.transform.position
            });

            AddBuffer<AgentPathEdge>(entity);
            AddBuffer<AgentPathPoint>(entity);
            AddComponent<AgentPath>(entity);
            AddComponent<AgenPathRequestedTag>(entity);
            SetComponentEnabled<AgenPathRequestedTag>(entity, false);
            AddComponent<AgentHasEdgePathTag>(entity);
            SetComponentEnabled<AgentHasEdgePathTag>(entity, false);
        }
    }
}