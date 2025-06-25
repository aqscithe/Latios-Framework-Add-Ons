using Unity.Entities;
using Unity.Mathematics;

namespace Latios.Navigator.Components
{
    public struct AgentPathPoint : IBufferElementData
    {
        public float3 Position;
    }


    public struct AgentPath : IComponentData
    {
        public int PathLength;
        public int PathIndex;
    }

    public struct NavmeshAgentTag : IComponentData { }

    public struct NavMeshAgent : IComponentData, IEnableableComponent
    {
        public float Radius;
    }

    public struct AgenPathRequestedTag : IComponentData, IEnableableComponent { }


    public struct AgentDestination : IComponentData
    {
        public float3 Position;
    }
}