using Unity.Entities;
using Unity.Mathematics;

namespace Latios.Navigator.Components
{
    internal struct AgentHasEdgePathTag : IComponentData, IEnableableComponent { }

    internal struct AgentPathEdge : IBufferElementData
    {
        public float3 PortalVertex1;
        public float3 PortalVertex2;
    }

    [TemporaryBakingType]
    internal struct TriangleElementInput : IBufferElementData
    {
        public int    Ia;
        public int    Ib;
        public int    Ic;
        public float3 PointA;
        public float3 PointB;
        public float3 PointC;
    }
}