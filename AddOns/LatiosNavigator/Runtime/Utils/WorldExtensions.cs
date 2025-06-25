using Latios.Navigator.Components;

namespace Latios.Navigator.Utils
{
    public static class WorldExtensions
    {
        public static NavMeshSurfaceBlobReference GetNavMeshSurfaceBlob(this LatiosWorldUnmanaged world)
        {
            if (world.sceneBlackboardEntity.HasComponent<NavMeshSurfaceBlobReference>())
                return world.sceneBlackboardEntity.GetComponentData<NavMeshSurfaceBlobReference>();

            if (world.worldBlackboardEntity.HasComponent<NavMeshSurfaceBlobReference>())
                return world.worldBlackboardEntity.GetComponentData<NavMeshSurfaceBlobReference>();

            return default;
        }
    }
}