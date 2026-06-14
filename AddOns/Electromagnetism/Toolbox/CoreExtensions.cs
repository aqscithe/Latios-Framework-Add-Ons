using Latios;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Latios.Anna.Electromagnetism
{
    public static class CoreExtensions
    {
        /// <summary>
        /// Returns the electromagnetism settings for the current scene. Falls
        /// back to the world blackboard, then to <c>default</c> (which a system
        /// should interpret as "no EM authored in this scene — skip update").
        ///
        /// Mirrors <c>Latios.Anna.CoreExtensions.GetPhysicsSettings</c>.
        /// </summary>
        public static ElectromagnetismSettings GetElectromagnetismSettings(this LatiosWorldUnmanaged latiosWorld)
        {
            if (latiosWorld.sceneBlackboardEntity.HasComponent<ElectromagnetismSettings>())
                return latiosWorld.sceneBlackboardEntity.GetComponentData<ElectromagnetismSettings>();
            if (latiosWorld.worldBlackboardEntity.HasComponent<ElectromagnetismSettings>())
                return latiosWorld.worldBlackboardEntity.GetComponentData<ElectromagnetismSettings>();
            return default;
        }

        /// <summary>
        /// True iff <see cref="ElectromagnetismSettings"/> has been authored
        /// somewhere reachable from a blackboard.
        /// </summary>
        public static bool HasElectromagnetismSettings(this LatiosWorldUnmanaged latiosWorld)
        {
            return latiosWorld.sceneBlackboardEntity.HasComponent<ElectromagnetismSettings>()
                || latiosWorld.worldBlackboardEntity.HasComponent<ElectromagnetismSettings>();
        }
    }
}
