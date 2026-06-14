using Latios;
using Unity.Burst;
using Unity.Entities;

namespace Latios.Anna.Electromagnetism.Systems
{
    /// <summary>
    /// Mirrors the baked-from-authoring <see cref="ElectromagnetismSettings"/>
    /// singleton onto the <c>sceneBlackboardEntity</c> so the rest of the EM
    /// systems can find it via the cheap
    /// <see cref="CoreExtensions.GetElectromagnetismSettings"/> path.
    ///
    /// Runs in <see cref="InitializationSystemGroup"/> so the mirror lands
    /// before <see cref="ElectromagnetismSuperSystem"/> (which lives in
    /// <see cref="SimulationSystemGroup"/>) every frame.
    ///
    /// After a successful mirror the system disables itself — otherwise the
    /// next-frame <c>HasSingleton&lt;T&gt;</c> check would throw "found 2
    /// instances" because the value now exists both on the original subscene
    /// entity and on the blackboard. <see cref="OnNewScene"/> re-arms the
    /// system whenever Latios reports a scene transition, so single-scene and
    /// multi-scene workflows both work without manual intervention.
    ///
    /// Mirrors the project-internal <c>AddPhysicsSettingsToBlackboardSystem</c>
    /// pattern that Anna's <c>PhysicsSettings</c> relies on.
    /// </summary>
    [DisableAutoCreation]
    [UpdateInGroup(typeof(InitializationSystemGroup))]
    [BurstCompile]
    public partial struct AddElectromagnetismSettingsToBlackboardSystem : ISystem, ISystemNewScene
    {
        LatiosWorldUnmanaged latiosWorld;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            latiosWorld = state.GetLatiosWorldUnmanaged();
        }

        public void OnNewScene(ref SystemState state)
        {
            // Re-arm for the new scene's blackboard. Without this, the system
            // would stay disabled after the first scene and never mirror the
            // second scene's settings.
            state.Enabled = true;
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            // Singleton check must come BEFORE any sceneBlackboardEntity access.
            // On early frames before the subscene loads, sceneBlackboardEntity
            // may not be initialized yet; accessing it then throws (mirrors the
            // working AddPhysicsSettingsToBlackboardSystem pattern).
            if (!SystemAPI.HasSingleton<ElectromagnetismSettings>())
                return;

            var settings = SystemAPI.GetSingleton<ElectromagnetismSettings>();
            latiosWorld.sceneBlackboardEntity.AddComponent<ElectromagnetismSettings>();
            latiosWorld.sceneBlackboardEntity.SetComponentData(settings);

            // Disable so the next frame's HasSingleton<T> doesn't throw because
            // there are now two instances of ElectromagnetismSettings — the
            // original on the subscene entity, and the mirror on the blackboard.
            state.Enabled = false;
        }
    }
}
