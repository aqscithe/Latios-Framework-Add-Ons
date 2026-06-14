using Latios;
using Unity.Entities;

namespace Latios.Anna.Electromagnetism
{
    public static class ElectromagnetismBootstrap
    {
        /// <summary>
        /// Installs the electromagnetism addon. Call AFTER
        /// <see cref="Latios.Anna.AnnaBootstrap.InstallAnna(World)"/> in your
        /// <c>LatiosBootstrap</c>.
        ///
        /// The returned <see cref="Systems.ElectromagnetismSuperSystem"/> can
        /// be used the same way Anna's super-system is used — for instance,
        /// to attach an <c>IRateManager</c> if you want EM to update at a
        /// different cadence than the rest of the simulation. By default it
        /// inherits Anna's rate via being scheduled <c>UpdateBefore(AnnaSuperSystem)</c>
        /// in the same SimulationSystemGroup.
        /// </summary>
        public static Systems.ElectromagnetismSuperSystem InstallElectromagnetism(World world)
        {
            // InitializationSystemGroup runs before SimulationSystemGroup each
            // frame, so installing this first guarantees the settings have
            // been mirrored to the scene blackboard by the time
            // ElectromagnetismSuperSystem updates.
            BootstrapTools.InjectSystem(
                TypeManager.GetSystemTypeIndex<Systems.AddElectromagnetismSettingsToBlackboardSystem>(),
                world);

            var result = BootstrapTools.InjectSystem(
                TypeManager.GetSystemTypeIndex<Systems.ElectromagnetismSuperSystem>(),
                world);
            return result.systemManaged as Systems.ElectromagnetismSuperSystem;
        }
    }
}
