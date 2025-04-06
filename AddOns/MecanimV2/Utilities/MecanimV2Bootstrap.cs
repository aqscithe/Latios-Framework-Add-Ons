using Unity.Entities;

namespace Latios.Mecanim
{
    public static class MecanimBootstrap
    {
        /// <summary>
        /// Installs the Mecanim v2 state machine runtime systems. This should only be installed in the runtime world.
        /// </summary>
        /// <param name="world"></param>
        public static void InstallMecanimAddon(World world)
        {
            BootstrapTools.InjectSystem(TypeManager.GetSystemTypeIndex<UpdateMecanimSystem>(), world);
        }
    }
}

