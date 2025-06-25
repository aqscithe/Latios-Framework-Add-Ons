using Latios.Authoring;
using Latios.Navigator.Authoring;
using Latios.Navigator.Systems;
using Unity.Entities;

namespace Latios.Navigator
{
    public static class NavigatorBakingBootstrap
    {
        public static void InstallNavigatorBakers(ref CustomBakingBootstrapContext context)
        {
            context.filteredBakerTypes.Add(typeof(NavMeshBaker));
            context.filteredBakerTypes.Add(typeof(NavMeshAgentBaker));
        }
    }

    public static class NavigatorBootstrap
    {
        public static void InstallNavigator(World world)
        {
            BootstrapTools.InjectSystem(TypeManager.GetSystemTypeIndex<NavRootSystem>(), world);
        }
    }
}