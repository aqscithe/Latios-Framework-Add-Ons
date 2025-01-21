using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Latios.MecanimV2
{
    public static class StateMachineEvaluation
    {
        // Todo: State machine updates are trickier than they seem at first, because with a large enough timestep,
        // we can transition through multiple states in a single update. Root motion needs to know all these passages
        // in order to work correctly, otherwise we can get acceleration jumps on transitions that don't make sense.
        // The big question here is whether we also need to broadcast state enter/exit/update events in-time, perhaps
        // in a way that allows for parameters to be modified. And how might such events interact with code-driven
        // state transitions, if at all?
    }
}

