# Mecanim V2 Feature Comparison

Mecanim V2 provides a different offering of features compared to Mecanim V1. But
like V1, it also does not try to perfectly replicate Game Objects in all
scenarios. This document uses tables to highlight the differences.

## Outputs

|                      | Game Objects              | V1                  | V2                |
|----------------------|---------------------------|---------------------|-------------------|
| QVVS Transforms      | Not Applicable            | Supported           | Supported         |
| Unity Transforms     | Not Applicable            | Supported           | Supported         |
| Exposed Skeletons    | Supported                 | Supported           | Not Supported     |
| Optimized Skeletons  | Supported                 | Supported           | Supported         |
| Blend Shapes         | Supported                 | Experimental        | Not Supported     |
| Arbitrary Properties | Supported                 | Not Supported       | Not Supported     |
| Root Motion          | Supported                 | Partially Supported | Supported         |
| IK Passes            | Supported                 | Not Supported       | Not Supported     |
| Update Times         | Normal, Fixed, and Manual | Normal Only         | Normal and Manual |
| Animation Culling    | Supported                 | Not Supported       | Not Supported     |

## State Machines

|                               | Game Objects                                             | V1                                                                                    | V2                                                                        |
|-------------------------------|----------------------------------------------------------|---------------------------------------------------------------------------------------|---------------------------------------------------------------------------|
| State Timings and Progression | Normalized, Dynamic                                      | Realtime, Baked, Does not apply state speed and multiplier parameter to time in state | Normalized, Dynamic                                                       |
| State Features                | Everything                                               | Mirror, Foot IK, and Write Defaults Ignored                                           | Mirror, Foot IK, and Write Defaults Ignored                               |
| Transition Features           | Everything                                               | Everything                                                                            | Everything                                                                |
| Max Transitions Per Update    | Multiple                                                 | 1                                                                                     | Multiple                                                                  |
| Transition Order              | Any First                                                | Any Last                                                                              | Any First                                                                 |
| Exit Times                    | Per loop if \<= 1f, once otherwise                       | Per loop, subject to realtime imprecision                                             | Per loop if \<= 1f, once otherwise                                        |
| Interrupt Features            | Everything                                               | Everything                                                                            | Everything                                                                |
| Interrupt Behavior            | Crossfade from current static pose to interrupting state | End both previous states and start interrupting state with inertial blend             | End both previous states and start interrupting state with inertial blend |
| Sub-State Machines            | Supported                                                | Not Supported                                                                         | Supported                                                                 |
| Entry and Exit                | Supported                                                | Partially Supported                                                                   | Supported                                                                 |
| Any States                    | Supported                                                | Supported                                                                             | Supported                                                                 |
| Initial State                 | Evaluated on first update                                | Force default state                                                                   | Evaluated on first update                                                 |
| Trigger Consumption           | First in update                                          | First in update                                                                       | First in update per state, then globally cleared at end of update         |

## Motions

|                        | Game Objects | V1                           | V2           |
|------------------------|--------------|------------------------------|--------------|
| Clips                  | Supported    | Supported                    | Supported    |
| 1D Blend Trees         | Supported    | Supported                    | Supported    |
| 2D Simple Directional  | Supported    | Does not support center clip | Supported    |
| 2D Freeform Cartesian  | Supported    | Supported                    | Supported    |
| 2D Freform Directional | Supported    | Supported                    | Supported    |
| Direct                 | Supported    | Supported                    | Experimental |
| Override Controllers   | Supported    | Supported                    | Supported    |

## Layers

|                            | Game Objects | V1                              | V2            |
|----------------------------|--------------|---------------------------------|---------------|
| Multiple Layers            | Supported    | Supported                       | Supported     |
| Sync Layers                | Supported    | Not Supported                   | Supported     |
| Sync Affects Timing Option | Supported    | Not Supported                   | Supported     |
| Layer Masks                | Supported    | Not Supported                   | Supported     |
| Override Layers            | Supported    | Always Weight 1                 | Supported     |
| Additive Layers            | Supported    | Samples with baked layer weight | Experimental  |
| Dynamic Layer Weights      | Supported    | Not Supported                   | Supported     |
| IK Pass                    | Supported    | Not Supported                   | Not Supported |

## API

|                           | Game Objects              | V1                                                                  | V2                                                     |
|---------------------------|---------------------------|---------------------------------------------------------------------|--------------------------------------------------------|
| MecanimAspect             | Not Applicable            | Supported                                                           | Supported                                              |
| Parameter Get/Set         | Name, Hash, or Index      | Name, Hash, or Index                                                | Name or Index                                          |
| Parameter Index Find      | Requires manual iteration | Authoring API or manual iteration reading the dynamic buffer length | Authoring API or MecanimAspect API with string or hash |
| Layer Index Find          | Supported                 | Not Supported                                                       | Supported                                              |
| Get State by Name         | Not Supported             | Requires manual search through blob                                 | Obtained via full name or hash                         |
| Layer Weight Modification | Supported                 | Not Supported                                                       | Supported                                              |
| Apply Root Motion Toggle  | Supported                 | IComponentData only                                                 | Supported                                              |
| Manual Updating           | Supported                 | Not Supported                                                       | Supported                                              |
| Crossfade on Demand       | Supported                 | Supported                                                           | Not Supported                                          |
| Info On All Clips Sampled | Supported                 | Supported                                                           | Not Supported                                          |
| State Transition Events   | Not Supported             | Not Supported                                                       | Accumulated in dynamic buffer                          |
| Clip Events               | Supported                 | Accumulated in dynamic buffer                                       | Not Supported                                          |
| State Behaviours          | Supported                 | Not Supported                                                       | Not Supported                                          |
