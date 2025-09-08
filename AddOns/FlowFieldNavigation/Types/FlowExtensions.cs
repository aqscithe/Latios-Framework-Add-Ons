using Unity.Collections;
using Unity.Mathematics;

namespace Latios.FlowFieldNavigation
{
    public static class FlowExtensions
    {
        public static float2 GetDirection(this Flow flow, int index)
        {
            var direction = flow.DirectionMap[index];
            var rotatedDirection = math.mul(flow.Transform.Value.rotation, direction.x0y());
            return rotatedDirection.xz;
        }

        internal static int4 GetCluster(int2 cell, int2 clusterSize, int width, int height)
        {
            // Вычисляем кластер для цели
            int clusterX = cell.x / clusterSize.x;
            int clusterY = cell.y / clusterSize.y;
                    
            int2 clusterMin = new int2(clusterX * clusterSize.x, clusterY * clusterSize.y);
            int2 clusterMax = new int2(clusterMin.x + clusterSize.x - 1, clusterMin.y + clusterSize.y - 1);
                    
            // Ограничиваем границы кластера размерами поля
            clusterMax = math.min(clusterMax, new int2(width - 1, height - 1));
                    
            return new int4(clusterMin, clusterMax);
        }

        internal static bool TryGetNeighbourCluster(int4 cluster, Grid.Direction direction, int2 clusterSize, int width, int height, out int4 neighbour)
        {
            int2 currentClusterMin = cluster.xy;
            int2 currentClusterMax = cluster.zw;
            
            // Вычисляем координаты центра текущего кластера
            int2 clusterCenter = (currentClusterMin + currentClusterMax) / 2;
            
            // Вычисляем смещение для соседнего кластера
            int2 offset = int2.zero;
            
            switch (direction)
            {
                case Grid.Direction.Up:
                    offset = new int2(0, clusterSize.y);
                    break;
                case Grid.Direction.Down:
                    offset = new int2(0, -clusterSize.y);
                    break;
                case Grid.Direction.Left:
                    offset = new int2(-clusterSize.x, 0);
                    break;
                case Grid.Direction.Right:
                    offset = new int2(clusterSize.x, 0);
                    break;
                case Grid.Direction.UpLeft:
                    offset = new int2(-clusterSize.x, clusterSize.y);
                    break;
                case Grid.Direction.UpRight:
                    offset = new int2(clusterSize.x, clusterSize.y);
                    break;
                case Grid.Direction.DownLeft:
                    offset = new int2(-clusterSize.x, -clusterSize.y);
                    break;
                case Grid.Direction.DownRight:
                    offset = new int2(clusterSize.x, -clusterSize.y);
                    break;
                default:
                    neighbour = int4.zero;
                    return false;
            }
            
            // Вычисляем центр соседнего кластера
            int2 neighborCenter = clusterCenter + offset;
            
            // Вычисляем границы соседнего кластера
            int2 neighborMin = new int2(
                (neighborCenter.x / clusterSize.x) * clusterSize.x,
                (neighborCenter.y / clusterSize.y) * clusterSize.y
            );
            
            int2 neighborMax = new int2(
                math.min(neighborMin.x + clusterSize.x - 1, width - 1),
                math.min(neighborMin.y + clusterSize.y - 1, height - 1)
            );
            
            // Проверяем, что кластер находится в пределах сетки
            if (neighborMin.x < 0 || neighborMin.y < 0 || neighborMax.x < 0 || neighborMax.y < 0)
            {
                neighbour = int4.zero;
                return false;
            }
            
            neighbour = new int4(neighborMin, neighborMax);
            return true;
        }

        internal static void CalculateCostsForCluster(int width, int height, int2 clusterSize,
            ref NativePriorityQueue<FlowFieldInternal.CostEntry, FlowFieldInternal.CostComparer> queue,
            ref NativeArray<float> costs,
            ref NativeArray<int> passabilityMap,
            ref NativeBitArray visited
            )
        {
            while (queue.TryDequeue(out var current))
            {
                var cluster = GetCluster(current.Pos, clusterSize, width, height);
                var min = cluster.xy;
                var max = cluster.zw;
                
                var currentCost = costs[Grid.CellToIndex(width, current.Pos)];

                for (var dir = Grid.Direction.Up; dir <= Grid.Direction.DownRight; dir++)
                {
                    if (!Grid.TryGetNeighborCell(width, height, current.Pos, dir, out var neighbor)) continue;
                    if (math.any(neighbor < min) || math.any(neighbor > max)) continue;

                    var neighborIndex = Grid.CellToIndex(width, neighbor);
                    if (visited.IsSet(neighborIndex)) continue;

                    if (passabilityMap[neighborIndex] < 0) continue;

                    var moveCost = passabilityMap[neighborIndex] + (dir > Grid.Direction.Right ? math.SQRT2 : 1);

                    var newCost = currentCost + moveCost;

                    if (newCost < costs[neighborIndex])
                    {
                        costs[neighborIndex] = newCost;
                        queue.Enqueue(new(neighbor, newCost));
                        visited.Set(neighborIndex, true);
                    }
                }
            }
        }
        
        internal static void CalculateCostsForCluster(int width, int height, int4 cluster,
            ref NativePriorityQueue<FlowFieldInternal.CostEntry, FlowFieldInternal.CostComparer> queue,
            ref NativeArray<float> costs,
            ref NativeArray<int> passabilityMap
            // ref NativeBitArray visited
            )
        {
            var min = cluster.xy;
            var max = cluster.zw;
            
            while (queue.TryDequeue(out var current))
            {
                var currentCost = costs[Grid.CellToIndex(width, current.Pos)];

                for (var dir = Grid.Direction.Up; dir <= Grid.Direction.DownRight; dir++)
                {
                    if (!Grid.TryGetNeighborCell(width, height, current.Pos, dir, out var neighbor)) continue;
                    if (math.any(neighbor < min) || math.any(neighbor > max)) continue;

                    var neighborIndex = Grid.CellToIndex(width, neighbor);
                    // if (visited.IsSet(neighborIndex)) continue;

                    if (passabilityMap[neighborIndex] < 0) continue;

                    var moveCost = passabilityMap[neighborIndex] + (dir > Grid.Direction.Right ? math.SQRT2 : 1);

                    var newCost = currentCost + moveCost;

                    if (newCost < costs[neighborIndex])
                    {
                        costs[neighborIndex] = newCost;
                        queue.Enqueue(new(neighbor, newCost));
                        // visited.Set(neighborIndex, true);
                    }
                }
            }
        }
    }
}