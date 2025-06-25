using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;

namespace Latios.Navigator.Utils
{
    internal struct NativeMinHeap<TElement, TComparer> : IDisposable
        where TElement : unmanaged // Element type must be unmanaged
        where TComparer : struct, IComparer<TElement>
    {
        NativeList<TElement> m_data;
        TComparer            m_comparer; // Store the comparer instance

        public bool IsEmpty => m_data.Length == 0;

        public NativeMinHeap(int initialCapacity,
            Allocator allocator,
            TComparer comparer = default)
        {
            m_data     = new NativeList<TElement>(initialCapacity, allocator);
            m_comparer = comparer; // Initialize the comparer
        }

        // Add an element and maintain heap property
        public void Enqueue(TElement element)
        {
            m_data.Add(element);
            SiftUp(m_data.Length - 1);
        }

        // Remove and return the smallest element
        public TElement Dequeue()
        {
            if (m_data.Length == 0) throw new InvalidOperationException("Heap is empty");

            var minElement = m_data[0];
            var lastIndex = m_data.Length - 1;

            // Move the last element to the root
            m_data[0] = m_data[lastIndex];
            m_data.RemoveAt(lastIndex); // Remove the last element

            // Restore heap property from the root
            if (m_data.Length > 0) SiftDown(0);

            return minElement;
        }

        public bool TryDequeue(out TElement element)
        {
            if (IsEmpty)
            {
                element = default;

                return false;
            }


            element = Dequeue();

            return true;
        }

        // Move element up the heap
        void SiftUp(int index)
        {
            if (index <= 0) return;

            var parentIndex = (index - 1) / 2;

            // Use the comparer: if element at index is smaller than its parent
            if (m_comparer.Compare(m_data[index], m_data[parentIndex]) < 0)
            {
                // Swap
                (m_data[index], m_data[parentIndex]) = (m_data[parentIndex], m_data[index]);

                // Continue sifting up from the parent index
                SiftUp(parentIndex);
            }
        }

        public TElement Peek()
        {
            if (m_data.Length == 0) throw new InvalidOperationException("Heap is empty");

            return m_data[0];
        }


        public bool TryPeek(out TElement element)
        {
            if (IsEmpty)
            {
                element = default;

                return false;
            }


            element = m_data[0];

            return true;
        }

        public void Clear()
        {
            m_data.Clear();
        }

        // Move element down the heap
        void SiftDown(int index)
        {
            var leftChildIndex = 2 * index + 1;
            var rightChildIndex = 2 * index + 2;
            var smallestIndex = index; // Assume current node is smallest

            // Compare with left child (using comparer)
            if (leftChildIndex < m_data.Length && m_comparer.Compare(m_data[leftChildIndex], m_data[smallestIndex]) < 0)
                smallestIndex = leftChildIndex;

            // Compare with right child (using comparer)
            if (rightChildIndex < m_data.Length &&
                m_comparer.Compare(m_data[rightChildIndex], m_data[smallestIndex]) < 0) smallestIndex = rightChildIndex;

            // If the smallest is not the current node, swap and continue sifting down
            if (smallestIndex != index)
            {
                (m_data[index], m_data[smallestIndex]) = (m_data[smallestIndex], m_data[index]);

                SiftDown(smallestIndex);
            }
        }

        public void Dispose()
        {
            if (m_data.IsCreated) m_data.Dispose();
        }

        // Optional: Dispose with JobHandle
        public JobHandle Dispose(JobHandle inputDeps)
        {
            if (m_data.IsCreated) return m_data.Dispose(inputDeps);

            return inputDeps;
        }
    }
}