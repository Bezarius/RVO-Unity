using System;
using UnityEngine;

namespace RVO.Core._3D
{
    /**
     * <summary>Defines k-D trees for agents and static obstacles in the
     * simulation.</summary>
     */
    internal class KdTree
    {
        /**
         * <summary>Defines a node of an agent k-D tree.</summary>
         */
        private struct AgentTreeNode
        {
            internal int Begin;
            internal int End;
            internal int Left;
            internal int Right;
            internal Vector3 MinVec;
            internal Vector3 MaxVec;
        }

        /**
         * <summary>The maximum size of an agent k-D tree leaf.</summary>
         */
        private const int MaxLeafSize = 10;

        private Agent[] _agents;
        private AgentTreeNode[] _agentTree;

        /**
         * <summary>Builds an agent k-D tree.</summary>
         */
        internal void BuildAgentTree()
        {
            if (_agents == null || _agents.Length != Simulator.Instance.Agents.Count)
            {
                _agents = new Agent[Simulator.Instance.Agents.Count];

                for (int i = 0; i < _agents.Length; ++i)
                {
                    _agents[i] = Simulator.Instance.Agents[i];
                }

                _agentTree = new AgentTreeNode[2 * _agents.Length];

                for (int i = 0; i < _agentTree.Length; ++i)
                {
                    _agentTree[i] = new AgentTreeNode();
                }
            }

            if (_agents.Length != 0)
            {
                BuildAgentTreeRecursive(0, _agents.Length, 0);
            }
        }


        /**
         * <summary>Computes the agent neighbors of the specified agent.
         * </summary>
         *
         * <param name="agent">The agent for which agent neighbors are to be
         * computed.</param>
         * <param name="rangeSq">The squared range around the agent.</param>
         */
        internal void ComputeAgentNeighbors(Agent agent, ref float rangeSq)
        {
            QueryAgentTreeRecursive(agent, ref rangeSq, 0);
        }

        /**
         * <summary>Recursive method for building an agent k-D tree.</summary>
         *
         * <param name="begin">The beginning agent k-D tree node node index.
         * </param>
         * <param name="end">The ending agent k-D tree node index.</param>
         * <param name="node">The current agent k-D tree node index.</param>
         */
        private void BuildAgentTreeRecursive(int begin, int end, int node)
        {
            _agentTree[node].Begin = begin;
            _agentTree[node].End = end;
            _agentTree[node].MinVec = _agents[begin].Position;
            _agentTree[node].MaxVec = _agents[begin].Position;

            for (int i = begin + 1; i < end; ++i)
            {
                _agentTree[node].MinVec = new Vector3(
                    Math.Max(_agentTree[node].MinVec.x, _agents[i].Position.x),
                    Math.Max(_agentTree[node].MinVec.y, _agents[i].Position.y),
                    Math.Max(_agentTree[node].MinVec.z, _agents[i].Position.z));

                _agentTree[node].MaxVec = new Vector3(
                    Math.Max(_agentTree[node].MaxVec.x, _agents[i].Position.x),
                    Math.Max(_agentTree[node].MaxVec.y, _agents[i].Position.y),
                    Math.Max(_agentTree[node].MaxVec.z, _agents[i].Position.z));
            }

            if (end - begin > MaxLeafSize)
            {
                int coord;

                if (_agentTree[node].MaxVec[0] - _agentTree[node].MinVec[0] > _agentTree[node].MaxVec[1] - _agentTree[node].MinVec[1] &&
                    _agentTree[node].MaxVec[0] - _agentTree[node].MinVec[0] > _agentTree[node].MaxVec[2] - _agentTree[node].MinVec[2])
                {
                    coord = 0;
                }
                else if (_agentTree[node].MaxVec[1] - _agentTree[node].MinVec[1] > _agentTree[node].MaxVec[2] - _agentTree[node].MinVec[2])
                {
                    coord = 1;
                }
                else
                {
                    coord = 2;
                }

                float splitValue = 0.5f * (_agentTree[node].MaxVec[coord] + _agentTree[node].MinVec[coord]);

                int left = begin;

                int right = end;

                while (left < right)
                {
                    while (left < right && _agents[left].Position[coord] < splitValue)
                    {
                        ++left;
                    }

                    while (right > left && _agents[right - 1].Position[coord] >= splitValue)
                    {
                        --right;
                    }

                    if (left < right)
                    {
                        Agent tempAgent = _agents[left];
                        _agents[left] = _agents[right - 1];
                        _agents[right - 1] = tempAgent;
                        ++left;
                        --right;
                    }
                }

                int leftSize = left - begin;

                if (leftSize == 0)
                {
                    ++leftSize;
                    ++left;
                    ++right;
                }

                _agentTree[node].Left = node + 1;
                _agentTree[node].Right = node + 2 * leftSize;

                BuildAgentTreeRecursive(begin, left, _agentTree[node].Left);
                BuildAgentTreeRecursive(left, end, _agentTree[node].Right);
            }
        }

        /**
         * <summary>Recursive method for computing the agent neighbors of the
         * specified agent.</summary>
         *
         * <param name="agent">The agent for which agent neighbors are to be
         * computed.</param>
         * <param name="rangeSq">The squared range around the agent.</param>
         * <param name="node">The current agent k-D tree node index.</param>
         */
        private void QueryAgentTreeRecursive(Agent agent, ref float rangeSq, int node)
        {
            if (_agentTree[node].End - _agentTree[node].Begin <= MaxLeafSize)
            {
                for (int i = _agentTree[node].Begin; i < _agentTree[node].End; ++i)
                {
                    agent.InsertAgentNeighbor(_agents[i], ref rangeSq);
                }
            }
            else
            {
                var minXLeft = Math.Max(0.0f, _agentTree[_agentTree[node].Left].MinVec[0] - agent.Position[0]);
                var maxXLeft = Math.Max(0.0f, agent.Position[0] - _agentTree[_agentTree[node].Left].MaxVec[0]);
                var minYLeft = Math.Max(0.0f, _agentTree[_agentTree[node].Left].MinVec[1] - agent.Position[1]);
                var maxYLeft = Math.Max(0.0f, agent.Position[1] - _agentTree[_agentTree[node].Left].MaxVec[1]);
                var minZLeft = Math.Max(0.0f, _agentTree[_agentTree[node].Left].MinVec[2] - agent.Position[2]);
                var maxZLeft = Math.Max(0.0f, agent.Position[2] - _agentTree[_agentTree[node].Left].MaxVec[2]);

                float distSqLeft = 
                    minXLeft * minXLeft + maxXLeft * maxXLeft + 
                    minYLeft * minYLeft + maxYLeft * maxYLeft + 
                    minZLeft * minZLeft + maxZLeft * maxZLeft;

                var minXRight = Math.Max(0.0f, _agentTree[_agentTree[node].Right].MinVec[0] - agent.Position[0]);
                var maxXRight = Math.Max(0.0f, agent.Position[0] - _agentTree[_agentTree[node].Right].MaxVec[0]);
                var minYRight = Math.Max(0.0f, _agentTree[_agentTree[node].Right].MinVec[1] - agent.Position[1]);
                var maxYRight = Math.Max(0.0f, agent.Position[1] - _agentTree[_agentTree[node].Right].MaxVec[1]);
                var minZRight = Math.Max(0.0f, _agentTree[_agentTree[node].Right].MinVec[2] - agent.Position[2]);
                var maxZRight = Math.Max(0.0f, agent.Position[2] - _agentTree[_agentTree[node].Right].MaxVec[2]);

                float distSqRight =
                    minXRight * minXRight + maxXRight * maxXRight +
                    minYRight * minYRight + maxYRight * maxYRight +
                    minZRight * minZRight + maxZRight * maxZRight;

                if (distSqLeft < distSqRight)
                {
                    if (distSqLeft < rangeSq)
                    {
                        QueryAgentTreeRecursive(agent, ref rangeSq, _agentTree[node].Left);

                        if (distSqRight < rangeSq)
                        {
                            QueryAgentTreeRecursive(agent, ref rangeSq, _agentTree[node].Right);
                        }
                    }
                }
                else
                {
                    if (distSqRight < rangeSq)
                    {
                        QueryAgentTreeRecursive(agent, ref rangeSq, _agentTree[node].Right);

                        if (distSqLeft < rangeSq)
                        {
                            QueryAgentTreeRecursive(agent, ref rangeSq, _agentTree[node].Left);
                        }
                    }
                }
            }
        }

    }
}
