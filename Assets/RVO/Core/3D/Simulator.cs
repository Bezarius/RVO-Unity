using System.Collections.Generic;
using System.Threading;
using Vector3 = UnityEngine.Vector3;

namespace RVO.Core._3D
{
    /**
     * <summary>Defines the simulation.</summary>
     */
    public class Simulator
    {
        /**
         * <summary>Defines a worker.</summary>
         */
        private class Worker
        {
            private readonly ManualResetEvent _doneEvent;
            private readonly int _end;
            private readonly int _start;

            /**
             * <summary>Constructs and initializes a worker.</summary>
             *
             * <param name="start">Start.</param>
             * <param name="end">End.</param>
             * <param name="doneEvent">Done event.</param>
             */
            internal Worker(int start, int end, ManualResetEvent doneEvent)
            {
                _start = start;
                _end = end;
                _doneEvent = doneEvent;
            }

            /**
             * <summary>Performs a simulation step.</summary>
             *
             * <param name="obj">Unused.</param>
             */
            internal void Step(object obj)
            {
                for (int agentNo = _start; agentNo < _end; ++agentNo)
                {
                    Instance.Agents[agentNo].ComputeNeighbors();
                    Instance.Agents[agentNo].ComputeNewVelocity();
                }

                _doneEvent.Set();
            }

            /**
             * <summary>updates the two-dimensional position and
             * two-dimensional velocity of each agent.</summary>
             *
             * <param name="obj">Unused.</param>
             */
            internal void Update(object obj)
            {
                for (int agentNo = _start; agentNo < _end; ++agentNo)
                {
                    Instance.Agents[agentNo].Update();
                }

                _doneEvent.Set();
            }
        }

        public bool IsMultithreaded = true;

        internal IList<Agent> Agents;
        internal KdTree KdTree;
        internal float TimeStep;

        private static readonly Simulator _instance = new Simulator();

        private Agent _defaultAgent;
        private ManualResetEvent[] _doneEvents;
        private Worker[] _workers;
        private int _numWorkers;
        private float _globalTime;

        public static Simulator Instance
        {
            get
            {
                return _instance;
            }
        }

        /**
         * <summary>Adds a new agent with default properties to the simulation.
         * </summary>
         *
         * <returns>The number of the agent, or -1 when the agent defaults have
         * not been set.</returns>
         *
         * <param name="position">The two-dimensional starting position of this
         * agent.</param>
         */
        public int AddAgent(Vector3 position)
        {
            if (_defaultAgent == null)
            {
                return -1;
            }

            var agent = new Agent
            {
                Id = Agents.Count,
                MaxNeighbors = _defaultAgent.MaxNeighbors,
                MaxSpeed = _defaultAgent.MaxSpeed,
                NeighborDist = _defaultAgent.NeighborDist,
                Position = position,
                Radius = _defaultAgent.Radius,
                TimeHorizon = _defaultAgent.TimeHorizon,
                TimeHorizonObst = _defaultAgent.TimeHorizonObst,
                Velocity = _defaultAgent.Velocity
            };
            Agents.Add(agent);

            return agent.Id;
        }

        /**
         * <summary>Adds a new agent to the simulation.</summary>
         *
         * <returns>The number of the agent.</returns>
         *
         * <param name="position">The two-dimensional starting position of this
         * agent.</param>
         * <param name="neighborDist">The maximum distance (center point to
         * center point) to other agents this agent takes into account in the
         * navigation. The larger this number, the longer the running time of
         * the simulation. If the number is too low, the simulation will not be
         * safe. Must be non-negative.</param>
         * <param name="maxNeighbors">The maximum number of other agents this
         * agent takes into account in the navigation. The larger this number,
         * the longer the running time of the simulation. If the number is too
         * low, the simulation will not be safe.</param>
         * <param name="timeHorizon">The minimal amount of time for which this
         * agent's velocities that are computed by the simulation are safe with
         * respect to other agents. The larger this number, the sooner this
         * agent will respond to the presence of other agents, but the less
         * freedom this agent has in choosing its velocities. Must be positive.
         * </param>
         * <param name="timeHorizonObst">The minimal amount of time for which
         * this agent's velocities that are computed by the simulation are safe
         * with respect to obstacles. The larger this number, the sooner this
         * agent will respond to the presence of obstacles, but the less freedom
         * this agent has in choosing its velocities. Must be positive.</param>
         * <param name="radius">The radius of this agent. Must be non-negative.
         * </param>
         * <param name="maxSpeed">The maximum speed of this agent. Must be
         * non-negative.</param>
         * <param name="velocity">The initial two-dimensional linear velocity of
         * this agent.</param>
         */
        public int AddAgent(Vector3 position, float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, Vector3 velocity)
        {
            Agent agent = new Agent
            {
                Id = Agents.Count,
                MaxNeighbors = maxNeighbors,
                MaxSpeed = maxSpeed,
                NeighborDist = neighborDist,
                Position = position,
                Radius = radius,
                TimeHorizon = timeHorizon,
                TimeHorizonObst = timeHorizonObst,
                Velocity = velocity
            };

            Agents.Add(agent);

            return agent.Id;
        }

        /**
         * <summary>Clears the simulation.</summary>
         */
        public void Clear()
        {
            Agents = new List<Agent>();
            _defaultAgent = null;
            KdTree = new KdTree();
            _globalTime = 0.0f;
            TimeStep = 0.1f;

            SetNumWorkers(0);
        }

        /**
         * <summary>Performs a simulation step and updates the two-dimensional
         * position and two-dimensional velocity of each agent.</summary>
         *
         * <returns>The global time after the simulation step.</returns>
         */
        public float DoStep()
        {
            if (IsMultithreaded)
            {
                return MultiThreadedStep();
            }
            return SingleThreadedStep();
        }

        private float SingleThreadedStep()
        {
            KdTree.BuildAgentTree();

            for (int i = 0; i < Agents.Count; ++i)
            {
                Instance.Agents[i].ComputeNeighbors();
                Instance.Agents[i].ComputeNewVelocity();
                Instance.Agents[i].Update();
            }

            _globalTime += TimeStep;

            return _globalTime;
        }

        private float MultiThreadedStep()
        {
            if (_workers == null)
            {
                _workers = new Worker[_numWorkers];
                _doneEvents = new ManualResetEvent[_workers.Length];

                for (int block = 0; block < _workers.Length; ++block)
                {
                    _doneEvents[block] = new ManualResetEvent(false);
                    _workers[block] = new Worker(block * GetNumAgents() / _workers.Length,
                        (block + 1) * GetNumAgents() / _workers.Length, _doneEvents[block]);
                }
            }

            KdTree.BuildAgentTree();

            for (int block = 0; block < _workers.Length; ++block)
            {
                _doneEvents[block].Reset();
                ThreadPool.QueueUserWorkItem(_workers[block].Step);
            }

            WaitHandle.WaitAll(_doneEvents);

            for (int block = 0; block < _workers.Length; ++block)
            {
                _doneEvents[block].Reset();
                ThreadPool.QueueUserWorkItem(_workers[block].Update);
            }

            WaitHandle.WaitAll(_doneEvents);

            _globalTime += TimeStep;

            return _globalTime;
        }

        /**
         * <summary>Returns the specified agent neighbor of the specified agent.
         * </summary>
         *
         * <returns>The number of the neighboring agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose agent neighbor is
         * to be retrieved.</param>
         * <param name="neighborNo">The number of the agent neighbor to be
         * retrieved.</param>
         */
        public int GetAgentAgentNeighbor(int agentNo, int neighborNo)
        {
            return Agents[agentNo].AgentNeighbors[neighborNo].Value.Id;
        }

        /**
         * <summary>Returns the maximum neighbor count of a specified agent.
         * </summary>
         *
         * <returns>The present maximum neighbor count of the agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose maximum neighbor
         * count is to be retrieved.</param>
         */
        public int GetAgentMaxNeighbors(int agentNo)
        {
            return Agents[agentNo].MaxNeighbors;
        }

        /**
         * <summary>Returns the maximum speed of a specified agent.</summary>
         *
         * <returns>The present maximum speed of the agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose maximum speed is
         * to be retrieved.</param>
         */
        public float GetAgentMaxSpeed(int agentNo)
        {
            return Agents[agentNo].MaxSpeed;
        }

        /**
         * <summary>Returns the maximum neighbor distance of a specified agent.
         * </summary>
         *
         * <returns>The present maximum neighbor distance of the agent.
         * </returns>
         *
         * <param name="agentNo">The number of the agent whose maximum neighbor
         * distance is to be retrieved.</param>
         */
        public float GetAgentNeighborDist(int agentNo)
        {
            return Agents[agentNo].NeighborDist;
        }

        /**
         * <summary>Returns the count of agent neighbors taken into account to
         * compute the current velocity for the specified agent.</summary>
         *
         * <returns>The count of agent neighbors taken into account to compute
         * the current velocity for the specified agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose count of agent
         * neighbors is to be retrieved.</param>
         */
        public int GetAgentNumAgentNeighbors(int agentNo)
        {
            return Agents[agentNo].AgentNeighbors.Count;
        }

        /**
         * <summary>Returns the two-dimensional position of a specified agent.
         * </summary>
         *
         * <returns>The present two-dimensional position of the (center of the)
         * agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose two-dimensional
         * position is to be retrieved.</param>
         */
        public Vector3 GetAgentPosition(int agentNo)
        {
            return Agents[agentNo].Position;
        }

        /**
         * <summary>Returns the two-dimensional preferred velocity of a
         * specified agent.</summary>
         *
         * <returns>The present two-dimensional preferred velocity of the agent.
         * </returns>
         *
         * <param name="agentNo">The number of the agent whose two-dimensional
         * preferred velocity is to be retrieved.</param>
         */
        public Vector3 GetAgentPrefVelocity(int agentNo)
        {
            return Agents[agentNo].PrefVelocity;
        }

        /**
         * <summary>Returns the radius of a specified agent.</summary>
         *
         * <returns>The present radius of the agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose radius is to be
         * retrieved.</param>
         */
        public float GetAgentRadius(int agentNo)
        {
            return Agents[agentNo].Radius;
        }

        /**
         * <summary>Returns the time horizon of a specified agent.</summary>
         *
         * <returns>The present time horizon of the agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose time horizon is
         * to be retrieved.</param>
         */
        public float GetAgentTimeHorizon(int agentNo)
        {
            return Agents[agentNo].TimeHorizon;
        }

        /**
         * <summary>Returns the time horizon with respect to obstacles of a
         * specified agent.</summary>
         *
         * <returns>The present time horizon with respect to obstacles of the
         * agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose time horizon with
         * respect to obstacles is to be retrieved.</param>
         */
        public float GetAgentTimeHorizonObst(int agentNo)
        {
            return Agents[agentNo].TimeHorizonObst;
        }

        /**
         * <summary>Returns the two-dimensional linear velocity of a specified
         * agent.</summary>
         *
         * <returns>The present two-dimensional linear velocity of the agent.
         * </returns>
         *
         * <param name="agentNo">The number of the agent whose two-dimensional
         * linear velocity is to be retrieved.</param>
         */
        public Vector3 GetAgentVelocity(int agentNo)
        {
            return Agents[agentNo].Velocity;
        }

        /**
         * <summary>Returns the global time of the simulation.</summary>
         *
         * <returns>The present global time of the simulation (zero initially).
         * </returns>
         */
        public float GetGlobalTime()
        {
            return _globalTime;
        }

        /**
         * <summary>Returns the count of agents in the simulation.</summary>
         *
         * <returns>The count of agents in the simulation.</returns>
         */
        public int GetNumAgents()
        {
            return Agents.Count;
        }

        /**
         * <summary>Returns the count of workers.</summary>
         *
         * <returns>The count of workers.</returns>
         */
        public int GetNumWorkers()
        {
            return _numWorkers;
        }

        /**
         * <summary>Returns the time step of the simulation.</summary>
         *
         * <returns>The present time step of the simulation.</returns>
         */
        public float GetTimeStep()
        {
            return TimeStep;
        }

        /**
         * <summary>Sets the default properties for any new agent that is added.
         * </summary>
         *
         * <param name="neighborDist">The default maximum distance (center point
         * to center point) to other agents a new agent takes into account in
         * the navigation. The larger this number, the longer he running time of
         * the simulation. If the number is too low, the simulation will not be
         * safe. Must be non-negative.</param>
         * <param name="maxNeighbors">The default maximum number of other agents
         * a new agent takes into account in the navigation. The larger this
         * number, the longer the running time of the simulation. If the number
         * is too low, the simulation will not be safe.</param>
         * <param name="timeHorizon">The default minimal amount of time for
         * which a new agent's velocities that are computed by the simulation
         * are safe with respect to other agents. The larger this number, the
         * sooner an agent will respond to the presence of other agents, but the
         * less freedom the agent has in choosing its velocities. Must be
         * positive.</param>
         * <param name="timeHorizonObst">The default minimal amount of time for
         * which a new agent's velocities that are computed by the simulation
         * are safe with respect to obstacles. The larger this number, the
         * sooner an agent will respond to the presence of obstacles, but the
         * less freedom the agent has in choosing its velocities. Must be
         * positive.</param>
         * <param name="radius">The default radius of a new agent. Must be
         * non-negative.</param>
         * <param name="maxSpeed">The default maximum speed of a new agent. Must
         * be non-negative.</param>
         * <param name="velocity">The default initial two-dimensional linear
         * velocity of a new agent.</param>
         */
        public void SetAgentDefaults(float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, Vector3 velocity)
        {
            if (_defaultAgent == null)
            {
                _defaultAgent = new Agent();
            }

            _defaultAgent.MaxNeighbors = maxNeighbors;
            _defaultAgent.MaxSpeed = maxSpeed;
            _defaultAgent.NeighborDist = neighborDist;
            _defaultAgent.Radius = radius;
            _defaultAgent.TimeHorizon = timeHorizon;
            _defaultAgent.TimeHorizonObst = timeHorizonObst;
            _defaultAgent.Velocity = velocity;
        }

        /**
         * <summary>Sets the maximum neighbor count of a specified agent.
         * </summary>
         *
         * <param name="agentNo">The number of the agent whose maximum neighbor
         * count is to be modified.</param>
         * <param name="maxNeighbors">The replacement maximum neighbor count.
         * </param>
         */
        public void SetAgentMaxNeighbors(int agentNo, int maxNeighbors)
        {
            Agents[agentNo].MaxNeighbors = maxNeighbors;
        }

        /**
         * <summary>Sets the maximum speed of a specified agent.</summary>
         *
         * <param name="agentNo">The number of the agent whose maximum speed is
         * to be modified.</param>
         * <param name="maxSpeed">The replacement maximum speed. Must be
         * non-negative.</param>
         */
        public void SetAgentMaxSpeed(int agentNo, float maxSpeed)
        {
            Agents[agentNo].MaxSpeed = maxSpeed;
        }

        /**
         * <summary>Sets the maximum neighbor distance of a specified agent.
         * </summary>
         *
         * <param name="agentNo">The number of the agent whose maximum neighbor
         * distance is to be modified.</param>
         * <param name="neighborDist">The replacement maximum neighbor distance.
         * Must be non-negative.</param>
         */
        public void SetAgentNeighborDist(int agentNo, float neighborDist)
        {
            Agents[agentNo].NeighborDist = neighborDist;
        }

        /**
         * <summary>Sets the two-dimensional position of a specified agent.
         * </summary>
         *
         * <param name="agentNo">The number of the agent whose two-dimensional
         * position is to be modified.</param>
         * <param name="position">The replacement of the two-dimensional
         * position.</param>
         */
        public void SetAgentPosition(int agentNo, Vector3 position)
        {
            Agents[agentNo].Position = position;
        }

        /**
         * <summary>Sets the two-dimensional preferred velocity of a specified
         * agent.</summary>
         *
         * <param name="agentNo">The number of the agent whose two-dimensional
         * preferred velocity is to be modified.</param>
         * <param name="prefVelocity">The replacement of the two-dimensional
         * preferred velocity.</param>
         */
        public void SetAgentPrefVelocity(int agentNo, Vector3 prefVelocity)
        {
            Agents[agentNo].PrefVelocity = prefVelocity;
        }

        /**
         * <summary>Sets the radius of a specified agent.</summary>
         *
         * <param name="agentNo">The number of the agent whose radius is to be
         * modified.</param>
         * <param name="radius">The replacement radius. Must be non-negative.
         * </param>
         */
        public void SetAgentRadius(int agentNo, float radius)
        {
            Agents[agentNo].Radius = radius;
        }

        /**
         * <summary>Sets the time horizon of a specified agent with respect to
         * other agents.</summary>
         *
         * <param name="agentNo">The number of the agent whose time horizon is
         * to be modified.</param>
         * <param name="timeHorizon">The replacement time horizon with respect
         * to other agents. Must be positive.</param>
         */
        public void SetAgentTimeHorizon(int agentNo, float timeHorizon)
        {
            Agents[agentNo].TimeHorizon = timeHorizon;
        }

        /**
         * <summary>Sets the time horizon of a specified agent with respect to
         * obstacles.</summary>
         *
         * <param name="agentNo">The number of the agent whose time horizon with
         * respect to obstacles is to be modified.</param>
         * <param name="timeHorizonObst">The replacement time horizon with
         * respect to obstacles. Must be positive.</param>
         */
        public void SetAgentTimeHorizonObst(int agentNo, float timeHorizonObst)
        {
            Agents[agentNo].TimeHorizonObst = timeHorizonObst;
        }

        /**
         * <summary>Sets the two-dimensional linear velocity of a specified
         * agent.</summary>
         *
         * <param name="agentNo">The number of the agent whose two-dimensional
         * linear velocity is to be modified.</param>
         * <param name="velocity">The replacement two-dimensional linear
         * velocity.</param>
         */
        public void SetAgentVelocity(int agentNo, Vector3 velocity)
        {
            Agents[agentNo].Velocity = velocity;
        }

        /**
         * <summary>Sets the global time of the simulation.</summary>
         *
         * <param name="globalTime">The global time of the simulation.</param>
         */
        public void SetGlobalTime(float globalTime)
        {
            _globalTime = globalTime;
        }

        /**
         * <summary>Sets the number of workers.</summary>
         *
         * <param name="numWorkers">The number of workers.</param>
         */
        public void SetNumWorkers(int numWorkers)
        {
            _numWorkers = numWorkers;

            if (_numWorkers <= 0)
            {
                int completionPorts;
                ThreadPool.GetMinThreads(out _numWorkers, out completionPorts);
            }
            _workers = null;
        }

        /**
         * <summary>Sets the time step of the simulation.</summary>
         *
         * <param name="timeStep">The time step of the simulation. Must be
         * positive.</param>
         */
        public void SetTimeStep(float timeStep)
        {
            TimeStep = timeStep;
        }

        /**
         * <summary>Constructs and initializes a simulation.</summary>
         */
        private Simulator()
        {
            Clear();
        }
    }
}
