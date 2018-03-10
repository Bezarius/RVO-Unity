using System;
using System.Collections.Generic;
using RVO.Core._2D;
using UnityEngine;

namespace RVO.Core._3D
{
    /**
     * <summary>Defines an agent in the simulation.</summary>
     */
    internal class Agent
    {
        internal readonly IList<KeyValuePair<float, Agent>> AgentNeighbors = new List<KeyValuePair<float, Agent>>();

        private readonly List<Plane> _orcaPlanes = new List<Plane>();
        internal Vector3 Position;
        internal Vector3 PrefVelocity;
        internal Vector3 Velocity;
        internal int Id = 0;
        internal int MaxNeighbors = 0;
        internal float MaxSpeed = 0.0f;
        internal float NeighborDist = 0.0f;
        internal float Radius = 0.0f;
        internal float TimeHorizon = 0.0f;
        internal float TimeHorizonObst = 0.0f;

        private Vector3 _newVelocity;

        /**
         * <summary>Computes the neighbors of this agent.</summary>
         */
        internal void ComputeNeighbors()
        {
            AgentNeighbors.Clear();

            if (MaxNeighbors > 0)
            {
                float rangeSq = RVOMath.Sqr(NeighborDist);
                Simulator.Instance.KdTree.ComputeAgentNeighbors(this, ref rangeSq);
            }
        }

        /**
         * <summary>Computes the new velocity of this agent.</summary>
         */
        internal void ComputeNewVelocity()
        {
            _orcaPlanes.Clear();

            float invTimeHorizon = 1.0f / TimeHorizon;

            /* Create agent ORCA lines. */
            for (int i = 0; i < AgentNeighbors.Count; ++i)
            {
                Agent other = AgentNeighbors[i].Value;

                Vector3 relativePosition = other.Position - Position;
                Vector3 relativeVelocity = Velocity - other.Velocity;
                float distSq = relativePosition.sqrMagnitude;
                float combinedRadius = Radius + other.Radius;
                float combinedRadiusSq = combinedRadius * combinedRadius;

                Plane plane;
                Vector3 u;

                if (distSq > combinedRadiusSq)
                {
                    /* No collision. */
                    Vector3 w = relativeVelocity - invTimeHorizon * relativePosition;

                    /* Vector from cutoff center to relative velocity. */
                    float wLengthSq = w.sqrMagnitude;
                    //dotProduct1 = w * relativePosition;
                    float dotProduct1 = Vector3.Dot(w, relativePosition);

                    if (dotProduct1 < 0.0f && dotProduct1 * dotProduct1 > combinedRadiusSq * wLengthSq)
                    {
                        /* Project on cut-off circle. */
                        float wLength = Mathf.Sqrt(wLengthSq);
                        Vector3 unitW = w / wLength;

                        plane.Normal = unitW;
                        u = (combinedRadius * invTimeHorizon - wLength) * unitW;
                    }
                    else
                    {
                        /* Project on cone. */

                        float a = distSq;
                        float b = Vector3.Dot(relativePosition, relativeVelocity);
                        float c = relativeVelocity.sqrMagnitude -
                                  Vector3.Cross(relativePosition, relativeVelocity).sqrMagnitude /
                                  (distSq - combinedRadiusSq);
                        float t = (b + Mathf.Sqrt(b * b - a * c)) / a;
                        Vector3 v = Vector3.Scale(
                            new Vector3(relativeVelocity.x - t, relativeVelocity.y - t, relativeVelocity.z - t),
                            relativePosition);
                        float vLen = v.magnitude;
                        Vector3 unitW = v / vLen;
                        plane.Normal = unitW;
                        u = (combinedRadius * t - vLen) * unitW;
                    }
                }
                else
                {
                    /* Collision.  */
                    float invTimeStep = 1.0f / Simulator.Instance.TimeStep;

                    /* Vector from cutoff center to relative velocity. */
                    Vector3 w = relativeVelocity - invTimeStep * relativePosition;

                    float wLength = w.magnitude;
                    Vector3 unitW = w / wLength;

                    plane.Normal = unitW;
                    u = (combinedRadius * invTimeStep - wLength) * unitW;
                }

                plane.Point = Velocity + 0.5f * u;
                _orcaPlanes.Add(plane);
            }

            
            int planeFail = LinearProgram3(_orcaPlanes, MaxSpeed, PrefVelocity, false, ref _newVelocity);
            CheckForNan(_newVelocity);

            if (planeFail < _orcaPlanes.Count)
            {
                LinearProgram4(_orcaPlanes, planeFail, MaxSpeed, ref _newVelocity);
                CheckForNan(_newVelocity);
            }
        }

        /**
         * <summary>Inserts an agent neighbor into the set of neighbors of this
         * agent.</summary>
         *
         * <param name="agent">A pointer to the agent to be inserted.</param>
         * <param name="rangeSq">The squared range around this agent.</param>
         */
        internal void InsertAgentNeighbor(Agent agent, ref float rangeSq)
        {
            if (this != agent)
            {
                float distSq = (Position - agent.Position).sqrMagnitude;

                if (distSq < rangeSq)
                {
                    if (AgentNeighbors.Count < MaxNeighbors)
                    {
                        AgentNeighbors.Add(new KeyValuePair<float, Agent>(distSq, agent));
                    }

                    int i = AgentNeighbors.Count - 1;

                    while (i != 0 && distSq < AgentNeighbors[i - 1].Key)
                    {
                        AgentNeighbors[i] = AgentNeighbors[i - 1];
                        --i;
                    }

                    AgentNeighbors[i] = new KeyValuePair<float, Agent>(distSq, agent);

                    if (AgentNeighbors.Count == MaxNeighbors)
                    {
                        rangeSq = AgentNeighbors[AgentNeighbors.Count - 1].Key;
                    }
                }
            }
        }

        /**
         * <summary>Updates the two-dimensional position and two-dimensional
         * velocity of this agent.</summary>
         */
        internal void Update()
        {
            CheckForNan(_newVelocity);
            Velocity = _newVelocity;
            Position += Velocity * Simulator.Instance.TimeStep;
        }

        private static bool LinearProgram1(IList<Plane> planes, int planeId, Line line, float radius,
            Vector3 optVelocity, bool directionOpt, ref Vector3 result)
        {
            float dotProduct = Vector3.Dot(line.Point, line.Direction);
            float discriminant = dotProduct * dotProduct + radius * radius - line.Point.sqrMagnitude;

            if (discriminant < 0.0f)
            {
                /* Max speed circle fully invalidates line lineNo. */
                return false;
            }

            float sqrtDiscriminant = Mathf.Sqrt(discriminant);
            float tLeft = -dotProduct - sqrtDiscriminant;
            float tRight = -dotProduct + sqrtDiscriminant;

            for (int i = 0; i < planeId; ++i)
            {
                float numerator = Vector3.Dot(planes[planeId].Point - line.Point, planes[planeId].Normal);
                float denominator = Vector3.Dot(line.Direction, planes[i].Normal);

                if (Math.Abs(denominator) <= Mathf.Epsilon)
                {
                    /* Lines line is (almost) parallel to plane i. */
                    if (numerator < 0.0f)
                    {
                        return false;
                    }

                    continue;
                }

                float t = numerator / denominator;

                if (denominator >= 0.0f)
                {
                    /* Plane i bounds line on the left. */
                    tLeft = Math.Max(tLeft, t);

                }
                else
                {
                    /* Plane i bounds line on the right. */
                    tRight = Math.Min(tRight, t);
                }

                if (tLeft > tRight)
                {
                    return false;
                }
            }

            if (directionOpt)
            {
                /* Optimize direction. */
                if (Vector3.Dot(optVelocity, line.Direction) > 0.0f)
                {
                    /* Take right extreme. */
                    result = line.Point + tRight * line.Direction;
                    CheckForNan(result);
                }
                else
                {
                    /* Take left extreme. */
                    result = line.Point + tLeft * line.Direction;
                    CheckForNan(result);
                }
            }
            else
            {
                /* Optimize closest point. */
                float t = Vector3.Dot(line.Direction, optVelocity - line.Point);

                if (t < tLeft)
                {
                    result = line.Point + tLeft * line.Direction;
                    CheckForNan(result);
                }
                else if (t > tRight)
                {
                    result = line.Point + tRight * line.Direction;
                    CheckForNan(result);
                }
                else
                {
                    result = line.Point + t * line.Direction;
                    CheckForNan(result);
                }
            }

            return true;
        }

        private static bool LinearProgram2(IList<Plane> planes, int planeId, float radius, Vector3 optVelocity,
            bool directionOpt, ref Vector3 result)
        {
            float planeDist = Vector3.Dot(planes[planeId].Point, planes[planeId].Normal);
            float planeDistSq = planeDist * planeDist;
            float radiusSq = radius * radius;

            if (planeDistSq > radiusSq)
            {
                /* Max speed sphere fully invalidates plane planeId. */
                return false;
            }

            float planeRadiusSq = radiusSq - planeDistSq;

            Vector3 planeCenter = planeDist * planes[planeId].Normal;

            if (directionOpt)
            {
                /* Project direction optVelocity on plane planeNo. */
                Vector3 planeOptVelocity = optVelocity -
                                           Vector3.Scale(Vector3.Scale(optVelocity, planes[planeId].Normal),
                                               planes[planeId].Normal);
                float planeOptVelocityLengthSq = planeOptVelocity.sqrMagnitude;

                if (planeOptVelocityLengthSq <= Mathf.Epsilon)
                {
                    result = planeCenter;
                    CheckForNan(result);
                }
                else
                {
                    result = planeCenter +  Mathf.Sqrt(planeRadiusSq / planeOptVelocityLengthSq) *
                             planeOptVelocity;

                    CheckForNan(result);
                }
            }
            else
            {
                /* Project point optVelocity on plane planeNo. */
                result = optVelocity +
                         Vector3.Scale(Vector3.Scale((planes[planeId].Point - optVelocity), planes[planeId].Normal),
                             planes[planeId].Normal);

                CheckForNan(result);

                /* If outside planeCircle, project on planeCircle. */
                if (result.sqrMagnitude > radiusSq)
                {
                    Vector3 planeResult = result - planeCenter;
                    float planeResultLengthSq = planeResult.sqrMagnitude;
                    result = planeCenter + Mathf.Sqrt(planeRadiusSq / planeResultLengthSq) * planeResult;
                    CheckForNan(result);
                }
            }

            for (int i = 0; i < planeId; ++i)
            {
                if (Vector3.Dot(planes[i].Normal, planes[i].Point - result) > 0.0f)
                {
                    /* Result does not satisfy constraint i. Compute new optimal result. */
                    /* Compute intersection line of plane i and plane planeNo. */
                    Vector3 crossProduct = Vector3.Cross(planes[i].Normal, planes[planeId].Normal);

                    if (crossProduct.sqrMagnitude <= Mathf.Epsilon)
                    {
                        /* Planes planeNo and i are (almost) parallel, and plane i fully invalidates plane planeNo. */
                        return false;
                    }

                    Line line;
                    line.Direction = crossProduct.normalized;
                    Vector3 lineNormal = Vector3.Cross(line.Direction, planes[planeId].Normal);

                    var v1 = Vector3.Scale(planes[i].Point - planes[planeId].Point, planes[i].Normal);
                    var v2 = Vector3.Scale(Vector3.Scale(lineNormal, planes[i].Normal), lineNormal);
                    line.Point = planes[planeId].Point + new Vector3(v1.x / v2.x, v1.y / v2.y, v1.z / v2.z);

                    if (!LinearProgram1(planes, i, line, radius, optVelocity, directionOpt, ref result))
                    {
                        CheckForNan(result);
                        return false;
                    }
                }
            }

            return true;
        }

        private static int LinearProgram3(IList<Plane> planes, float radius, Vector3 optVelocity, bool directionOpt,
            ref Vector3 result)
        {
            if (directionOpt)
            {
                /* Optimize direction. Note that the optimization velocity is of unit length in this case. */
                result = optVelocity * radius;
                CheckForNan(result);
            }
            else if (optVelocity.sqrMagnitude > radius * radius)
            {
                /* Optimize closest point and outside circle. */
                result = optVelocity.normalized * radius;
                CheckForNan(result);
            }
            else
            {
                /* Optimize closest point and inside circle. */
                result = optVelocity;
                CheckForNan(result);
            }

            for (var i = 0; i < planes.Count; ++i)
            {
                if (Vector3.Dot(planes[i].Normal, planes[i].Point - result) > 0.0f)
                {
                    /* Result does not satisfy constraint i. Compute new optimal result. */
                    Vector3 tempResult = result;

                    if (!LinearProgram2(planes, i, radius, optVelocity, directionOpt, ref result))
                    {
                        result = tempResult;
                        CheckForNan(result);
                        return i;
                    }
                }
            }

            return planes.Count;
        }

        private void LinearProgram4(List<Plane> planes, int beginPlane, float radius, ref Vector3 result)
        {
            float distance = 0.0f;

            for (int i = beginPlane; i < planes.Count; ++i)
            {
                if (Vector3.Dot(planes[i].Normal, planes[i].Point - result) > distance)
                {
                    /* Result does not satisfy constraint of plane i. */
                    var projPlanes = new List<Plane>();

                    for (int j = 0; j < i; ++j)
                    {
                        Plane plane;

                        Vector3 crossProduct = Vector3.Cross(planes[j].Normal, planes[i].Normal);

                        if (crossProduct.sqrMagnitude <= Mathf.Epsilon)
                        {
                            /* Plane i and plane j are (almost) parallel. */
                            if (Vector3.Dot(planes[i].Normal, planes[j].Normal) > 0.0f)
                            {
                                /* Plane i and plane j point in the same direction. */
                                continue;
                            }
                            else
                            {
                                /* Plane i and plane j point in opposite direction. */
                                plane.Point = 0.5f * (planes[i].Point + planes[j].Point);
                            }
                        }
                        else
                        {
                            /* Plane.point is point on line of intersection between plane i and plane j. */
                            Vector3 lineNormal = Vector3.Cross(crossProduct, planes[i].Normal);
                            var v1 = Vector3.Scale(planes[j].Point - planes[i].Point, planes[j].Normal);
                            var v2 = Vector3.Scale(Vector3.Scale(lineNormal, planes[j].Normal), lineNormal);
                            plane.Point = planes[i].Point + new Vector3(v1.x / v2.x, v1.y / v2.y, v1.z / v2.z);
                        }

                        plane.Normal = (planes[j].Normal - planes[i].Normal).normalized;
                        projPlanes.Add(plane);
                    }

                    Vector3 tempResult = result;

                    if (LinearProgram3(projPlanes, radius, planes[i].Normal, true, ref result) < projPlanes.Count)
                    {
                        /* This should in principle not happen.  The result is by definition already in the feasible region of this linear program. If it fails, it is due to small floating point error, and the current result is kept. */
                        result = tempResult;
                        CheckForNan(result);
                    }

                    distance = Vector3.Dot(planes[i].Normal, planes[i].Point - result);
                }
            }
        }

        private static void CheckForNan(Vector3 vec)
        {
            if(float.IsNaN(vec.x) || float.IsNaN(vec.y) || float.IsNaN(vec.z))
                Debug.Log("!");
        }
    }
}
