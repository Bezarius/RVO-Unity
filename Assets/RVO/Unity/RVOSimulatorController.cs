﻿using RVO.Core._2D;
using UnityEngine;

namespace RVO.Unity
{
    public class RVOSimulatorController : MonoBehaviour
    {
        [SerializeField]
        private bool _isMT;

        private void Awake()
        {
            Simulator.Instance.IsMultithreaded = _isMT;
        }

        private void Update()
        {
            Simulator.Instance.SetTimeStep(Time.deltaTime);
            Simulator.Instance.DoStep();
        }
    }
}