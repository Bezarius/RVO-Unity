using RVO.Core._3D;
using UnityEngine;

namespace RVO.Unity
{
    public class NavAgent3D : CachedBehaviour
    {
        private int _agentId;

        [SerializeField] private float _neighborDist = 5f;

        [SerializeField] private int _maxNeighbors = 10;

        [SerializeField] private float _timeHorizon = 5f;

        [SerializeField] private float _timeHorizonObst = 5f;

        [SerializeField] private float _radius = 1f;

        [SerializeField] private float _maxSpeed = 4;

        [SerializeField] private Vector3 _velocity;

        private Vector3 _targetPosition;

        public Vector3 TargetPosition
        {
            get { return _targetPosition; }
            set
            {
                _disableRVO = false;
                _targetPosition = value;
            }
        }

        private bool _disableRVO;

        private Vector3 _preferredVelocity;

        private const int RandMax = 0x7fff;

        protected override void Awake()
        {
            base.Awake();

            TargetPosition = new Vector3(Random.Range(-100, 100), this.transform.position.y, Random.Range(-100, 100));

            _agentId = Simulator.Instance.AddAgent(
                this.transform.position,
                _neighborDist,
                _maxNeighbors,
                _timeHorizon,
                _timeHorizonObst,
                _radius,
                _maxSpeed,
                _velocity
            );
        }

        protected void Update()
        {
            if (Vector3.Distance(this.transform.position, _targetPosition) > _radius * 2)
            {
                _velocity = _preferredVelocity;

                if (!_disableRVO)
                    _velocity = Simulator.Instance.GetAgentVelocity(_agentId);

                if(float.IsNaN(_velocity.x) || float.IsNaN(_velocity.y) || float.IsNaN(_velocity.z))
                    return; // todo: fix this posibility

                // Apply velocity on transform, this is where you multiply by dt (velocity is in m/s)
                transform.localPosition += _velocity * Time.deltaTime;

                // Face same direction as velocity
                if (_velocity.magnitude > 0.1f)
                {
                    Quaternion targetRotation = Quaternion.LookRotation(_velocity);
                    var factor = Time.deltaTime < float.Epsilon
                        ? 0
                        : Time.deltaTime / (Time.deltaTime + 0.1f); // 0.0f = snap, +infinity = don't move
                    if (targetRotation != transform.rotation)
                        transform.localRotation = Quaternion.Slerp(transform.localRotation, targetRotation, factor);
                }

                // Done after position update, before RVO
                UpdatePreferredVelocity();
            }
            else
            {
                _disableRVO = true;
            }

            // Update agent parameters for next frame calculations
            // There may be a 1 frame lag depending on when Simulator is updated
            Simulator.Instance.SetAgentPrefVelocity(_agentId, _preferredVelocity);
            Simulator.Instance.SetAgentPosition(_agentId, transform.localPosition);
        }

        public void OnDrawGizmos()
        {
            if (transform)
            {
                Vector3 position = transform.position;
                Gizmos.color = Color.green;
                Gizmos.DrawLine(position, position + _preferredVelocity);
                Gizmos.color = Color.red;
                Gizmos.DrawLine(position, position + _velocity);
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireSphere(position, _radius);
            }
        }

        private void UpdatePreferredVelocity()
        {
            var currentPosition = transform.position;

            var direction = (TargetPosition - currentPosition).normalized;
            Quaternion targetPosRotation = Quaternion.LookRotation(direction);

            _preferredVelocity = targetPosRotation * Vector3.forward * _maxSpeed;

            // Not sure if this is necessary
            float angle = Random.Range(0, RandMax) * 2.0f * Mathf.PI / RandMax;
            float dist = Random.Range(0, RandMax) * 0.0001f / RandMax;

            _preferredVelocity += dist * new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));
        }
    }
}