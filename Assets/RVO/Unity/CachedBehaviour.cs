using UnityEngine;

// ReSharper disable once CheckNamespace
namespace RVO.Unity
{
    public class CachedBehaviour : MonoBehaviour
    {

        [HideInInspector]
        public new Transform transform;

        protected virtual void Awake()
        {
            transform = this.gameObject.transform;
        }
    }
}
