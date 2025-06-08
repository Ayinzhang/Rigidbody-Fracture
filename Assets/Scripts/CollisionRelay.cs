using UnityEngine;

public class CollisionRelay : MonoBehaviour
{
    void OnCollisionEnter(Collision collision)
    {
        RigidbodyFracture fracture = GetComponentInParent<RigidbodyFracture>();
        if (fracture != null) fracture.OnCollisionEnter(collision);
    }
}
