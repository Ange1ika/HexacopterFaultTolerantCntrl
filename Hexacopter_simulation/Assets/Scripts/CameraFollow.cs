using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform target;
    public Vector3   offset = new Vector3(0, 4, -4);
    public float     smoothSpeed = 5f;

    void LateUpdate()
    {
        if (target == null)
        {
            var drone = GameObject.Find("Hexacopter");
            if (drone) target = drone.transform;
            return;
        }

        Vector3 desired = target.position + offset;
        transform.position = Vector3.Lerp(
            transform.position, desired, Time.deltaTime * smoothSpeed);
        transform.LookAt(target.position); //+ Vector3.up * 2f
    }
}

