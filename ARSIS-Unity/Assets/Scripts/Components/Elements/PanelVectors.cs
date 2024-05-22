using System.Collections;
using System.Collections.Generic;
//using Unity.Mathematics;
using UnityEngine;

public class PanelVectors : MonoBehaviour
{
    Rigidbody _rbMe;
    private float thrust = 1;
    private float zed = 0;

    private void Start()
    {
        _rbMe = GetComponent<Rigidbody>();
    }

    private void OnTriggerStay(Collider other)
    {
       // Debug.Log(other.transform.name);
        if (other.transform.name == "Plate")
        {
            Vector3 myrandomVector = Random.insideUnitSphere;
            myrandomVector.z = zed;
            Vector3 distance1 = transform.position - other.transform.position + myrandomVector;
            //distance1 = Quaternion.Euler(0, 90, 0) * distance1;
            distance1.z = zed;
            distance1.y = zed;
            distance1 = distance1.normalized;
            Debug.Log(distance1);
            //_rbMe.AddForce(distance1);
            transform.position += distance1*0.4f;
        }
    }

    private void Update()
    {
        _rbMe.velocity *= 0.2f * Time.deltaTime;
    }
}
