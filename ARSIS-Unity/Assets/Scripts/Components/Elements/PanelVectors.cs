using System.Collections;
using System.Collections.Generic;
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
        _rbMe.AddForce(0,1*thrust,zed);
    }
}
