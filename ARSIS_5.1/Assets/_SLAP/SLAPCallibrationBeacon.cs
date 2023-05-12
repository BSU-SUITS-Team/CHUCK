using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;

public class SLAPCallibrationBeacon : MonoBehaviour
{
    public SLAPManager sm;

    void Start ()
    {
        sm = SLAPManager.instance;
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag == "MainCamera")
        {
            CallibrateAsync();
        }
    }

    async void CallibrateAsync ()
    {
        await SLAPUtils.CallibrateSecondPosAsync(transform.position);
        Destroy(this.gameObject);
    }
}