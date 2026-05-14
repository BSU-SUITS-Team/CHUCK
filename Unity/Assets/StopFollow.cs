using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MixedReality.Toolkit.SpatialManipulation;

public class StopFollow : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        StartCoroutine(StopFollowing());
    }

    private IEnumerator StopFollowing()
    {
        yield return new WaitForSeconds(0.1f);
        Follow followObject = GetComponent<Follow>();
        if (followObject != null)
        {
            followObject.enabled = false;
        }
    }
}
