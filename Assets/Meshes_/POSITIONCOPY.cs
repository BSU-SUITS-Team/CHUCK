using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class POSITIONCOPY : MonoBehaviour
{
    public GameObject target;
    private Vector3 offset;
    // Start is called before the first frame update
    public void Start() {
        offset = target.transform.position - transform.position;
    }
    // Update is called once per frame
    void Update()
    {
        transform.position = target.transform.position + offset;
    }
}
