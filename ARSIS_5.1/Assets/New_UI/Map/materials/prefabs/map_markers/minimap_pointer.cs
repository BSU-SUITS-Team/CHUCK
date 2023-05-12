using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class minimap_pointer : MonoBehaviour
{
    void Start () {
        this.gameObject.transform.position = new Vector3(0,0,0);
    }

    public void hide () {
        this.gameObject.SetActive(false);
    }

    public void show () {
        this.gameObject.SetActive(true);
    }

    public void point (GameObject go) {
        this.gameObject.transform.LookAt(go.transform);
        this.gameObject.transform.localEulerAngles += new Vector3 (-90, 0, 0);
    }
}
