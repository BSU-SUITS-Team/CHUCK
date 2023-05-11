using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class minimap_icon : MonoBehaviour
{
    public void changeHeight (float f) {
        transform.position = new Vector3 (0, f, 0);
    }
}
