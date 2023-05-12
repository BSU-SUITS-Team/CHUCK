using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class minimap_follow_player : MonoBehaviour
{

    public GameObject player;
    private Vector3 pRotat;

    void Update()
    {
        // // rotate
        // pRotat = player.transform.eulerAngles;
        // transform.eulerAngles = new Vector3 (0f, pRotat.y, 0f);

        // translate
        transform.position = player.transform.position;
    }
}
