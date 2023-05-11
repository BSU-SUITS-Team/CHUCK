using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class minimap_tag : MonoBehaviour
{

    private Vector2 xyCoords = new Vector2 (0,0);

    void Update () {
        this.gameObject.transform.position = this.gameObject.transform.parent.position;
        //this.gameObject.transform.localPosition += new Vector3 (xyCoords.x, xyCoords.y, 0);
    }

    public void updateXY (Vector2 v2) {
        xyCoords = v2;
    }

    public void updateText (string s) {
        this.gameObject.GetComponent<TextMeshPro>().SetText(s);
    }

    public void hide () {
        this.gameObject.SetActive(false);
    }

    public void show () {
        this.gameObject.SetActive(true);
    }


}
