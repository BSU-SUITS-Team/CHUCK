using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using EventSystem;

public class ProceduresDisplay : MonoBehaviour
{
    // Start is called before the first frame update
    bool notShown;
    void Start()
    {

        notShown = true;
    }

    // Update is called once per frame
    void Update()
    {
        if (ProcedureCache.Instance.Count() > 0 && notShown){
            notShown = false;
            Debug.Log("Procedures not empty");
            ProcedureEvent pe = ProcedureCache.Instance.getProcedure("Mock Procedure");

            GameObject textGO = new GameObject();
            TextMeshProUGUI text = textGO.AddComponent<TextMeshProUGUI>();
            text.text = pe.taskList.Count.ToString();
            Task task0 = pe.taskList[0];
            foreach(Step s in task0.stepList){
                GameObject taskGO = new GameObject();
                taskGO.transform.SetParent(this.transform);
                if (s.type == "image"){
                    Image suits_image = taskGO.AddComponent<Image>();
                    byte[]  imageBytes = Convert.FromBase64String(s.body);
                    Texture2D tex = new Texture2D(2, 2);
                    tex.LoadImage( imageBytes );
                    Sprite sprite = Sprite.Create(tex, new Rect(0.0f, 0.0f, tex.width, tex.height), new Vector2(0.5f, 0.5f), 100.0f);
                    suits_image.sprite = sprite;
                }
                if (s.type == "text"){
                    TextMeshProUGUI newText = taskGO.AddComponent<TextMeshProUGUI>();
                    newText.text = s.body;
                }
                Debug.Log(s.type);
            }
        }
    }
}
