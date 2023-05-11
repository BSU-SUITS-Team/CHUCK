using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.UI;
using System;

public class ProceduresMenu : MonoBehaviour
{
    void Start()
    {
        ListManager list = this.GetComponent<ListManager>(); 
        List<Procedure> procedures = TaskManager.S.allProcedures;
        foreach (Procedure p in procedures)
        {
           if (p.emergency) return; 
           for (int j = 0; j < p.Tasks.Length; j++)
           {
                GameObject listItem = list.addListItem(p.Tasks[j].Title);

                Interactable interact = listItem.GetComponent<Interactable>();
                int taskNum = j; 
                interact.OnClick.AddListener(() =>
                {
                    MenuController.s.currentProcedure = 0; // TODO: should be actual procedure number if we had more than one
                    MenuController.s.currentTask = taskNum;
                    MenuController.s.currentSubTask = 0;
                    VoiceManager.S.generateTaskMenu();
                });  
           }

            
        }
    }
}
