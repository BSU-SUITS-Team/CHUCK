using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NotificationButton : MonoBehaviour
{
    public string relatedProcedureName;

    public void PressButton()
    {
        Procedures parentProcedures = transform.GetComponentInParent<Procedures>();
        parentProcedures.CreateProcedureDisplay(parentProcedures.FindProcedureByName(relatedProcedureName));
    }
}
