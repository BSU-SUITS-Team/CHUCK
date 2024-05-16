using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PopupMenuManager : MonoBehaviour
{
    public GameObject notificationObject;

    public void DestroyObject()
    {
        Destroy(notificationObject);
    }
}
