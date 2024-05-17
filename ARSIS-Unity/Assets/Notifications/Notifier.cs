using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class Notifier : MonoBehaviour
{
    public static Notifier instance;
    public GameObject notificationText;
    public GameObject timeStamp;
    public Image colorBand;
    public GameObject NotificationObj;
    void Start()
    {
        if(instance != null)
        {
            Destroy(gameObject);
        }
        else
        {
            instance = this;
        }
    }

    public void NotifyRed(string message)
    {
        GameObject newNotification = Instantiate(notificationText);
        newNotification.GetComponent<RectTransform>().SetParent(instance.transform);
        newNotification.GetComponent<RectTransform>().localScale = Vector3.one;
        newNotification.GetComponent<TextMeshProUGUI>().text = message;
        
        GameObject newTimeStamp = Instantiate(timeStamp);
        newTimeStamp.GetComponent<RectTransform>().SetParent(instance.transform);
        newTimeStamp.GetComponent<RectTransform>().localScale = Vector3.one;
        newTimeStamp.GetComponent<TextMeshProUGUI>().text = message;

        colorBand.color = Color.red;
    }

    public void DestroyGameObject()
    {
        Destroy(NotificationObj);
    }
}
