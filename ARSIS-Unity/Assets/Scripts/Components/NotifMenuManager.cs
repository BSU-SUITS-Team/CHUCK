using System.Collections.Generic;
using UnityEngine;
using TMPro;
using UnityEngine.UI;
using ARSIS.EventManager;

public class NotifMenuManager : MonoBehaviour
{
    public GameObject miniNotifPrefab; // Reference to the miniNotif prefab
    public Transform notificationContainer; // Reference to the container for miniNotif instances

    private List<BaseArsisEvent> notifications = new List<BaseArsisEvent>();

    public void AddNotification(BaseArsisEvent notification)
    {
        notifications.Add(notification);
        DisplayNotification(notification);
    }

    private void DisplayNotification(BaseArsisEvent notification)
    {
        // Check if the notification is of type Notifications
        if (notification.GetType() == typeof(Notifications))
        {
            ARSIS.EventManager.Notifications notificationData = (ARSIS.EventManager.Notifications)notification;
            NotificationsData data = notificationData.data;

            // Instantiate the prefab dynamically
            GameObject notifInstance = Instantiate(miniNotifPrefab, notificationContainer);
            notifInstance.transform.Find("Content").GetComponent<TextMeshProUGUI>().text = data.content;
            notifInstance.transform.Find("DateTime").GetComponent<TextMeshProUGUI>().text = data.time.ToString(); // Assuming 'time' is a timestamp or a similar integer
            notifInstance.transform.Find("SeverityIndicator").GetComponent<Image>().color = GetSeverityColor(data.severity);
        }
    }

    private Color GetSeverityColor(int severity)
    {
        // Implement logic to return color based on severity
        switch (severity)
        {
            case 2: return Color.green;
            case 1: return Color.yellow;
            case 0: return Color.red;
            default: return Color.white;
        }
    }
}
