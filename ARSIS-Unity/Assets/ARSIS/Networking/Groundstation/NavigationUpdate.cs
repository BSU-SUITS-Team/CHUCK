using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARSISEventSystem;
using UnityEngine.Networking;
using Newtonsoft.Json;


public class NaviagtionUpdate : MonoBehaviour
{
    private static string navigationEndpoint = "http://0.0.0.0:8181/navigation/";
    // Start is called before the first frame update
    void Start()
    {
        EventManager.AddListener<NavigationGet>(getNavigationTrigger);
    }


    void getNavigationTrigger(NavigationGet pg){
        StartCoroutine(getNavigation(pg));
    }
    IEnumerator getNavigation(NavigationGet navigationToGet){
        string name = navigationToGet.navigationPathName;
        Debug.Log(name);
        UnityWebRequest www = UnityWebRequest.Get(navigationEndpoint+name);
        yield return www.SendWebRequest();

        if (www.result != UnityWebRequest.Result.Success) {
            Debug.Log(www.error);
        }
        else {
            // Show results as text
            string resultString = www.downloadHandler.text;
            Debug.Log(resultString);
            NavigationEvent newNavigationEvent = JsonConvert.DeserializeObject<NavigationEvent>(resultString);
            Debug.Log("before trigger" + newNavigationEvent.name);
            EventManager.Trigger(newNavigationEvent);
        }
    }
}
