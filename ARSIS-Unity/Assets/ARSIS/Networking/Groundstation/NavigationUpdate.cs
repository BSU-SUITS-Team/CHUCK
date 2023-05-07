using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARSISEventSystem;
using UnityEngine.Networking;
using Newtonsoft.Json;


public class NavigationUpdate : MonoBehaviour
{
    private static string navigationEndpoint = "http://0.0.0.0:8181/navigation/";
    // Start is called before the first frame update
    void Start()
    {
        EventManager.AddListener<NavigationGet>(getNavigationTrigger);
        EventManager.AddListener<UpdateNavigationEvent>(updateNavigationTrigger);
    }

    void updateNavigationTrigger(UpdateNavigationEvent up){
        StartCoroutine(updateNavigation());
    }

    IEnumerator updateNavigation(){
        UnityWebRequest www = UnityWebRequest.Get(navigationEndpoint);
        yield return www.SendWebRequest();

        if (www.result != UnityWebRequest.Result.Success) {
            Debug.Log(www.error);
        }
        else {
            // Show results as text
            string resultString = www.downloadHandler.text;
            Dictionary<string, NavigationEvent> dictOnly = JsonConvert.DeserializeObject<Dictionary<string, NavigationEvent>>(resultString);
            NavigationDictionary newNavigationDictionary = new NavigationDictionary(dictOnly);
            EventManager.Trigger(newNavigationDictionary);
        }
    }

    void getNavigationTrigger(NavigationGet pg){
        StartCoroutine(getNavigation(pg));
    }
    IEnumerator getNavigation(NavigationGet navigationToGet){
        string name = navigationToGet.navigationPathName;
        UnityWebRequest www = UnityWebRequest.Get(navigationEndpoint+name);
        yield return www.SendWebRequest();

        if (www.result != UnityWebRequest.Result.Success) {
            Debug.Log(www.error);
        }
        else {
            // Show results as text
            string resultString = www.downloadHandler.text;
            NavigationEvent newNavigationEvent = JsonConvert.DeserializeObject<NavigationEvent>(resultString);
            EventManager.Trigger(newNavigationEvent);
        }
    }
}
