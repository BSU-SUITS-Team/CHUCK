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
        Debug.Log("asdfasdf");
        StartCoroutine(updateNavigation());
    }

    IEnumerator updateNavigation(){
        Debug.Log("asdf");
        UnityWebRequest www = UnityWebRequest.Get(navigationEndpoint);
        yield return www.SendWebRequest();

        if (www.result != UnityWebRequest.Result.Success) {
            Debug.Log(www.error);
        }
        else {
            // Show results as text
            string resultString = www.downloadHandler.text;
            Debug.Log(resultString);
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
