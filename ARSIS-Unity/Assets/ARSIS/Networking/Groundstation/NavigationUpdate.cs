using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARSISEventSystem;
using UnityEngine.Networking;
using Newtonsoft.Json;
using System.Net.Http;
using System.Threading.Tasks;

public class NavigationUpdate : MonoBehaviour
{
    private static string navigationEndpoint = "http://127.0.0.1:8181/navigation/";
    private readonly HttpClient httpClient = new HttpClient();
    // Start is called before the first frame update
    void Start()
    {
        EventManager.AddListener<NavigationGet>(getNavigationTrigger);
        EventManager.AddListener<UpdateNavigationEvent>(updateNavigationTrigger);
    }

    // TODO: Make this something that deals with generics
    void updateNavigationTrigger(UpdateNavigationEvent up){
        Debug.Log("updateNavigationTrigger");
        Task updateTask = Task.Run(() => updateNavigation());
        updateTask.GetAwaiter().GetResult();
    }

    public async Task updateNavigation(){
        var response = await httpClient.GetAsync(navigationEndpoint);
        var content = await response.Content.ReadAsStringAsync();
        /* UnityWebRequest www = UnityWebRequest.Get(navigationEndpoint); */
        /* yield return www.SendWebRequest(); */

        if (response.StatusCode != System.Net.HttpStatusCode.OK) {
            Debug.Log(response.StatusCode);
        }
        else {
            // Show results as text
            string resultString = content;
            Debug.Log(resultString);
            Dictionary<string, NavigationEvent> dictOnly = JsonConvert.DeserializeObject<Dictionary<string, NavigationEvent>>(resultString);
            NavigationDictionary newNavigationDictionary = new NavigationDictionary(dictOnly);
            EventManager.Trigger(newNavigationDictionary);
        }
    }

    // TODO ASYNC THIS
    void getNavigationTrigger(NavigationGet pg){
        StartCoroutine(getNavigation(pg).GetEnumerator());
    }
    IEnumerable getNavigation(NavigationGet navigationToGet){
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
