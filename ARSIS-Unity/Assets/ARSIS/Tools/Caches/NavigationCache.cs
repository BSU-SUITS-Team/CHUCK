using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARSISEventSystem;

public class NavigationCache : MonoBehaviour
{
    public static NavigationCache Instance { get; private set; }
    private Dictionary<string, NavigationEvent> navigationCache;
    private WaitForSeconds navigationPollingDelay = new WaitForSeconds(1.0f);
    public int numberOfNavigation = 0;

    void Start()
    {
        EventManager.AddListener<NavigationEvent>(proccessNavigationEvent);
        EventManager.AddListener<NavigationDictionary>(proccessNavigationDictionary);
        navigationCache = new Dictionary<string, NavigationEvent>();
        StartCoroutine(StartNavigationCache());
    }

    private void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Destroy(this);
        }
        else
        {
            Instance = this;
        }
    }

    public int Count(){
        return navigationCache.Count;
    }

    public NavigationEvent getNavigation(string name){
        //return navigationCache.GetValueOrDefault(name, null);
        return null;
    }

    IEnumerator StartNavigationCache() {
        while(navigationCache.Count < 1){
            StartCoroutine(updateNavigation());
            yield return navigationPollingDelay;
        }
    }

    //TODO this should select an actual navigation
    IEnumerator fetchNavigation(string navigationName){
        NavigationGet pg = new NavigationGet(navigationName);
        EventManager.Trigger(pg);
        yield return null;
    }

    IEnumerator updateNavigation(){
        UpdateNavigationEvent up = new UpdateNavigationEvent();
        EventManager.Trigger(up);
        yield return null;
    }

    void proccessNavigationDictionary(NavigationDictionary pd){
        if (pd.navigationDictionary == null){
            return;
        }
        foreach(KeyValuePair<string, NavigationEvent> entry in pd.navigationDictionary){
            navigationCache[entry.Key] = entry.Value;
        }
        numberOfNavigation = Count();
    }


    void proccessNavigationEvent(NavigationEvent pe){
        navigationCache[pe.name] = pe;
        numberOfNavigation = Count();
    }

    Dictionary<string, NavigationEvent> getNavigation(){
        return new Dictionary<string, NavigationEvent>(navigationCache);
    }

    List<string> getNavigationNames(){
        return new List<string>(navigationCache.Keys);
    }
}
