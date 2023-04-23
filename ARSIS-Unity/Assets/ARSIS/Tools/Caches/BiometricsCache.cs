using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARSISEventSystem;

public class BiometricsCache : MonoBehaviour
{

    public BiometricsEvent BiometricsEvent;
    private static int MINUTES = 5;
    private static int MAX_ENTRYS = 60*MINUTES;
    private List<BiometricsEvent> BiometricsList;
    public static BiometricsCache BiometricsCacheSingleton { get; private set; }
    private void Awake()
    {
        BiometricsList = new List<BiometricsEvent>();
        if (BiometricsCacheSingleton != null && BiometricsCacheSingleton != this)
        {
            Destroy(this);
            EventManager.RemoveListener<BiometricsEvent>(UpdateBiometrics);
        }
        else
        {
            BiometricsCacheSingleton = this;
            EventManager.AddListener<BiometricsEvent>(UpdateBiometrics);
        }
    }
    void UpdateBiometrics(BiometricsEvent be){
        BiometricsEvent = be;
        Debug.Log(BiometricsEvent.heartrate);
        if (BiometricsList.Count > MAX_ENTRYS){
            BiometricsList.RemoveAt(0);
        }
        BiometricsList.Add(be);
        Debug.Log(BiometricsList.Count);
    }

    public float getHeartrate(){
        float heartrate = 0;
        if(BiometricsEvent != null){
            heartrate = BiometricsEvent.heartrate;
        }
        return heartrate;
    }

    public string getBiometricsString(){
        return BiometricsEvent.heartrate.ToString();
    }

    public List<float> getHeartrateList{
        List<float> heartrateList = new List<float>()
        foreach (BiometricsEvent biometricsEvent in BiometricsList){
            heartrateList.append(biometricsEvent.heartrate);
        }
        return heartrateList;
    }

    public List<float> getO2List{
        List<float> o2List = new List<float>()
        foreach (BiometricsEvent biometricsEvent in BiometricsList){
            o2List.append(biometricsEvent.o2);
        }
        return o2List;
    }

    public List<float> getBatteryList{
        List<float> batteryList = new List<float>()
        foreach (BiometricsEvent biometricsEvent in BiometricsList){
            batteryList.append(biometricsEvent.battery);
        }
        return batteryList;
    }
}
