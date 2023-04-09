using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARSISEventSystem;

public class BiometricsCache : MonoBehaviour
{

    public BiometricsEvent BiometricsEvent;
    private static int MINUTES = 5;
    private static int MAX_ENTRYS = 60*MINUTES;
    private List<BiometricsEvent> BiometricsList;eee
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
}
