using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARSISEventSystem;

public class BiometricsCache : MonoBehaviour
{

    public BiometricsEvent BiometricsEvent;
    public static BiometricsCache BiometricsCacheSingleton { get; private set; }
    private void Awake()
    {
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
