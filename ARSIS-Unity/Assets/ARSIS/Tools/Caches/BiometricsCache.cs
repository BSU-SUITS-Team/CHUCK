using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARSISEventSystem;

public class BiometricsCache : MonoBehaviour
{

    public BiometricsEvent biometricsEvent;
    private static int MINUTES = 5;
    private static int MAX_ENTRYS = 60*MINUTES;
    private List<BiometricsEvent> BiometricsList;
    public static BiometricsCache BiometricsCacheSingleton { get; private set; }
    public int Count;
    public int OutOfRangeCount;
    public HashSet<string> outOfRangeBiometrics;
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
            outOfRangeBiometrics = new HashSet<string>();
        }
    }
    void UpdateBiometrics(BiometricsEvent be){
        biometricsEvent = be;
        CheckBiometrics(biometricsEvent);
        if (BiometricsList.Count > MAX_ENTRYS){
            BiometricsList.RemoveAt(0);
        }
        BiometricsList.Add(be);
        Count = BiometricsList.Count;
        OutOfRangeCount = outOfRangeBiometrics.Count;
    }

    void CheckBiometrics(BiometricsEvent be){

        foreach((int rate, int[] limits, string name) in be.intCheckList){
            if(rate < limits[0] || rate > limits[1]){
                outOfRangeBiometrics.Add(name);
            }
        }
        foreach((bool status, bool nominal, string name) in be.boolCheckList){
            if(status != nominal){
                outOfRangeBiometrics.Add(name);
            }
        }
    }

    public float getHeartrate(){
        float heartrate = 0;
        if(biometricsEvent != null){
            heartrate = biometricsEvent.heartrate;
        }
        return heartrate;
    }

    public List<float> getHeartrateList(){
        List<float> heartrateList = new List<float>();
        foreach (BiometricsEvent biometricsEvent in BiometricsList){
            heartrateList.Add(biometricsEvent.heartrate);
        }
        return heartrateList;
    }

    public List<float> getO2List(){
        List<float> o2List = new List<float>();
        foreach (BiometricsEvent biometricsEvent in BiometricsList){
            o2List.Add(biometricsEvent.o2);
        }
        return o2List;
    }

    public List<float> getBatteryList(){
        List<float> batteryList = new List<float>();
        foreach (BiometricsEvent biometricsEvent in BiometricsList){
            batteryList.Add(biometricsEvent.battery);
        }
        return batteryList;
    }
    public void acknowledgeOutOfRange(string name){
        outOfRangeBiometrics.Remove(name);
    }
}
