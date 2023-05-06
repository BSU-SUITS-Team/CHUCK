using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARSISEventSystem;
using UnityEngine.Networking;
using Newtonsoft.Json;


public class ProcedureUpdate : MonoBehaviour
{
    private static string procedureEndpoint = "http://0.0.0.0:8181/procedures/";
    // Start is called before the first frame update
    void Start()
    {
        EventManager.AddListener<ProcedureGet>(getProcedureTrigger);
        EventManager.AddListener<UpdateProceduresEvent>(updateProceduresTrigger);
    }

    void updateProceduresTrigger(UpdateProceduresEvent up){
        StartCoroutine(updateProcedures());
    }

    IEnumerator updateProcedures(){
        UnityWebRequest www = UnityWebRequest.Get(procedureEndpoint);
        yield return www.SendWebRequest();

        if (www.result != UnityWebRequest.Result.Success) {
            Debug.Log(www.error);
        }
        else {
            // Show results as text
            string resultString = www.downloadHandler.text;

            Debug.Log(resultString);
            Dictionary<string, ProcedureEvent> dictOnly = JsonConvert.DeserializeObject<Dictionary<string, ProcedureEvent>>(resultString);
            ProcedureDictionary newProcedureDictionary = new ProcedureDictionary(dictOnly);
            Debug.Log(newProcedureDictionary.procedureDictionary);
            EventManager.Trigger(newProcedureDictionary);
        }
    }
    void getProcedureTrigger(ProcedureGet pg){
        StartCoroutine(getProcedure(pg));
    }

    IEnumerator getProcedure(ProcedureGet procedureToGet){
        string name = procedureToGet.procedureName;
        UnityWebRequest www = UnityWebRequest.Get(procedureEndpoint+name);
        yield return www.SendWebRequest();

        if (www.result != UnityWebRequest.Result.Success) {
            Debug.Log(www.error);
        }
        else {
            // Show results as text
            string resultString = www.downloadHandler.text;
            ProcedureEvent newProcedureEvent = JsonConvert.DeserializeObject<ProcedureEvent>(resultString);
            EventManager.Trigger(newProcedureEvent);
        }
    }
}
