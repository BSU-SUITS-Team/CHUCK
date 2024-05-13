using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARSISEventSystem;
using UnityEngine.Networking;
using Newtonsoft.Json;
using System.Net.Http;
using System.Threading.Tasks;


public class ProcedureUpdate : MonoBehaviour
{
    private static string procedureEndpoint = "http://127.0.0.1:8181/procedures/";
    private readonly HttpClient httpClient = new HttpClient();
    // Start is called before the first frame update
    void Start()
    {
        EventManager.AddListener<ProcedureGet>(getProcedureTrigger);
        EventManager.AddListener<UpdateProceduresEvent>(updateProceduresTrigger);
    }

    // TODO: Make this something that deals with generics
    void updateProceduresTrigger(UpdateProceduresEvent up){
        Debug.Log("update procedures trigger");
        Task updateTask = Task.Run(() => updateProcedures());
        updateTask.GetAwaiter().GetResult();
    }

    public async Task updateProcedures(){
        var response = await httpClient.GetAsync(procedureEndpoint);
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
            Dictionary<string, ProcedureEvent> dictOnly = JsonConvert.DeserializeObject<Dictionary<string, ProcedureEvent>>(resultString);
            ProcedureDictionary newProcedureDictionary = new ProcedureDictionary(dictOnly);
            EventManager.Trigger(newProcedureDictionary);
        }
    }

    // TODO ASYNC THIS
    void getProcedureTrigger(ProcedureGet pg){
        StartCoroutine(getProcedure(pg).GetEnumerator());
    }

    IEnumerable getProcedure(ProcedureGet procedureToGet){
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
