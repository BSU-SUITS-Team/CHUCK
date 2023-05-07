using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARSISEventSystem;

public class ProcedureCache : MonoBehaviour
{
    public static ProcedureCache Instance { get; private set; }
    private Dictionary<string, ProcedureEvent> procedureCache;
    private WaitForSeconds procedurePollingDelay = new WaitForSeconds(1.0f);
    public int numberOfProcedures = 0;

    void Start()
    {
        EventManager.AddListener<ProcedureEvent>(proccessProcedureEvent);
        EventManager.AddListener<ProcedureDictionary>(proccessProcedureDictionary);
        procedureCache = new Dictionary<string, ProcedureEvent>();
        StartCoroutine(StartProcedureCache());
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
        return procedureCache.Count;
    }

    public ProcedureEvent getProcedure(string name){
        return procedureCache.GetValueOrDefault(name, null);
    }

    IEnumerator StartProcedureCache() {
        while(procedureCache.Count < 1){
            StartCoroutine(updateProcedures());
            yield return procedurePollingDelay;
        }
    }

    //TODO this should select an actual procedure
    IEnumerator fetchProcedure(string procedureName){
        ProcedureGet pg = new ProcedureGet(procedureName);
        EventManager.Trigger(pg);
        yield return null;
    }

    IEnumerator updateProcedures(){
        UpdateProceduresEvent up = new UpdateProceduresEvent();
        EventManager.Trigger(up);
        yield return null;
    }

    void proccessProcedureDictionary(ProcedureDictionary pd){
        if (pd.procedureDictionary == null){
            return;
        }
        /* foreach(KeyValuePair<string, ProcedureEvent> entry in pd.procedureDictionary){ */
        /*     procedureCache[entry.Key] = entry.Value; */
        /* } */
        procedureCache = pd.procedureDictionary;
        numberOfProcedures = Count();
    }


    void proccessProcedureEvent(ProcedureEvent pe){
        procedureCache[pe.name] = pe;
        numberOfProcedures = Count();
    }

    Dictionary<string, ProcedureEvent> getProcedures(){
        return new Dictionary<string, ProcedureEvent>(procedureCache);
    }

    List<string> getProcedureNames(){
        return new List<string>(procedureCache.Keys);
    }
}
