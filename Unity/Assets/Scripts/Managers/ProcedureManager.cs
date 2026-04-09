using ARSIS.EventManager;
using System.Collections.Generic;
using UnityEngine;

public class ProcedureManager : MonoBehaviour
{
    private static ProcedureManager instance;
    private Dictionary<string, ProcedureRoot> procedureCache = new();
    private const string PROCEDURES_PATH = "Procedures"; // Resources/Procedures/

    public static ProcedureManager Instance
    {
        get
        {
            if (instance == null)
            {
                instance = FindObjectOfType<ProcedureManager>();
                if (instance == null)
                {
                    GameObject obj = new GameObject("ProcedureManager");
                    instance = obj.AddComponent<ProcedureManager>();
                }
            }
            return instance;
        }
    }

    void Awake()
    {
        if (instance == null)
        {
            instance = this;
            DontDestroyOnLoad(gameObject);
            LoadAllProcedures();
        }
        else if (instance != this)
        {
            Destroy(gameObject);
        }
    }

    /// <summary>
    /// Loads all JSON procedure files from Resources/Procedures folder
    /// </summary>
    public void LoadAllProcedures()
    {
        procedureCache.Clear();
        
        TextAsset[] jsonFiles = Resources.LoadAll<TextAsset>(PROCEDURES_PATH);
        
        Debug.Log($"Found {jsonFiles.Length} procedure files in Resources/Procedures/");
        
        foreach (TextAsset json in jsonFiles)
        {
            try
            {
                ProcedureRoot procedure = JsonUtility.FromJson<ProcedureRoot>(json.text);
                if (procedure != null && !string.IsNullOrEmpty(procedure.name))
                {
                    procedureCache[procedure.name] = procedure;
                    Debug.Log($"Loaded procedure: {procedure.name}");
                }
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Failed to load procedure '{json.name}': {e.Message}");
            }
        }
        
        Debug.Log($"Successfully loaded {procedureCache.Count} procedures");
    }

    /// <summary>
    /// Get a specific procedure by name
    /// </summary>
    public ProcedureRoot GetProcedure(string procedureName)
    {
        if (procedureCache.TryGetValue(procedureName, out var procedure))
        {
            return procedure;
        }
        
        Debug.LogWarning($"Procedure '{procedureName}' not found in cache");
        return null;
    }

    /// <summary>
    /// Get all loaded procedures
    /// </summary>
    public List<ProcedureRoot> GetAllProcedures()
    {
        return new List<ProcedureRoot>(procedureCache.Values);
    }

    /// <summary>
    /// Check if a procedure exists by name
    /// </summary>
    public bool HasProcedure(string procedureName)
    {
        return procedureCache.ContainsKey(procedureName);
    }

    /// <summary>
    /// Get the count of loaded procedures
    /// </summary>
    public int GetProcedureCount()
    {
        return procedureCache.Count;
    }
}
