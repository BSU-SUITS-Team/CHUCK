// using ARSIS.EventManager;
// using ARSIS.UI;
// using MixedReality.Toolkit.Experimental;
// using MixedReality.Toolkit.UX;
// using MixedReality.Toolkit.UX.Experimental;
// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;

// public class Procedures : MonoBehaviour, IRenderable
// {
//     [SerializeField] GameObject procedureButton;
//     [SerializeField] ScrollArea scrollArea;
//     [SerializeField] GameObject procedureDisplay;
//     [SerializeField] GameObject summaryTimeline;
//     private List<ProcedureRoot> procedures = new List<ProcedureRoot>();
//     private bool initialized = false;

//     void CreateProcedureDisplay(ProcedureRoot procedureRoot)
//     {
//         // Wrap ProcedureRoot in a Procedure event object for ProcedureDisplay
//         Procedure procedure = new Procedure
//         {
//             type = "procedure",
//             data = procedureRoot,
//             label = procedureRoot.name
//         };

//         GameObject display = Instantiate(procedureDisplay); // procedureDisplay prefab is active = false by default
//         ProcedureDisplay view = display.GetComponent<ProcedureDisplay>();
//         view.SetProcedure(procedure); // apply the procedure
//         display.SetActive(true); // enable after procedure is applied
//     }

//     public void ShowSummaryTimeline()
//     {
//         Instantiate(summaryTimeline);
//     }

//     void Start()
//     {
//         // Load procedures from ProcedureManager instead of EventDatastore
//         ProcedureManager manager = ProcedureManager.Instance;
//         procedures = manager.GetAllProcedures();
//         initialized = true;
//         RenderProcedures();
//     }

//     void Update()
//     {
//         if (!initialized) return;
//         // No need for frame-by-frame updates anymore since procedures are loaded once in Start
//     }

//     /// <summary>
//     /// Render all loaded procedures as buttons in the scroll area
//     /// </summary>
//     private void RenderProcedures()
//     {
//         List<GameObject> entries = new();
        
//         foreach (ProcedureRoot procedureRoot in procedures)
//         {
//             GameObject entry = Instantiate(procedureButton);
//             Button button = entry.GetComponent<Button>();
//             button.SetText(procedureRoot.name);
//             PressableButton pressableButton = button.GetPressableButton();
            
//             // Create closure to capture the procedure root
//             ProcedureRoot procCopy = procedureRoot;
//             pressableButton.OnClicked.AddListener(() => CreateProcedureDisplay(procCopy));
            
//             entries.Add(entry);
//         }
        
//         scrollArea.SetEntries(entries);
//         Debug.Log($"Rendered {entries.Count} procedure buttons");
//     }

//     /// <summary>
//     /// IRenderable compatibility - can be used to refresh from EventDatastore if needed
//     /// </summary>
//     void IRenderable.Render(List<BaseArsisEvent> list)
//     {
//         // This method is kept for backward compatibility with IRenderable interface
//         // In normal operation, procedures are loaded from ProcedureManager in Start()
//         Debug.Log("Render called via IRenderable - using ProcedureManager instead");
//     }
// }
