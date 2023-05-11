using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using Microsoft.MixedReality.Toolkit.UI; 

/// <summary>
/// Controls all displaying of menus. 
/// </summary>
public class MenuController : MonoBehaviour
{
    public static MenuController s; 
    // Default menu; set as current menu on initialization 
    [HideInInspector]
    public GameObject m_CurrentMenu;
    private GameObject m_PreviousMenu;
    private ArrayList activeMenus;

    // Variables to keep track of your place in the procedure
    public int currentProcedure;
    public int currentTask;
    public int currentSubTask;
    public int currentMaxSteps;

    // Menus 
    [Header("Set Menus Here")]
    public GameObject m_mainMenu;
    public GameObject m_settingsMenu;
    public GameObject m_brightnessMenu;
    public GameObject m_volumeMenu;
    public GameObject m_sosMenu;
    public GameObject m_helpMenu;
    public GameObject m_biometricsMenu;
    public GameObject m_procedureList;
    public GameObject m_taskList;
    public GameObject m_musicMenu;
    public GameObject m_overlapMessage;
    public GameObject m_blankTaskMenu;
    public GameObject m_diagramList;
    public GameObject m_Diagram;
    public GameObject m_newFieldNote;
    public GameObject m_fieldNotes;
    //--5.0 Additions
    public GameObject m_mapMenu1;
    public GameObject m_mapMenu2;
    public GameObject m_calibrationMenu;
    public GameObject m_debugMenu;
    public GameObject m_beaconListMenu;

    //Diagrams
    public Texture2D d_BGA_Hardware_Overview;
    public Texture2D d_BMRRM_Connectors;
    public Texture2D d_BMRRM_Fasterners;
    public Texture2D d_DCU;
    public Texture2D d_UIA;
    public RawImage diagramImage;



    // Elements of task menu (procedurally populated) 
    public TextMeshPro m_TaskText;
    public TextMeshPro m_SubTaskText;
    public RawImage m_stepImage;
    public TextMeshPro m_stepPrevText;
    public TextMeshPro m_stepCurText;
    public TextMeshPro m_stepNextText;
    public TextMeshPro m_warningText;
    public TextMeshPro m_QRScanData;
    public GameObject nextButton; 

    public bool taskZoomedIn = false;

    //Menu Size
    float xScale = 0.2545305f;
    float ySacle = 0.2971f;

    [Header("Audio")]
    public AudioSource m_Source;
    public AudioClip m_changeMenuSound;

    public GameObject currentMenuHit = null;

    public delegate void Unhighlight();
    public Unhighlight unhighlight;

    public delegate void Deselect();
    public Deselect deselect; 

    public void Start()
    {
        //SpatialMapping.Instance.MappingEnabled = false; 

        activeMenus = new ArrayList();
        currentSubTask = 0;
        currentTask = 0;
        currentProcedure = 0;

        s = this; 
    }

    //hide old menu, and switch to new menu
    public void ChangeMenu(GameObject newMenu)
    {
        GameObject oldMenu = m_CurrentMenu;
        m_CurrentMenu = newMenu;
        if (oldMenu != null)
        {
            m_PreviousMenu = currentMenuHit;
            currentMenuHit.SetActive(false);

            // Set position and rotation of the new menu to be the same as the previous menu 
            newMenu.transform.position = currentMenuHit.transform.position;
            newMenu.transform.rotation = currentMenuHit.transform.rotation;
        }

        // Make the new menu visible 
        ToggleVisibility(newMenu);
        currentMenuHit = null; 

        // Play sound 
        m_Source.clip = m_changeMenuSound;
        m_Source.Play();
    }

    public void toggleDisplay(GameObject holoMenu)
    {

        if (holoMenu.activeSelf == false)
        {
            holoMenu.transform.position = Camera.main.transform.position + (Camera.main.transform.forward);

            Quaternion q = Quaternion.LookRotation(holoMenu.transform.position - Camera.main.transform.position, Camera.main.transform.up);

            holoMenu.transform.rotation = q;

            holoMenu.transform.rotation = Quaternion.Euler(holoMenu.transform.eulerAngles.x, holoMenu.transform.eulerAngles.y, holoMenu.transform.eulerAngles.z);
            // OT Removed + 90
            holoMenu.SetActive(true);
            activeMenus.Add(holoMenu);
        }

        else
        {
            holoMenu.SetActive(false);
        }
    }

    // Zoom out of task menu 
    public void zoomOut()
    {
        m_stepImage.gameObject.SetActive(false);
        m_SubTaskText.gameObject.SetActive(false);
        m_warningText.gameObject.SetActive(false);

        m_stepPrevText.gameObject.SetActive(true);
        m_stepNextText.gameObject.SetActive(true);
        m_stepCurText.gameObject.SetActive(true);
    }

    public void closeMenu()
    {
        if (currentMenuHit != null)
        {
            if (currentMenuHit == m_newFieldNote)
            {
                FieldNotesManager.s.inProgress = false;
            }
            //currentMenuHit.transform.gameObject.SetActive(false);
            currentMenuHit.SetActive(false); 
        }
    }

    public void followMe()
    {
        if (currentMenuHit != null)
        {
            //currentMenuHit.transform.gameObject.SetActive(false);
            FollowMeToggle followtoggle = currentMenuHit.GetComponentInParent<FollowMeToggle>();
            if (followtoggle == null) followtoggle = GetComponent<FollowMeToggle>();
            followtoggle.ToggleFollowMeBehavior(); 
        }
    }
    /* aads a menu to the field of view. 
     * When user tries to place a menu over an existing menu,
     * the option is given to replace the old menu with the new 
     */
    public void addMenu(GameObject holoMenu)
    {
        // Commenting this out to disable toggling - OT 4/11
        //  if (holoMenu.activeSelf == false)
        //   {
        // Check if current gaze interesects with an active menu
        RaycastHit hit;

        if (Physics.Raycast(Camera.main.transform.position, Camera.main.transform.rotation * Vector3.forward, out hit, Mathf.Infinity) && hit.transform.tag == "Menu")
        {
            Debug.Log("Trying to place menu where one already exists!");
            // if overlap move overlapping menu to the side
            GameObject triggeredObj = hit.transform.gameObject;
            // alert user: Do you want to replace the old menu with the new one
            //toggleDisplay(m_overlapMessage);
            m_CurrentMenu = triggeredObj; // This is needed for the change menu functionality

            // Get users answer
            //VoiceManager.S.Answer(holoMenu);

            //OT 4/11 - just replacing the menu
            ChangeMenu(holoMenu);

            //----------------------------------------------------------------UNUSED CODE---------------------------------------------------------------------------------
            // May want to implement one of these ideas if time
            //Speech to text    
            // The object for controlling the speech synthesis engine (voice).
            /*
#if !UNITY_EDITOR
            var synth = new Windows.Media.SpeechSynthesis.SpeechSynthesizer();
            // Generate the audio stream from plain text.
            SpeechSynthesisStream stream = await synth.SynthesizeTextToStreamAsync("Menu Overlap Detected, Do you want to replace the ");
            // Send the stream to the media object.
            m_Source.clip = stream;
            m_Source.Play();
#endif
*/
            // this does move it
            /* 
             triggeredObj.transform.Translate(new Vector3(-.35f, -.25f, 0));
             Quaternion qu = Quaternion.LookRotation(triggeredObj.transform.position - Camera.main.transform.position, Camera.main.transform.up);
             triggeredObj.transform.rotation = qu;
             triggeredObj.transform.rotation = Quaternion.Euler(triggeredObj.transform.eulerAngles.x + 90, triggeredObj.transform.eulerAngles.y, triggeredObj.transform.eulerAngles.z);
             */

            //----------------------------------------------------------------END OF UNUSED CODE ----------------------------------------------------------------------------------------------------------------
        }

        else
        {
            // open new menu where the user is looking
            holoMenu.transform.position = Camera.main.transform.position + (Camera.main.transform.forward);

            Quaternion q = Quaternion.LookRotation(holoMenu.transform.position - Camera.main.transform.position, Camera.main.transform.up);

            holoMenu.transform.rotation = q;

            holoMenu.transform.rotation = Quaternion.Euler(holoMenu.transform.eulerAngles.x, holoMenu.transform.eulerAngles.y, holoMenu.transform.eulerAngles.z);
            // OT Removed + 90
            holoMenu.SetActive(true);
            activeMenus.Add(holoMenu);
        }
        //  }

        // stop dislpaying the menu
        //  else
        //  {

        //holoMenu.SetActive(false);
        //activeMenus.Remove(holoMenu);
        // }

        // Play menu sound
        m_Source.clip = m_changeMenuSound;
        m_Source.Play();
    }


    // display a menu where someone is looking whether it is open or not
    public void Retrieve(GameObject holoMenu)
    {
        //set it to false in case it was open
        holoMenu.SetActive(false);

        // display the menu where the user is looking
        addMenu(holoMenu);
    }

    /*closes all open menus*/
    public void closeAll()
    {
        FieldNotesManager.s.inProgress = false; 
        // loop through the list of open menus and set the visibility to false

        foreach (GameObject menu in activeMenus)
        {
            menu.SetActive(false);
            activeMenus.Remove(menu);
        }

    }

    // Zoom in to task menu 
    public void zoomIn()
    {
        m_stepImage.gameObject.SetActive(true);
        m_SubTaskText.gameObject.SetActive(true);
        m_warningText.gameObject.SetActive(true);

        m_stepPrevText.gameObject.SetActive(false);
        m_stepNextText.gameObject.SetActive(false);
        m_stepCurText.gameObject.SetActive(false);
    }

    private void ToggleVisibility(GameObject holoMenu)
    {
        if (holoMenu == null) return;

        if (m_CurrentMenu != null)
        {
            m_CurrentMenu.SetActive(false);
        }

        holoMenu.SetActive(true);

        holoMenu.transform.position =
            Camera.main.transform.position + (Camera.main.transform.forward);

        Vector3 cameraPos = Camera.main.transform.position;

        Quaternion q = Quaternion.LookRotation(holoMenu.transform.position - Camera.main.transform.position, Camera.main.transform.up);

        holoMenu.transform.rotation = q;
        holoMenu.transform.rotation = Quaternion.Euler(holoMenu.transform.eulerAngles.x, holoMenu.transform.eulerAngles.y, holoMenu.transform.eulerAngles.z);
        // OT Removed + 90 

        m_CurrentMenu = holoMenu;
    }

    //go back to previous menu
    public void GoBack()
    {
        m_PreviousMenu.transform.position = m_CurrentMenu.transform.position;
        m_PreviousMenu.transform.rotation = Quaternion.Euler(new Vector3(m_CurrentMenu.transform.eulerAngles.x, Camera.main.transform.eulerAngles.y, m_PreviousMenu.transform.eulerAngles.z));

        m_CurrentMenu.SetActive(false);
        m_PreviousMenu.SetActive(true);
    }

   //public SelectableObj currentSelection = null; 
    private void Update()
    {

        RaycastHit hit;

        if (Physics.Raycast(Camera.main.transform.position, Camera.main.transform.rotation * Vector3.forward, out hit, Mathf.Infinity) && hit.transform.tag == "Menu")
        {
            if (currentMenuHit == null)
            {
                //Debug.Log("Forward!");
                //hit.transform.position = new Vector3(hit.transform.position.x, hit.transform.position.y, hit.transform.position.z - 0.05f);
                FollowMeToggle followtoggle = hit.transform.gameObject.GetComponentInParent<FollowMeToggle>();
                if (followtoggle == null) followtoggle = hit.transform.gameObject.GetComponent<FollowMeToggle>();
                GameObject menugo = followtoggle.gameObject;
                currentMenuHit = menugo;
            }
        }
        else if (currentMenuHit != null)
        {
            //Debug.Log("Back you!");
            //currentMenuHit.transform.position = new Vector3(currentMenuHit.transform.position.x, currentMenuHit.transform.position.y, currentMenuHit.transform.position.z + 0.05f);
            
            currentMenuHit = null;
        }

        /*if (Physics.Raycast(Camera.main.transform.position, Camera.main.transform.rotation * Vector3.forward, out hit, Mathf.Infinity) && hit.transform.tag == "selectable")
        {
            currentSelection = hit.transform.gameObject.GetComponent<SelectableObj>(); 
            unhighlight.Invoke(); // unselect everything before selecting the new button 
            currentSelection.onHighlight(); 
        } else if (currentSelection != null)
        {
            unhighlight.Invoke(); 
            currentSelection.onUnhighlight(); 
            currentSelection = null; 
        }*/
    }
}

