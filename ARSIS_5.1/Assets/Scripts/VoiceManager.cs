using System.Collections;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Windows.Speech;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using System;
using UnityEngine.Events;
using Microsoft.MixedReality.Toolkit; 

/// <summary>
/// Manages all ADELE voice commands. 
/// </summary>
public class VoiceManager : MonoBehaviour
{
    public static VoiceManager S;

    private KeywordRecognizer _keywordRecognizer = null;
    private readonly Dictionary<string, System.Action> _keywords = new Dictionary<string, System.Action>();
    private bool _visible = false;
    // List of all the diagrams
    public List<Image> allDiagrams = new List<Image>();

    public MenuController mc;
    private GameObject menuToUse;

    public GameObject sadFace; 

    // Needed to check which settings menu is open for slider function 
    public GameObject m_brightnessMenu;
    public GameObject m_volumeMenu;
    public GameObject m_beaconListMenu;

    [Header("Audio")]
    public AudioSource m_Source;
    public AudioSource m_musicSource;

    public AudioClip m_OpenMenu;
    public AudioClip m_CloseMenu;
    public AudioClip m_ChangeMenu;
    public AudioClip m_NextButton;
    public AudioClip m_BackButton;
    public AudioClip m_ZoomIn;
    public AudioClip m_ZoomOut;
    public AudioClip m_SliderSound;

    public bool m_IsQRCodeScanning = false;
    private float m_QRCodeStartTime = 0.0f;

    private DictationRecognizer dictationRecognizer;

    public Image dot;
    public Text recognizedWord;

    float dictationTimer = 5.0f;
    bool dictationIsOn = false;

    public delegate void onCapture();
    public onCapture captureEvent;

    public bool voiceOn = true;
    public GameObject mrtkVoiceObject;

    public GameObject map;
    public bool showingMap = true; 

    void Awake()
    {
        S = this;

        /*    #region keywords
            ///////////////////// Main Menus ///////////////////// 
            _keywords.Add("Adele Main", MainMenu);
            _keywords.Add("Adele Menu", MainMenu);
            _keywords.Add("Adele Main Menu", MainMenu);
            _keywords.Add("Adele Settings", Settings);
            _keywords.Add("Adele Brightness", Brightness);
            _keywords.Add("Adele Volume", Volume);
            _keywords.Add("Adele Biometrics", Biometrics);
            _keywords.Add("Adele Houston", Houston);
            _keywords.Add("Adele Help", Help);
            _keywords.Add("Help", Help);
            _keywords.Add("Adele uh", Help);
            _keywords.Add("Adele um", Help);
            _keywords.Add("uh", Help);
            _keywords.Add("um", Help);
            _keywords.Add("Adele Diagrams", DiagramList);
            _keywords.Add("Adele Procedures", ProcedureList);
            _keywords.Add("Adele Music", musicMenu);
            _keywords.Add("Adele Tasklist", TaskList);
            _keywords.Add("Adele New Field Note", NewFieldNote);
            _keywords.Add("Adele Field Notes", FieldNotes); 

            ///////////////////// Menu Navigation /////////////////////
            _keywords.Add("Adele Reset", ResetScene);
            _keywords.Add("Adele Clear", ResetScene);
            _keywords.Add("Adele Close", Close);
            _keywords.Add("Adele Task", generateTaskMenu);
            _keywords.Add("Adele Next", Next);
            _keywords.Add("Adele Continue", Next);
            _keywords.Add("Adele Complete", Next);
            _keywords.Add("Adele Back", Back);
            _keywords.Add("Adele Zoom In", zoomIn);
            _keywords.Add("Adele Zoom Out", zoomOut);
            _keywords.Add("Adele Choose", Select); 
            // _keywords.Add("Adele Retrieve", Retrieve);

            ///////////////////// Translation /////////////////////
            _keywords.Add("Adele Record Path", StartTranslation);
            _keywords.Add("Adele End Path", StopTranslation);
            _keywords.Add("Adele Show Path", ShowPath);
            _keywords.Add("Adele Hide Path", HidePath);
            _keywords.Add("Adele Scan Code", ScanQRCode);

            ///////////////////// Special Functions /////////////////////
            _keywords.Add("Increase", Increase);
            _keywords.Add("Decrease", Decrease);
            _keywords.Add("Adele Capture", TakePhoto);
            _keywords.Add("Adele Toggle", Toggle);

            ///////////////////// Music /////////////////////
            //_keywords.Add("Adele Hello", PlayAdele);
            //_keywords.Add("Adele Africa", PlayAfrica);
            //_keywords.Add("Adele Skyfall", PlaySkyfall);
            //_keywords.Add("Adele Space Oddity", PlaySpaceOddity);
            //_keywords.Add("Adele Thunderstruck", PlayThunderstruck);
            //_keywords.Add("Adele Eclipse", PlayEclipse);
            //_keywords.Add("Adele Rocket Man", PlayRocketMan);
            //_keywords.Add("Adele Stop", StopMusic);
            //_keywords.Add("Adele shut up", PlaySkyfall);

            ///////////////////// Mesh /////////////////////
            _keywords.Add("Enable Mesh", enableMesh);
            _keywords.Add("Disable Mesh", disableMesh);
            _keywords.Add("Enable Mapping", enableMapping);
            _keywords.Add("Disable Mapping", disableMapping);

            ///////////////////// Diagrams /////////////////////
            _keywords.Add("Adele Diagram 1", Diagram1);
            _keywords.Add("Adele Diagram 2", Diagram2);
            _keywords.Add("Adele Diagram 3", Diagram3);
            _keywords.Add("Adele Diagram 4", Diagram4);
            _keywords.Add("Adele Diagram 5", Diagram5);

            #endregion */
        _keywords.Add("Adele Hello", PlayAdele); 

        // Sets up keyword recognition 
        _keywordRecognizer = new KeywordRecognizer(_keywords.Keys.ToArray());
        _keywordRecognizer.OnPhraseRecognized += KeywordRecognizer_OnPhraseRecognized;
        _keywordRecognizer.Start(); 

        // Initializes menu controller 
        //mc = FindObjectOfType(typeof(MenuController)) as MenuController;

        //Add the diagrams to allDiagrams list

    }

    public void resetKeywordRecognizer()
    {
        _keywordRecognizer.Stop();
        _keywordRecognizer.Dispose();
        _keywordRecognizer = new KeywordRecognizer(_keywords.Keys.ToArray());
        _keywordRecognizer.OnPhraseRecognized += KeywordRecognizer_OnPhraseRecognized;
        _keywordRecognizer.Start();
    }

    public void addProcedureCommand(string name, int procedureIndex, int taskIndex)
    {
        if (_keywords.ContainsKey(name)) return;

        _keywords.Add(name, () => {
            mc.currentProcedure = procedureIndex;
            mc.currentTask = taskIndex;
            mc.currentSubTask = 0;
            generateTaskMenu();
        });
        resetKeywordRecognizer();
        Debug.Log("Added voice command: " + name); 
    }

    public void addCommand(string name, UnityEvent response) {
        if (_keywords.ContainsKey(name)) return;
        _keywords.Add(name, () =>
        {
            response.Invoke(); 
        });
        //Debug.Log("Added command: " + name); 
        resetKeywordRecognizer(); 
    }

    public void removeCommand(string name)
    {
        if (_keywords.ContainsKey(name))
        {
            _keywords.Remove(name); 
        }
        //Debug.Log("Removed command: " + name); 
        resetKeywordRecognizer(); 
    }

    public void toggleSpatialMapping()
    {
        MeshDataGatherer.S.toggleTelestration(); 
    }

    public void toggleBiometrics()
    {

    }

    public void toggleMapVisibility()
    {
        Debug.Log("TOGGLE MAP VISIBILITY"); 
        if (showingMap)
        {
            showingMap = false; 
            map.SetActive(false);
        } else
        {
            showingMap = true;
            map.SetActive(true); 
        }
    }

    public void toggleVoice()
    {
        if (!voiceOn)
        {
            voiceOn = true;
            mrtkVoiceObject.SetActive(true);
        }
        else if (voiceOn)
        {
            voiceOn = false;
            mrtkVoiceObject.SetActive(false);
        }
    }

    public void toggleMap()
    {
        int currentMap = minimap_manager.S.currentMap; 
        if (currentMap == 0)
        {
            minimap_manager.S.currentMap = 1;
            minimap_manager.S.LoadMap(1); 
        } else if (currentMap == 1)
        {
            minimap_manager.S.currentMap = 0;
            minimap_manager.S.LoadMap(0);
        }
    }


    // Keyword Functions 
    #region Menu Functions

    public void MainMenu()
    {
        mc.addMenu(mc.m_mainMenu);
    }

    public void musicMenu()
    {
        mc.addMenu(mc.m_musicMenu);
    }

    public void Settings()
    {
        mc.addMenu(mc.m_settingsMenu);
    }

    public void Houston()
    {
        mc.addMenu(mc.m_sosMenu);
       // ServerConnect.S.sos();
    }

    public void Help()
    {
        mc.addMenu(mc.m_helpMenu);
    }

    public void Biometrics()
    {
        mc.addMenu(mc.m_biometricsMenu);
    }

    public void Brightness()
    {
        mc.addMenu(mc.m_brightnessMenu);
    }

    public void Volume()
    {
        mc.addMenu(mc.m_volumeMenu);
    }

    public void MapMenu1()
    {
        mc.addMenu(mc.m_mapMenu1);
    }

    public void MapMenu2()
    {
        mc.addMenu(mc.m_mapMenu2);
    }

    public void CalibrationMenu()
    {
        mc.addMenu(mc.m_calibrationMenu);
    }

    public void DebugMenu()
    {
        mc.addMenu(mc.m_debugMenu);
    }

    public void ProcedureList()
    {
        MenuController.s.addMenu(MenuController.s.m_procedureList);
    }

    public void DiagramList()
    {
        mc.addMenu(mc.m_diagramList);
    }

    public void TaskList()
    {
        mc.addMenu(mc.m_taskList);
        displayStep();
    }

    public void NewFieldNote()
    {
        mc.addMenu(mc.m_newFieldNote);
        FieldNotesManager.s.startFieldNote(); 
    }

    public void FieldNoteReturn()
    {
        if (mc.currentMenuHit == mc.m_fieldNotes)
        {
            FieldNotes(); 
        }
    }

    public void FieldNotes()
    {
        mc.addMenu(mc.m_fieldNotes);
        FieldNotesManager.s.showAllFieldNotes(); 

    }

    public void beaconMenu()
    {
        mc.addMenu(mc.m_beaconListMenu);
    }
    
    public void RoverMenu()
    {
        mc.addMenu(mc.m_RoverMenu);
    }

    /*Functions to add Diagrams to the scene*/
    private void Diagram1()
    {
        //Debug.Log("Diagram 1");
        Texture2D diagram = mc.d_BGA_Hardware_Overview;

        mc.diagramImage.texture = diagram;

        mc.addMenu(mc.m_Diagram);
    }

    private void Diagram2()
    {
        //Debug.Log("Diagram 1");
        Texture2D diagram = mc.d_BMRRM_Connectors;

        mc.diagramImage.texture = diagram;

        mc.addMenu(mc.m_Diagram);
    }

    private void Diagram3()
    {
        //Debug.Log("Diagram 1");
        Texture2D diagram = mc.d_BMRRM_Fasterners;

        mc.diagramImage.texture = diagram;

        mc.addMenu(mc.m_Diagram);
    }

    private void Diagram4()
    {
        //Debug.Log("Diagram 1");
        Texture2D diagram = mc.d_DCU;

        mc.diagramImage.texture = diagram;

        mc.addMenu(mc.m_Diagram);
    }

    private void Diagram5()
    {
        //Debug.Log("Diagram 1");
        Texture2D diagram = mc.d_UIA;

        mc.diagramImage.texture = diagram;

        mc.addMenu(mc.m_Diagram);
    }


    // handles voice cmds to retreive menu based off menu name
    public void Retrieve()
    {
        PhraseRecognitionSystem.Shutdown();
        dictationRecognizer = new DictationRecognizer();

        // start dictation reconizer
        dictationRecognizer.Start();
        dictationRecognizer.DictationResult += DictationRecognizer_DictationResult;
        dictationIsOn = true;
        
        //you have 5 seconds to say the menu name following the retrieve keyword
        dictationTimer = 5.0f;
    }

    // handles voice cmds to decide to replace the menu
    public void Answer(GameObject holoMenu)
    {
        menuToUse = holoMenu;
        PhraseRecognitionSystem.Shutdown();
        dictationRecognizer = new DictationRecognizer();

        // start dictation reconizer
        dictationRecognizer.Start();
        dictationRecognizer.DictationResult += Dictation_yesNo;

        dictationIsOn = true;
        dictationTimer = 5.0f;
    }



    private void DictationRecognizer_DictationResult(string text, ConfidenceLevel confidence)
    {
        GameObject holoMenu = null;
        Debug.Log("String heard: " + text);
        dictationRecognizer.Stop();


        // Cases to set holoMenu to the correct menu
        if (text.ToLower().Equals("main") || text.ToLower().Equals("main menu") || text.ToLower().Equals("maine") || text.ToLower().Equals("mean")) holoMenu = mc.m_mainMenu;
        else if (text.ToLower().Equals("biometrics")) holoMenu = mc.m_biometricsMenu;
        else if (text.ToLower().Equals("help")) holoMenu = mc.m_helpMenu;
        else if (text.ToLower().Equals("music")) holoMenu = mc.m_musicMenu;
        else if (text.ToLower().Equals("settings")) holoMenu = mc.m_settingsMenu;
        else if (text.ToLower().Equals("brightness")) holoMenu = mc.m_brightnessMenu;
        else if (text.ToLower().Equals("volume")) holoMenu = mc.m_volumeMenu;
        else if (text.ToLower().Equals("procedure")) holoMenu = mc.m_blankTaskMenu;
        else if (text.ToLower().Equals("beacon")) holoMenu = mc.m_beaconListMenu;
        else
        {
            Debug.Log("Cmd not recognized.");
            // This does not fail eloquently
        }


        // call function in MenuController to retrieve the specific menu
        if (holoMenu != null)
        {
            mc.Retrieve(holoMenu);
        }

        dictationRecognizer.Dispose();
        PhraseRecognitionSystem.Restart();
    }

    private void Dictation_yesNo(string text, ConfidenceLevel confidence)
    {


        dictationRecognizer.Stop();
        // dispose of dictation reconizer

        if (text.ToLower().Equals("yes"))
        {
            Debug.Log("String heard: " + text);
            mc.ChangeMenu(menuToUse);
            //mc.toggleDisplay(menuToUse);

        }
        else
        {
            Debug.Log("String heard: " + text);

        }

        mc.toggleDisplay(mc.m_overlapMessage);
        dictationRecognizer.Dispose();
        PhraseRecognitionSystem.Restart();
    }

    #endregion

    #region Navigation Functions 

    public void Menu()
    {
        mc.m_blankTaskMenu.gameObject.SetActive(false);
        mc.ChangeMenu(mc.m_blankTaskMenu);

        m_Source.clip = m_OpenMenu;
        m_Source.Play();
    }

    public void ResetScene()
    {
        m_Source.clip = m_CloseMenu;
        m_Source.Play();

        SceneManager.LoadScene(0);
    }

    public void Previous()
    {
        mc.GoBack();
    }

    public void Close()
    {
        //m_Source.clip = m_CloseMenu;
        //m_Source.Play();
        mc.closeMenu();
    }

    public void FollowMe()
    {
        mc.followMe(); 
    }

    public void Select()
    {
       /* if (mc.currentSelection != null)
        {
            mc.currentSelection.GetComponent<SelectableObj>().onSelect(); 
        }*/
    }

    public void BadWord()
    {
        sadFace.SetActive(true); 
        Invoke("sadFaceGoAway", 1f); 
    }

    private void sadFaceGoAway()
    {
        sadFace.SetActive(false); 
    }

    #endregion

    #region Special Functions 

    public void TakePhoto()
    {
        CameraCapture.S.Capture(); 

        m_Source.clip = m_ZoomOut;
        m_Source.Play();
    }

    public void Toggle()
    {
      //  VuforiaCameraCapture.S.ToggleImage();

        m_Source.clip = m_ZoomIn;
        m_Source.Play();
    }

  /*  public void Increase()
    {
        if (mc.currentMenuHit.Equals(m_brightnessMenu))
        {
            Debug.Log("Increasing Brightness");
            GameObject GOlt = GameObject.Find("Point light");
            Light lt = GOlt.GetComponent<Light>();
            if (lt.intensity < 1.4)
            {
                lt.intensity += 0.2f;
                SliderMove sm = mc.m_CurrentMenu.GetComponent<SliderMove>();
                sm.Increase();

                m_Source.clip = m_SliderSound;
                m_Source.Play();
            }
        }
        if (mc.currentMenuHit.Equals(m_volumeMenu))
        {
            Debug.Log("Increasing Volume");
            if (m_Source.volume < 1)
            {
                m_Source.volume += 0.2f;
                m_musicSource.volume += 0.2f;
                SliderMove sm = mc.m_CurrentMenu.GetComponent<SliderMove>();
                sm.Increase();

                m_Source.clip = m_SliderSound;
                m_Source.Play();
            }
        }
    }

    public void Decrease()
    {
        if (mc.m_CurrentMenu.Equals(GameObject.Find("ToggleSliderMenu")))
        {
            GameObject GOlt = GameObject.Find("Point light");
            Light lt = GOlt.GetComponent<Light>();
            if (lt.intensity > 0.6)
            {
                lt.intensity -= 0.2f;
                SliderMove sm = mc.m_CurrentMenu.GetComponent<SliderMove>();
                sm.Decrease();

                m_Source.clip = m_SliderSound;
                m_Source.Play();
            }
        }
        if (mc.m_CurrentMenu.Equals(m_volumeMenu))
        {
            Debug.Log("Decreasing Volume");
            if (m_Source.volume > 0)
            {
                m_Source.volume -= 0.2f;
                m_musicSource.volume -= 0.2f;
                SliderMove sm = mc.m_CurrentMenu.GetComponent<SliderMove>();
                sm.Decrease();

                m_Source.clip = m_SliderSound;
                m_Source.Play();
            }
        }
    }*/

    #endregion

    #region Task List Functions 

    public void generateTaskMenu()
    {
        mc.addMenu(mc.m_blankTaskMenu);
        displayStep();

        m_Source.clip = m_OpenMenu;
        m_Source.Play();
    }

    public void Skip()
    {
        if (FieldNotesManager.s.inProgress)
        {
            FieldNotesManager.s.setSkipped();
            FieldNotesManager.s.nextQuestion(); 
            //mc.deselect.Invoke();
            //mc.unhighlight.Invoke(); 
        }
    }

    public void Next()
    {
        if (FieldNotesManager.s.inProgress)
        {
            FieldNotesManager.s.nextQuestion();
            //mc.deselect.Invoke();
            //mc.unhighlight.Invoke(); 
            m_Source.clip = m_NextButton;
            m_Source.Play();
        }
        if (mc.currentMenuHit == mc.m_taskList)
        {
            NextStep(); 
        }
    }

    public void NextStep()
    {
        mc.currentSubTask++;

        //int maxLength = TaskManager.S.allTasks[mc.currentTask];
        if (mc.currentSubTask > TaskManager.S.GetTask(mc.currentProcedure, mc.currentTask).SubTasks.Length - 1)
        {
            //if there are no more subtasks, task is complete
            mc.currentSubTask = 0;
            mc.currentTask++;



            if (mc.currentTask > TaskManager.S.GetProcedure(mc.currentProcedure).Tasks.Length - 1)
            {
                //when procedure is complete
                mc.m_blankTaskMenu.SetActive(false);
                mc.currentProcedure = -1; 
                mc.currentTask = 0;
                mc.currentSubTask = 0; 
                //mc.currentProcedure++;
            }
        }

        displayStep();
        m_Source.clip = m_NextButton;
        m_Source.Play();
    }

    public void Back()
    {
        mc.currentSubTask--;
        //int maxLength = TaskManager.S.allTasks[mc.currentTask]; // 
        if (mc.currentSubTask < 0)
        {
            mc.currentTask--;

            if (mc.currentTask < 0)//if there are no more subtasks, task is complete
            {
                //when procedure is complete
                mc.currentProcedure--;

                if (mc.currentProcedure < 0)
                {
                    mc.currentProcedure = 0; //ensure that procedure index is never less than 0
                    mc.currentTask = TaskManager.S.GetProcedure(mc.currentProcedure).Tasks.Length - 1;
                    mc.currentSubTask = TaskManager.S.GetTask(mc.currentProcedure, mc.currentTask).SubTasks.Length - 1;
                }
                else
                {
                    mc.currentTask = TaskManager.S.GetProcedure(mc.currentProcedure).Tasks.Length - 1;
                    mc.currentSubTask = TaskManager.S.GetTask(mc.currentProcedure, mc.currentTask).SubTasks.Length - 1;
                }
            }
            else
            {
                mc.currentSubTask = TaskManager.S.GetTask(mc.currentProcedure, mc.currentTask).SubTasks.Length - 1;
            }
        }

        displayStep();

        m_Source.clip = m_BackButton;
        m_Source.Play();
    }

    public void zoomOut()
    {
        mc.zoomOut();

        m_Source.clip = m_ZoomOut;
        m_Source.Play();
    }

    public void zoomIn()
    {
        mc.zoomIn();

        m_Source.clip = m_ZoomIn;
        m_Source.Play();
    }

    public void displayStep()
    {
        int curProcedure = mc.currentProcedure;
        int curTask = mc.currentTask;
        int curSubTask = mc.currentSubTask;

        bool emergency = TaskManager.S.GetProcedure(mc.currentProcedure).emergency; 
        if (emergency)
        {
            mc.nextButton.SetActive(false); 
        } else
        {
            mc.nextButton.SetActive(true); 
        }

        Debug.Log("Trying to display procedure " + mc.currentProcedure + "task" + mc.currentTask + " subtask " + mc.currentSubTask);

        string taskText = TaskManager.S.GetTask(curProcedure, curTask).Title;
        string curText = TaskManager.S.GetSubTask(curProcedure, curTask, curSubTask);
        string prevText = TaskManager.S.GetSubTask(curProcedure, curTask, curSubTask - 1);
        string nextText = TaskManager.S.GetSubTask(curProcedure, curTask, curSubTask + 1);

        mc.m_TaskText.text = taskText;
        mc.m_SubTaskText.text = curText;

        //mc.m_stepPrevText.text = prevText;
        //mc.m_stepCurText.text = curText;
        //mc.m_stepNextText.text = nextText;

        Texture2D curImage = TaskManager.S.getPic(curProcedure, curTask, curSubTask);

        if (curImage != null)
        {
            mc.m_stepImage.texture = curImage;
        }

        string warningText = TaskManager.S.getWarning(curProcedure, curTask, curSubTask);
        //mc.m_warningText.text = warningText;
    }

    private void Update()
    {
        if (dictationIsOn)
        {
            dictationTimer -= Time.deltaTime;
        }
        if (dictationTimer < 0)
        {
            destroyDictationRecognizer();
            dictationIsOn = false;
            dictationTimer = 5.0f;
            Debug.Log("Dictation stopped");
        }

        //For Debugging without voice
        //if (Input.GetKeyDown(KeyCode.I))
        //{
        //    TaskList();
        //}

        //if (Input.GetKeyDown(KeyCode.O))
        //{
        //    Next();
        //}

        //if (Input.GetKeyDown(KeyCode.B))
        //{
        //    Biometrics();
        //}

    }

    private void FixedUpdate()
    {
        //if QR code has finished scanning
        if (m_QRCodeStartTime - Time.realtimeSinceStartup > 10.0f)
        {
            m_IsQRCodeScanning = false;
        }
    }

    void destroyDictationRecognizer()
    {
        if (mc.m_overlapMessage.gameObject.activeSelf)
        {
            mc.toggleDisplay(mc.m_overlapMessage);
        }
        dictationRecognizer.Stop();
        dictationRecognizer.Dispose();
        PhraseRecognitionSystem.Restart();
    }

    #endregion

    #region Task Names

    //public void disableAlarm()
    //{
    //    mc.currentTask = 1;
    //    mc.currentStep = 1;
    //    generateTaskMenu();
    //}

    //public void reroutePower()
    //{
    //    mc.currentTask = 2;
    //    mc.currentStep = 1;
    //    generateTaskMenu();
    //}

    //public void lightSwitch()
    //{
    //    mc.currentTask = 3;
    //    mc.currentStep = 1;
    //    generateTaskMenu(); 
    //}

    #endregion

    #region Music Functions 

    public void PlayAdele()
    {
       // MusicManager.m_Instance.PlaySong(MusicManager.m_Instance.m_AdeleSong);
    }

    public void PlayAfrica()
    {
      //  MusicManager.m_Instance.PlaySong(MusicManager.m_Instance.m_Africa);
    }

    public void PlaySkyfall()
    {
      //  MusicManager.m_Instance.PlaySong(MusicManager.m_Instance.m_Skyfall);
    }

    public void PlaySpaceOddity()
    {
       // MusicManager.m_Instance.PlaySong(MusicManager.m_Instance.m_SpaceOddity);
    }

    public void PlayThunderstruck()
    {
       // MusicManager.m_Instance.PlaySong(MusicManager.m_Instance.m_Thunderstruck);
    }

    public void PlayEclipse()
    {
      //  MusicManager.m_Instance.PlaySong(MusicManager.m_Instance.m_Eclipse);
    }

    public void PlayRocketMan()
    {
      //  MusicManager.m_Instance.PlaySong(MusicManager.m_Instance.m_RocketMan);
    }

    public void StopMusic()
    {
      //  MusicManager.m_Instance.StopMusic();
        m_Source.Stop();
    }
    #endregion

    #region Translation 

    public void StartTranslation()
    {
        TranslationController.S.startPathCapture();
    }

    public void StopTranslation()
    {
        TranslationController.S.stopPathCapture();
    }

    public void ShowPath()
    {
        TranslationController.S.showPath();
    }

    public void HidePath()
    {
        TranslationController.S.hidePath();
    }

    public void ScanQRCode()
    {
      //  VuforiaCameraCapture.S.BeginScanQRCode();
        m_IsQRCodeScanning = true;

        Debug.Log("Scanning QR Code");
    }

    #endregion

    #region Mesh

    public void enableMesh()
    {
       // MeshDataGatherer.S.enableMeshDisplay();
    }


    public void disableMesh()
    {
      //  MeshDataGatherer.S.disableMeshDisplay();
    }

    public void enableMapping()
    {

    }

    public void disableMapping()
    {

    }

    #endregion
    // Keyword Recognition 
    private void KeywordRecognizer_OnPhraseRecognized(PhraseRecognizedEventArgs args)
    {
        if (!voiceOn) return; 
        //dot.color = Color.red;
        /*string input = args.text;
        if (input.ToLower().Contains("adele"))
        {
            string[] array = input.Split(' ');
            input = "";
            for (int i = 1; i < array.Length; i++)
            {
                input += array[i];
                input += " ";
            }
        }*/
        //recognizedWord.text = input;
        //Invoke("dotWhite", 1f);

        System.Action keywordAction;
        if (_keywords.TryGetValue(args.text, out keywordAction))
        {
            keywordAction.Invoke();
        }
    }

    private void dotWhite()
    {
        recognizedWord.text = "";
        dot.color = Color.white;
    }
}