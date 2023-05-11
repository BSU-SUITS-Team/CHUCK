using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Microsoft.MixedReality.Toolkit.UI;
using TMPro;
using UnityEngine.Events; 

public class FieldNoteDisplay : MonoBehaviour
{
    public GameObject skipButton;
    public GameObject[] buttons; 
    public TextMeshProUGUI promptText;
    public GameObject instructionsText;
    public RawImage image;
    public Texture picturePrompt;
    public GameObject recordButton;
    public GameObject audioText; 

    private List<string> prevVoiceCommands; // keeps track of what voice commands to remove 

    float coolDown = 0f; // Cooldown exists because otherwise a button tap may be registered for the next set of buttons

    private void Update()
    {
        if (coolDown >= 0)
        {
            coolDown -= Time.deltaTime; 
        }
    }

    private void Awake()
    {
        prevVoiceCommands = new List<string>();
        recordButton.SetActive(false);
        audioText.SetActive(false); 
    }

    public void setQuestion(string text, string[] options, bool skippable)
    {
        recordButton.SetActive(false);
        audioText.SetActive(false); 
        foreach (string command in prevVoiceCommands)
        {
            VoiceManager.S.removeCommand(command); 
        }
        prevVoiceCommands = new List<string>();

        image.gameObject.SetActive(false); 
        promptText.text = text;
        instructionsText.SetActive(true); 
        for (int i = 1; i <= options.Length; i++)
        {
            buttons[i-1].SetActive(true);
            buttons[i - 1].GetComponentInChildren<TextMeshProUGUI>().text = options[i - 1];
            Interactable interact = buttons[i-1].GetComponent<Interactable>();
            if (i == 1) // This is bad like this because Olivia was lazy a year ago and hardcoded things in a terrible way, and she is also lazy this year and doesn't want to fix it.
                            // If you fix it you are a better person than her.
            {
                UnityEvent uevent = new UnityEvent();
                uevent.AddListener(firstAnswer); 
                VoiceManager.S.addCommand(options[i - 1], uevent);
                prevVoiceCommands.Add(options[i - 1]); 
                interact.OnClick.AddListener(firstAnswer);
            } else if (i == 2)
            {
                UnityEvent uevent = new UnityEvent();
                uevent.AddListener(secondAnswer);
                VoiceManager.S.addCommand(options[i - 1], uevent);
                prevVoiceCommands.Add(options[i - 1]);
                interact.OnClick.AddListener(secondAnswer);
            } else if (i == 3)
            {
                UnityEvent uevent = new UnityEvent();
                uevent.AddListener(thirdAnswer);
                VoiceManager.S.addCommand(options[i - 1], uevent);
                prevVoiceCommands.Add(options[i - 1]);
                interact.OnClick.AddListener(thirdAnswer);
            }
            else if (i == 4)
            {
                UnityEvent uevent = new UnityEvent();
                uevent.AddListener(fourthAnswer);
                VoiceManager.S.addCommand(options[i - 1], uevent);
                prevVoiceCommands.Add(options[i - 1]);
                interact.OnClick.AddListener(fourthAnswer);
            }
            else if (i == 5)
            {
                UnityEvent uevent = new UnityEvent();
                uevent.AddListener(fifthAnswer);
                VoiceManager.S.addCommand(options[i - 1], uevent);
                prevVoiceCommands.Add(options[i - 1]);
                interact.OnClick.AddListener(fifthAnswer);
            }
            else if (i == 6)
            {
                UnityEvent uevent = new UnityEvent();
                uevent.AddListener(sixthAnswer);
                VoiceManager.S.addCommand(options[i - 1], uevent);
                prevVoiceCommands.Add(options[i - 1]);
                interact.OnClick.AddListener(sixthAnswer);
            }

            //buttons[i - 1].GetComponentInChildren<SelectableObj>().resetCommand(options[i - 1]); 
        }
        for (int j = options.Length + 1; j <= 6; j++)
        {
            buttons[j - 1].SetActive(false); 
        }
        if (skippable)
        {
            skipButton.SetActive(true); 
        } else
        {
            skipButton.SetActive(false); 
        }
    }

    public void displayFinalQuestion()
    {
        skipButton.SetActive(false);
        for (int i = 0; i < buttons.Length; i++)
        {
            buttons[i].SetActive(false); 
        }
        instructionsText.SetActive(false); 
        promptText.text = "Field note recorded!"; 
    }

    public void displayRecordButton()
    {
        skipButton.SetActive(false);
        for (int i = 0; i < buttons.Length; i++)
        {
            buttons[i].SetActive(false);
        }
        instructionsText.SetActive(false);
        image.gameObject.SetActive(false);
        promptText.text = "Create a voice recording";
        setRecordButtonText("Start"); 
        recordButton.SetActive(true);
        audioText.SetActive(true); 
    } 

    public void setRecordButtonText(string text)
    {
        recordButton.GetComponentInChildren<TextMeshProUGUI>().text = text;
    }

    public void displayPicturePrompt()
    {
        recordButton.SetActive(false);
        audioText.SetActive(false); 
        image.gameObject.SetActive(true);
        image.texture = picturePrompt; 
        skipButton.SetActive(false);
        for (int i = 0; i < buttons.Length; i++)
        {
            buttons[i].SetActive(false);
        }
        instructionsText.SetActive(false);
        promptText.text = "Take a picture of the sample by saying \"Capture\"";
    }

    public Texture getPicture()
    {
        return image.texture; 
    }

    private void startCooldown()
    {
        coolDown = 1f; 
    }

    private bool canSelect()
    {
        return coolDown <= 0.1; 
    }

    public void firstAnswer()
    {
        if (!canSelect()) return; 
        startCooldown(); 
        FieldNotesManager.s.selectFirstAnswer();
        VoiceManager.S.Next();
    }

    public void secondAnswer()
    {
        if (!canSelect()) return;
        startCooldown();
        FieldNotesManager.s.selectSecondAnswer();
        VoiceManager.S.Next();
    }

    public void thirdAnswer()
    {
        if (!canSelect()) return;
        startCooldown();
        FieldNotesManager.s.selectThirdAnswer();
        VoiceManager.S.Next();
    }

    public void fourthAnswer()
    {
        if (!canSelect()) return;
        startCooldown();
        FieldNotesManager.s.selectFourthAnswer();
        VoiceManager.S.Next();
    }

    public void fifthAnswer()
    {
        if (!canSelect()) return;
        startCooldown();
        FieldNotesManager.s.selectFifthAnswer();
        VoiceManager.S.Next();
    }

    public void sixthAnswer()
    {
        if (!canSelect()) return;
        startCooldown();
        FieldNotesManager.s.selectSixthAnswer();
        VoiceManager.S.Next();
    }

    public void skip()
    {
        if (!canSelect()) return;
        startCooldown();
        FieldNotesManager.s.setSkipped();
        VoiceManager.S.Next(); 
    }
}
