using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI; 
using System;
using UnityEngine.Networking;

/// <summary>
/// Manages all tasks. Tasks are currently hard-coded in; however, we are working on a solution to pull them from a server. 
/// </summary>
public class TaskManager : MonoBehaviour
{

    // Singleton 
    public static TaskManager S;

    // List of all steps organized by procedure 
    public List<Procedure> allProcedures = new List<Procedure>(); 
    
    // Lists of individual steps 
    //public Procedure disableAlarm = new Procedure("Disable Alarm", false, 24);
    //public Procedure reroutPower = new Procedure("Reroute Power", false, 30);
    //public Procedure lightSwitch = new Procedure("Light Switch", false, 20);

    // Static Array of images for each task 
    public Texture2D[] images;

    // Web Connection 
    public string url = "";
    //private OutputErrorData m_OutputErrorData;

    // For testing 
    //public GameObject cube; 

    void Start()
    {
        S = this;
        //add procedure voice commands
        for (int i = 0; i < allProcedures.Count; i++)
        {
            string s = allProcedures[i].procedure_title.Split(':')[0];
            //VoiceManager.S.addProcedureCommand("Adele " + s + " Procedure", i, 0);
        }

        //add task voice commands
        for (int i = 0; i < allProcedures.Count; i++)
        {
            for (int j = 0; j < allProcedures[i].Tasks.Length; j++)
            {
                string s = allProcedures[i].Tasks[j].Title.Split(':')[0];
                VoiceManager.S.addProcedureCommand(s, i, j);
            }
        }

        //m_OutputErrorData = FindObjectOfType<OutputErrorData>();
        //InvokeRepeating("UpdateSystemData", 1, 5);
    }

    private void LateUpdate()
    {
        int currentProcedure = MenuController.s.currentProcedure;
        int currentTask = MenuController.s.currentTask;
        int currentSubTask = MenuController.s.currentSubTask;

        if (currentProcedure != 0 || currentTask == -1 || currentSubTask == -1) return;
        if (!MenuController.s.m_blankTaskMenu.activeInHierarchy) return; // only check if the procedures menu is open
        if (currentProcedure == 0 && currentTask == 3 && currentSubTask == 3) return; // hardcoded hack to turn off checking if we are on the congrats you are done step
        SubTask currentSubTaskObj = allProcedures[currentProcedure].GetTask(currentTask).GetSubTask(currentSubTask);

        string varToCheck = currentSubTaskObj.varName;
        string expectedValue = currentSubTaskObj.expectedValue;

        string result = UIAController.S.checkVar(varToCheck);

        if (result != "" && result == expectedValue)
        {
            VoiceManager.S.NextStep(); 
        }
    }

    public Procedure GetProcedure(int procedureIndex)
    {
        Procedure returnVal;
        try
        {
            returnVal = allProcedures[procedureIndex];
        }
        catch (ArgumentOutOfRangeException)
        {
            Debug.Log("Argument out of range");
            returnVal = null;
        }
        catch (NullReferenceException)
        {
            Debug.Log("Null reference exception for procedure " + procedureIndex);
            returnVal = null;
        }

        return returnVal;
    }

    public Task GetTask(int procedureIndex, int taskIndex)
    {
        Task returnVal;
        try
        {
            returnVal = allProcedures[procedureIndex].GetTask(taskIndex);
        }
        catch (ArgumentOutOfRangeException)
        {
            Debug.Log("Argument out of range");
            returnVal = null;
        }
        catch (NullReferenceException)
        {
            Debug.Log("Null reference exception for procedure " + procedureIndex + " task " + taskIndex);
            returnVal = null;
        }

        return returnVal;
    }

    public string GetSubTask(int procedureIndex, int taskIndex, int subtaskIndex)
    {
        if (procedureIndex < 0 || taskIndex < 0 || subtaskIndex < 0)
        {
            //if any index is negative, bail
            return "";
        }

        string returnVal;
        try
        {
            returnVal = allProcedures[procedureIndex].GetTask(taskIndex).GetSubTask(subtaskIndex).text;
        }
        catch (ArgumentOutOfRangeException)
        {
            Debug.Log("Argument out of range");
            returnVal = "";
        }
        catch (NullReferenceException)
        {
            Debug.Log("Null reference exception for procedure " + procedureIndex + " task " + taskIndex + "subtask" + subtaskIndex);
            returnVal = "";
        }
        return returnVal;
    }

    public int getProcedureIndexByName(string name)
    {
        for (int i = 0; i < allProcedures.Count; i++)
        {
            if (allProcedures[i].procedure_title.Equals(name))
            {
                return i;
            }
        }
        return -1;
    }

    public Texture2D getPic(int procedureIndex, int taskIndex, int subtaskIndex)
    {
        Texture2D retval;
        try
        {
            retval = allProcedures[procedureIndex].GetTask(taskIndex).GetSubTask(subtaskIndex).getImage();
        }
        catch (ArgumentOutOfRangeException)
        {
            retval = null;
        }

        catch (NullReferenceException)
        {
            Debug.Log("Something is null");
            retval = null;
        }
        return retval;
    }

    public string getWarning(int procedureIndex, int taskIndex, int subtaskIndex)
    {
        string retval;
        try
        {
            retval = allProcedures[procedureIndex].GetTask(taskIndex).GetSubTask(subtaskIndex).warning;
        }
        catch (ArgumentOutOfRangeException)
        {
            retval = "";
        }

        catch (NullReferenceException)
        {
            retval = "";
        }
        return retval;
    }

    // Server Connection Stuffs 
    private void UpdateSystemData()
    {
        StartCoroutine(RunWWW());
    }

    IEnumerator RunWWW()
    {
        using (UnityWebRequest www = UnityWebRequest.Get(url))
        {
            yield return www.SendWebRequest();

            string fromServer = "";
            if (www.isNetworkError)
            {
                Debug.Log("NETWORK ERROR Not connected to tasklist server :(");
            }
            else if (www.isHttpError)
            {
                Debug.Log("HTTP ERROR Not connected to tasklist server :(");
            }
            else
            {
                //Debug.Log("Connected to tasklist server"); 
                fromServer = www.downloadHandler.text;

                //Debug.Log(fromServer);

                Procedure jsonObject = JsonUtility.FromJson<Procedure>(fromServer);

                //Debug.Log(jsonObject.steps[0].image); 
                //string image = (String)jsonObject.steps[0].image;
                //Debug.Log(image); 

                //String b64String = (String)fromSocket["image"];
            }
        }
    }
}


// Used for parsing JSON from server 
[System.Serializable]
public class Procedure
{
    [Header("Procedure")]
    public string procedure_title;
    public bool emergency;
    public int num_steps;
    public Task[] Tasks;

    public Procedure(string title, bool emergency, int numSteps)
    {
        this.procedure_title = title;
        this.emergency = emergency;
        this.num_steps = numSteps;
        this.Tasks = new Task[numSteps]; 
    }

    public Task GetTask(int index)
    {
        try
        {
            return Tasks[index];
        } catch (IndexOutOfRangeException)
        {
            Debug.Log("Get tasks in procedure: " + procedure_title + "is out of range");
            return null;
        }
         
    }

    public void AddTask(int index, Task newTask)
    {
        Tasks[index] = newTask;
    }
}

//task is next level down under procedure
[System.Serializable]
public class Task
{

    [Header("Task")]
    public string Title;
    public SubTask[] SubTasks;
    bool complete = false; 

    public Task(string title, int numSubTasks)
    {
        this.Title = title;
        this.SubTasks = new SubTask[numSubTasks]; 
    }

    public void addSubTask(int index, SubTask s)
    {
        SubTasks[index] = s; 
    }

    public SubTask GetSubTask(int index)
    {
        try
        {
            return SubTasks[index];
        }
        catch (IndexOutOfRangeException)
        {
            Debug.Log("Get subtask in task: " + Title + "is out of range");
            return null;
        }

    }
}

[System.Serializable]
public class SubTask
{
    [Header("SubTask")]
    public string text;
    public string type;
    public string warning;
    public string image;

    public string varName = ""; // id of the variable to check for completeness 
    public string expectedValue = ""; // expected value of the variable called varName when the task is complete 

    public Texture2D imageTex = null; 

    public SubTask(string content, string type, string warning, Texture2D image)
    {
        this.text = content;
        this.type = type;
        this.warning = warning;
        this.imageTex = image;
    }

    public SubTask(string content, Texture2D image, string warning)
    {
        this.text = content;
        this.warning = warning;
        this.imageTex = image;
    }

    public Texture2D getImage()
    {
        if (this.imageTex == null)
        {
            // do magical stuff to convert the string to a texture
            return null; 
        } else
        {
            return imageTex; 
        }
    }
}
