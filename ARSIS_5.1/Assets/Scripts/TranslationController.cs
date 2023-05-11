using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class TranslationController : MonoBehaviour
{
    public LineRenderer lineRenderer = null;
    public float lineResolution = 0.1f; // resolution in meters, smaller is more accurate.
    private List<Vector3> waypoints = null;
    private bool doCapture = false;
    public GameObject recordingIndicator; 

    public static TranslationController S; 


    // Start is called before the first frame update
    void Start()
    {
        S = this; 
        if(lineRenderer==null)
        {
            Destroy(this);
            Debug.LogError("lineRenderer not set!");
            return;
        }
        waypoints = new List<Vector3>();
        clearPath();
        recordingIndicator.SetActive(false); 
    }
    
    void FixedUpdate()
    {
        if (doCapture)
            handlePathCapture();
    }

    public void startPathCapture()
    {
        clearPath();
        showPath(); 
        doCapture = true;
        recordingIndicator.SetActive(true);
        Debug.Log("Starting path capture.");
    }

    public void stopPathCapture()
    {
        doCapture = false;
        recordingIndicator.SetActive(false);
        Debug.Log("Stopping path capture.");
    }

    public void togglePathCapture()
    {
        if (doCapture)
        {
            stopPathCapture(); 
        } else
        {
            startPathCapture(); 
        }
    }

    public void hidePath()
    {
        Debug.Log("Hiding Path"); 
        lineRenderer.enabled = false;
    }

    public void showPath()
    {
        Debug.Log("Showing Path"); 
        lineRenderer.enabled = true;
    }

    private void handlePathCapture()
    {
        if (waypoints == null || lineRenderer == null)
        {
            Destroy(this);
            Debug.LogError("An object we depend on is not set!");
            return;
        }
        Vector3 pos = Camera.main.transform.position + new Vector3(0.0f, -0.3f, 0.0f);
        if (waypoints.Count <= 0)
        {
            lineRenderer.enabled = false;
            Vector3 firstPoint = pos;
            waypoints.Add(firstPoint);
            return;
        }
        
        for (int i = 0; i < waypoints.Count; i++)
        {
            float dist = (waypoints[i] - pos).magnitude;
            if (dist < lineResolution)
                return;
        }

        Vector3 newPoint = pos;
        waypoints.Add(newPoint);

        if (waypoints.Count >= 2)
        {
            lineRenderer.enabled = true;
            lineRenderer.positionCount = waypoints.Count;
            lineRenderer.SetPositions(waypoints.ToArray());
        }
        //Debug.Log("Tick: " + lineRenderer.enabled + " " + waypoints.Count + " " + lineRenderer.positionCount);
    }

    private void clearPath()
    {
        waypoints.Clear();
        lineRenderer.enabled = false;
        lineRenderer.SetPositions(waypoints.ToArray());       
        Debug.Log("Clearing path.");
    }

}
