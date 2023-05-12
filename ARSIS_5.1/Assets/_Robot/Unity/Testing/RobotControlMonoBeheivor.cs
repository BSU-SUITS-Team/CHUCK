using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class RobotControlMonoBeheivor : MonoBehaviour
{
    private IRobotInterface robotInterface;
    public const int port = 28512;

    [Range(0, 255)]
    public int speed = 127;

    public Text statusText;
    void Start()
    {
        robotInterface = new SimpleRobot();
    }
    // Update is called once per frame
    void Update()
    {
        if (robotInterface.isConnected)
        {
            statusText.text = "Status: Connected";
        }
        else
        {
            statusText.text = "Status: Disconnected";
        }
    }
    public void Connect()
    {
        robotInterface.BeginServer(port, delegate { OnConnect(); });
    }

    public void Disconnect()
    {
        robotInterface.Disconnect();
    }
    public void OnConnect()
    {
        Debug.Log("Connected");
    }

    public void MoveForward()
    {
        robotInterface.MoveForward((byte)speed);
    }

    public void MoveBackward()
    {
        robotInterface.MoveBackward((byte)speed);
    }

    public void RotateLeft()
    {
        robotInterface.RotateLeft((byte)speed);
    }

    public void RotateRight()
    {
        robotInterface.RotateRight((byte)speed);
    }

    public void Stop()
    {
        robotInterface.Stop();
    }
}
