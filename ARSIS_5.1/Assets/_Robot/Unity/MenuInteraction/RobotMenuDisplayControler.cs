using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class RobotMenuDisplayControler : MonoBehaviour
{
    private static IRobotInterface robotController;

    public Button forwardButton;
    public Button backwardButton;
    public Button leftButton;
    public Button rightButton;


    public Slider speedSlider;
    void Start()
    {
        if (robotController == null)
        {
            robotController = new SimpleRobot();
        }
    }

    public void Forward()
    {
        robotController.MoveForward((byte)speedSlider.value);
    }

    public void Backward()
    {
        robotController.MoveBackward((byte)speedSlider.value);
    }

    public void Left()
    {
        robotController.RotateLeft((byte)speedSlider.value);
    }

    public void Right()
    {
        robotController.RotateRight((byte)speedSlider.value);
    }

    public void Stop()
    {
        robotController.Stop();
    }

    public void Reset()
    {
        robotController = new SimpleRobot();
    }
}
