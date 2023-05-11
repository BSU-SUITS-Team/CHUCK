/*
    This interface is used to communicate with the robot.
    It is used to send and receive data from the robot.
*/
public interface IRobotInterface
{
    /// <summary>Returns the connection status of the robot.</summary>
    bool isConnected { get; }

    /// <returns><c>true</c> if there is another message in the queue, otherwise <c>false</c>.</returns>
    bool hasRecievedMessage { get; }

    /**
        <summary>
        Begins the server the robot can connect to.
        If it is already connected, it will do nothing.
        </summary>

        <param name="port">The port of the robot.</param>
    */
    void BeginServer(int port);

    /**
        <summary>
        Begins the server the robot can connect to.
        If it is already started, it will do nothing.
        After the robot connects, it will call the callback
        function.
        </summary>

        <param name="port">The port of the robot.</param>
        <param name="callback">
        The callback function to be called after the connection is attempted.
        It should be a function that takes no parameters and returns nothing.
        It can check the connection status using the <c>isConnected</c> property
        but that should always return true.
        </param>
    */
    void BeginServer(int port, System.Action callback);

    /**
        <summary>
        Disconnects from the robot.
        If it is already disconnected, it will do nothing.
        </summary>
    */
    void Disconnect();

    /**
        <summary>
        Sends a message to the robot.
        This is mainly for internal use and
        debugging. It should not be nesssary 
        to use it.

        NOT IMPLEMENTED YET ON THE ROBOT.
        </summary>

        <param name="message">
        The message to be sent to the Robot.
        </param>        
    */
    void SendMessage(string message);

    /**
        <summary>
        This will recieve the next message from the
        message queue and return it.

        This is mainly for internal use and
        debugging. It should not be nesssary
        to use it.
        </summary>

        <returns>The next message in the queue.</returns>
        <exception cref="System.InvalidOperationException"> Thrown if the queue is empty. </exception>
    */
    string GetMessage();

    /**
        <summary>
        Adds a function to be called when a message is recieved.
        The delagate will be called with the message as the argument.
        This will not remove the function from the list or remove the
        message from the queue.
        </summary>

        <param name="callback">
        The function to be called when a message is recieved.
        It should be a function that takes a string as an argument and
        it should not return anything.
        </param>
    */
    void AddMessageListener(System.Action<string> callback);

    /**
        <summary>
        Adds a function to be called when the robot disconnects.
        </summary>

        <param name="callback">
        The function to be called when the robot disconnects.
        It should be a function that takes no arguments and
        it should not return anything. This will be removed on
        disconnect.
        </param>
    */
    void AddDisconnectionListener(System.Action callback);


    // Robot Control Methods

    /**
        <summary>
        Sets the speed of a motor. This should not be necessary
        to use outside the implementation of the <c>MoveForward</c>,
        <c>MoveBackward</c>, <c>RotateLeft</c>, and <c>RotateRight</c>
        methods
        </summary>

        <param name="indx">The index of the motor.</param>
        <param name="speed">The speed to set the motor to. <c>0-255</c> </param>

        <exception cref="System.ArgumentException">Thrown if the index is not 0 or 1.</exception>
    */
    void SetSpeed(int indx, byte speed);

    /**
        <summary>
        Sets the speed of motors to move forward.
        </summary>

        <param name="speed">The speed to set the motors to. <c>0-255</c> </param>
    */
    void MoveForward(byte speed);

    /**
        <summary>
        Sets the speed of motors to move backward.
        </summary>

        <param name="speed">The speed to set the motors to. <c>0-255</c> </param>
    */
    void MoveBackward(byte speed);

    /**
        <summary>
        Sets the speed of motors to rotate left.
        </summary>

        <param name="speed">The speed to set the motors to. <c>0-255</c> </param>
    */
    void RotateLeft(byte speed);

    /**
        <summary>
        Sets the speed of motors to rotate right.
        </summary>

        <param name="speed">The speed to set the motors to. <c>0-255</c> </param>
    */
    void RotateRight(byte speed);

    /**
        <summary>
        Stops the motors.
        </summary>
    */
    void Stop();
}
