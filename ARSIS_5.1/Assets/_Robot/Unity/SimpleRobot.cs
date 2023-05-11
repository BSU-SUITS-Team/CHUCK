using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;

public class SimpleRobot : IRobotInterface
{
    // Variables used to keep track of the connection.
    private TcpListener server = null;
    private Thread serverThread = null;
    public bool connected = false;

    //the message queue for the messages received from the robot
    private Queue<string> recievedMessageQueue = new Queue<string>();
    // private Queue<string> sendMessageQueue = new Queue<string>(); //might be used for more advanced communication later

    //The port to listen on.
    private int port;

    public bool isConnected { get { return connected; } }

    public bool hasRecievedMessage { get { return recievedMessageQueue.Count > 0; } }

    // callback delagates: used to call functions when the connection is made or message is recieved
    private System.Action connectionCallback;
    private List<System.Action<string>> messageCallbacks = new List<System.Action<string>>();
    private List<System.Action> disconnectionCallbacks = new List<System.Action>();

    private byte[] speeds = new byte[2];

    // Delagate type definitions
    
    delegate void MessageCallback(string message);

    /**
        <summary>
        The second thread that handles the connection to the robot.
        It is a TCP server that listens on the port specified in the <c>Connect</c> function.
        </summary>
    */
    private void Listen()
    {
        // Create a TCP/IP socket. 0.0.0.0 means that the socket will be available on all network interfaces/external IPs.
        server = new TcpListener(IPAddress.Parse("0.0.0.0"), port);
        byte[] inputBytes = new byte[1024];
        server.Start();
        // loop asa long as the thread is alive
        while (true)
        {
            // wait for a connection
            TcpClient client = server.AcceptTcpClient();
            connected = true;
            if (connectionCallback != null)
                connectionCallback();
            NetworkStream stream = client.GetStream();
            // loop as long as the connection is alive
            while (true)
            {
                try {
                    int bytesRead = stream.Read(inputBytes, 0, inputBytes.Length);
                    if (bytesRead == 0)
                        break;

                    string recievedMessage = System.Text.Encoding.ASCII.GetString(inputBytes, 0, bytesRead);
                    recievedMessageQueue.Enqueue(recievedMessage);

                    for (int i = 0; i < messageCallbacks.Count; i++)
                        messageCallbacks[i](recievedMessage);

                    // send current motor speeds to client
                    // every recieved message needs a response.
                    stream.Write(speeds, 0, 2);

                    //sleep for 50ms to avoid unnesssary communication
                    Thread.Sleep(50);
                }
                catch (SocketException) {
                    break;
                }
            }
            //Disconnected
            stream.Close();
            connected = false;
            connectionCallback = null;
            messageCallbacks.Clear();
            for (int i = 0; i < disconnectionCallbacks.Count; i++)
                disconnectionCallbacks[i]();
            client.Close();
            serverThread = null;
            break;
        }
    }

    public void AddMessageListener(System.Action<string> callback)
    {
        messageCallbacks.Add(callback);
    }

    public void AddDisconnectionListener(System.Action callback)
    {
        disconnectionCallbacks.Add(callback);
    }

    public void BeginServer(int port)
    {
        if (connected || serverThread != null)
            return;
        this.port = port;
        serverThread = new Thread(new ThreadStart(Listen));
        serverThread.IsBackground = true;
        serverThread.Start();
    }

    public void BeginServer(int port, System.Action callback)
    {
        if (connected || serverThread != null)
            return;
        this.port = port;
        connectionCallback = callback;
        serverThread = new Thread(new ThreadStart(Listen));
        serverThread.IsBackground = true;
        serverThread.Start();
    }

    public void Disconnect()
    {
        //TODO: Cleanly disconnect from server. Instead of just killing the thread.
        if (serverThread == null)
            return;
        serverThread.Abort();
        serverThread = null;
    }

    public string GetMessage()
    {
        if (recievedMessageQueue.Count == 0)
        {
            throw new System.InvalidOperationException("Queue is empty.");
        }
        return recievedMessageQueue.Dequeue();
    }

    public void MoveBackward(byte speed)
    {
        // Adds 128 (0b10000000 || 0x80) to reverse the direction.
        byte newSpeed = (byte)((speed >> 1) | 0x80);
        SetSpeed(0, newSpeed);
        SetSpeed(1, newSpeed);
    }

    public void MoveForward(byte speed)
    {
        byte newSpeed = (byte)(speed >> 1);
        SetSpeed(0, newSpeed);
        SetSpeed(1, newSpeed);
    }

    public void RotateLeft(byte speed)
    {
        byte newSpeed = (byte)(speed >> 1);
        SetSpeed(0, 0);
        SetSpeed(1, newSpeed);
    }

    public void RotateRight(byte speed)
    {
        byte newSpeed = (byte)(speed >> 1);
        SetSpeed(0, newSpeed);
        SetSpeed(1, 0);
    }

    public void SendMessage(string message)
    {
        // sendMessageQueue.Enqueue(message);
        throw new System.NotImplementedException();
    }

    public void SetSpeed(int indx, byte speed)
    {
        if (indx < 0 || indx > 1)
        {
            throw new System.ArgumentException("Index must be 0 or 1.");
        }
        speeds[indx] = speed;
    }

    public void Stop()
    {
        SetSpeed(0, 0);
        SetSpeed(1, 0);
    }
}