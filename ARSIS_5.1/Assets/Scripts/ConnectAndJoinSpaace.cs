using UnityEngine;
using Photon.Pun;
using Photon.Realtime;
using Photon.Pun.Demo.Cockpit;
using System;

namespace Photon.Pun.UtilityScripts
{
    /// <summary>Simple component to call ConnectUsingSettings and to get into a PUN room easily.</summary>
    /// <remarks>A custom inspector provides a button to connect in PlayMode, should AutoConnect be false.</remarks>
    public class ConnectAndJoinSpaace : MonoBehaviourPunCallbacks
    {
        /// <summary>Connect automatically? If false you can set this to true later on or call ConnectUsingSettings in your own scripts.</summary>
        public bool AutoConnect = true;

        /// <summary>Used as PhotonNetwork.GameVersion.</summary>
        public byte Version = 1;

        private bool isConnecting = false;
        public static bool isConnected = false;
        public static int disconnectCount = 0;

        public RandomMeshGenerator randomMesh;

        public void Start()
        {
            if (this.AutoConnect)
            {
                this.ConnectNow();
            }
        }

        public void FixedUpdate()
        {
            if (PhotonNetwork.InRoom)
                return;
            if (!isConnecting && !PhotonNetwork.InRoom)
            {
                if (PhotonNetwork.InLobby)
                {
                    RoomOptions roomOptions = new RoomOptions();
                    roomOptions.IsVisible = true;
                    roomOptions.MaxPlayers = 4;
                    roomOptions.IsOpen = true;
                    PhotonNetwork.JoinOrCreateRoom("Spaace", roomOptions, TypedLobby.Default);
                    isConnecting = true;
                }
                else
                {
                    ConnectNow();
                }
            }
        }

        public void ConnectNow()
        {
            isConnecting = true;
            Debug.Log("ConnectAndJoinSpaace.ConnectNow() will now call: PhotonNetwork.ConnectUsingSettings().");
            PhotonNetwork.ConnectUsingSettings();
            PhotonNetwork.GameVersion = this.Version + "";

        }


        // below, we implement some callbacks of the Photon Realtime API.
        // Being a MonoBehaviourPunCallbacks means, we can override the few methods which are needed here.


        public override void OnConnectedToMaster()
        {
            Debug.Log("OnConnectedToMaster() was called by PUN. Now this client is connected and could join a room. Calling: PhotonNetwork.JoinOrCreateRoom(Spaace, roomOptions, TypedLobby.Default);");
            //PhotonNetwork.JoinRandomRoom();
            RoomOptions roomOptions = new RoomOptions();
            roomOptions.IsVisible = true;
            roomOptions.MaxPlayers = 4;
            roomOptions.IsOpen = true;
            PhotonNetwork.JoinOrCreateRoom("Spaace", roomOptions, TypedLobby.Default);
            //PhotonNetwork.JoinRoom("Spaace");
            isConnecting = true;
        }

        public override void OnJoinedLobby()
        {
            Debug.Log("OnJoinedLobby(). This client is connected. This script now calls: PhotonNetwork.JoinOrCreateRoom(Spaace, roomOptions, TypedLobby.Default);");

            RoomOptions roomOptions = new RoomOptions();
            roomOptions.IsVisible = true;
            roomOptions.MaxPlayers = 4;
            roomOptions.IsOpen = true;
            PhotonNetwork.JoinOrCreateRoom("Spaace", roomOptions, TypedLobby.Default);
            isConnecting = true;
        }

        public override void OnPlayerEnteredRoom(Player newPlayer)
        {
            Debug.Log("Other player entered room");
        }

        public override void OnJoinRandomFailed(short returnCode, string message)
        {
            Debug.Log("OnJoinRandomFailed() was called by PUN. No random room available, so we create one. Calling: PhotonNetwork.CreateRoom(null, new RoomOptions() {maxPlayers = 4}, null);");
            RoomOptions roomOptions = new RoomOptions();
            roomOptions.IsVisible = true;
            roomOptions.MaxPlayers = 4;
            roomOptions.IsOpen = true;
            PhotonNetwork.JoinOrCreateRoom("Spaace", roomOptions, TypedLobby.Default);
            isConnecting = true;
        }

        // the following methods are implemented to give you some context. re-implement them as needed.
        public override void OnDisconnected(DisconnectCause cause)
        {
            Debug.LogError("OnDisconnected(" + cause + ")");
            isConnecting = false;
            isConnected = false;
            disconnectCount++;

            PhotonNetwork.QuickResends = UnityEngine.Random.Range(2, 10);
            PhotonNetwork.MaxResendsBeforeDisconnect = UnityEngine.Random.Range(2, 100);
            PhotonNetwork.NetworkingClient.LoadBalancingPeer.MaximumTransferUnit = UnityEngine.Random.Range(10, 1000);

        }

        public override void OnJoinedRoom()
        {
            Debug.Log("OnJoinedRoom() called by PUN. Now this client is in a room. From here on, your game would be running.");
            isConnecting = false;
            isConnected = true;

            // Uncomment the following lines if you want to confirm mesh sending works from the editor: 
            //Mesh rand = randomMesh.generateRandomGarbageMesh(100);
            //PhotonMeshTransfer.getSingleton().sendMesh(Vector3.zero, this.transform.rotation, rand);
            MeshDataGatherer.S.sendAllMeshes(); 
        }
    }
}
