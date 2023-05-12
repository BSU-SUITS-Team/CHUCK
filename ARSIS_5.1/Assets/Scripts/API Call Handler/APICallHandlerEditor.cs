

using System.Collections;
using System.Collections.Generic;
using SystemTasks = System.Threading.Tasks;
using UnityEngine;
using UnityEditor;

#if UNITY_EDITOR

[CustomEditor(typeof(APICallHandler))]

public class APICallHandlerEditor : Editor
{
    private string route = "/api/lsar";
    private string name = "ARSIS";
    private string room = "8";

   
    public override async void OnInspectorGUI()
    {
        DrawDefaultInspector();
        APICallHandler myTarget = (APICallHandler)target;

        this.route = EditorGUILayout.TextField("Route", this.route);
        this.name = EditorGUILayout.TextField("Name", this.name);
        this.room = EditorGUILayout.TextField("Room", this.room);

#region buttons
        if (GUILayout.Button("Register ARSIS"))
        {
            myTarget.CreateUser(name, room);
        }

        if (GUILayout.Button("Call API"))
        {
            myTarget.GetAPI();
        }

        if (GUILayout.Button("Get Request To Route"))
        {
            myTarget.SendGetRequest(this.route);
        }

        if (GUILayout.Button("Post Request To Route"))
        {
            WWWForm body = new WWWForm();

            foreach (KeyValuePair<string, string> entry in myTarget.requestFields) {
                body.AddField(entry.Key, entry.Value);
            }

            myTarget.SendPostRequest(this.route, body);
        }

        if (GUILayout.Button("Get Simulation Data in Room"))
        {
            myTarget.GetSimulationStateInRoom(myTarget.requestFields["room"]);
        }

        if (GUILayout.Button("Get Location of ID"))
        {
            LocationData received = await myTarget.GetLocationDataById(myTarget.requestFields["id"]);

            if (received.users.Length != 0) {
                foreach (UserLocation user in received.users) {
                    Debug.Log($"User: {user.user}, Latitude: {user.latitude}, Longitude: {user.longitude}");
                }
            }
        }

        if (GUILayout.Button("Get Location of Room"))
        {
            LocationData received = await myTarget.GetLocationDataByRoom(myTarget.requestFields["room"]);

            if (received.users.Length != 0) {
                foreach (UserLocation user in received.users) {
                    Debug.Log($"User: {user.user}, Latitude: {user.latitude}, Longitude: {user.longitude}");
                }
            }
        }





        if (GUILayout.Button("Get LSAR of ID"))
        {
            LSARMessage received = await myTarget.GetLSARDataById(myTarget.requestFields["id"]);
            Debug.Log($"Sender: {received.sender}, Latitude: {received.encoded_lat}, Longitude: {received.encoded_lon}");
        }

        if (GUILayout.Button("Get LSAR of Room"))
        {
            LSARData received = await myTarget.GetLSARDataByRoom(myTarget.requestFields["room"]);
            Debug.Log("checking in room " + myTarget.requestFields["room"]);
            if (received.messages.Length != 0)
            {
                foreach (LSARMessage message in received.messages)
                {
                    Debug.Log($"ID: {message.id}, Sender: {message.sender}, Latitude: {message.encoded_lat}, Longitude: {message.encoded_lon}");
                }
            }
        }



#endregion

    }
}


#endif