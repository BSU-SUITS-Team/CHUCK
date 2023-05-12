using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class LocationData : ISerializationCallbackReceiver
{
    public UserLocation[] users;

    public void OnBeforeSerialize() { }
    public void OnAfterDeserialize() { }
}

[System.Serializable]
public class UserLocation : ISerializationCallbackReceiver
{
    public int id;
    public string user;
    public int room;
    public double latitude;
    public double longitude;
    public float altitude;
    public string? accuracy;
    public string? altitudeAccuracy;
    public float heading;
    public float speed;

    public System.DateTime CreatedAt;
    public System.DateTime UpdatedAt;

    [SerializeField] private string createdAt;
    [SerializeField] private string updatedAt;

    public void OnBeforeSerialize()
    {
        createdAt = CreatedAt.ToString("o");
        updatedAt = UpdatedAt.ToString("o");
    }

    public void OnAfterDeserialize()
    {
        System.DateTime.TryParse(createdAt, out CreatedAt);
        System.DateTime.TryParse(updatedAt, out UpdatedAt);
    }
}