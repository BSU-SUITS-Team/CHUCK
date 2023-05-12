using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class ListManager : MonoBehaviour
{
    public GameObject grid;
    public GameObject listItemPrefab; 

    public GameObject addListItem(string text)
    {
        GameObject newListItem = Instantiate(listItemPrefab);
        newListItem.transform.SetParent(grid.transform, false); 
        TextMeshProUGUI textmesh = newListItem.GetComponentInChildren<TextMeshProUGUI>();
        textmesh.text = text;
        return newListItem; 
    }
}
