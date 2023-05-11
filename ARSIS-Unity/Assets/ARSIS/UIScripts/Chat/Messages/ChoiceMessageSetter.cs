using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class ChoiceMessageSetter : MonoBehaviour, IMessageSetter
{
    [SerializeField]
    private GameObject _choicePrefab;
    public void SetMessage(string message)
    {
        //csv list of choices
        string[] choices = message.Split(',');
        //create a choice button for each choice
        foreach (string choice in choices)
        {
            GameObject choiceButton = Instantiate(_choicePrefab, transform);
            choiceButton.transform.GetChild(1).GetComponent<TMPro.TMP_Text>().text = choice;
            choiceButton.GetComponent<Button>().onClick.AddListener(() => ChatQueryManager.SendChoiceResponse("{\"content\":\"" + choice + "\"}"));
        }
    }
}