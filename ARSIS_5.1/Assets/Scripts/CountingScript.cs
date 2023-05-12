using UnityEngine;
using TMPro;

public class CountingScript : MonoBehaviour
{
    public TextMeshProUGUI countText;
    private int count;

    private void Start()
    {
        count = 1;
        countText.text = count.ToString();
    }

    private void Update()
    {
        if (count <= 1000)
        {
            count++;
            countText.text = count.ToString();
        }
    }
}
