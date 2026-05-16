using UnityEngine;

namespace ARSIS.UI
{
    public class Menu : MonoBehaviour
    {
        public void InstantiatePrefab(GameObject prefab)
        {
            HololensCommandManager.OpenPrefab(prefab);
        }
    }
}
