using UnityEngine;

namespace ARSIS.UI
{
    public class Menu : MonoBehaviour
    {
        public void InstantiatePrefab(GameObject prefab)
        {
            GameObject existing = GameObject.Find(prefab.name);
            if (existing != null) Destroy(existing);
            HololensCommandManager.OpenPrefab(prefab);
        }
    }
}
