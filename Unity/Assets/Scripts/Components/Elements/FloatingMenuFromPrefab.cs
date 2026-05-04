using System.Collections.Generic;
using UnityEngine;

namespace ARSIS.UI
{
    /// <summary>
    /// When a prefab with this component is opened via <see cref="Menu.InstantiatePrefab"/>,
    /// Unity instantiates that prefab asset the first time (a disabled instance of the same prefab already in the
    /// scene hierarchy is not enabled or moved). Later opens reuse that one instance: SetActive + placement.
    /// </summary>
    [DisallowMultipleComponent]
    public class FloatingMenuFromPrefab : MonoBehaviour
    {
        private static readonly Dictionary<int, FloatingMenuFromPrefab> ByPrefabId = new();

        [Tooltip("Meters in front of the camera along the horizontal look direction.")]
        [SerializeField]
        private float distanceMeters = 0.85f;

        [Tooltip("Extra world-space Y (meters) added to the placement position.")]
        [SerializeField]
        private float heightOffsetMeters;

        [Tooltip("If set, used for placement instead of Camera.main / first active camera.")]
        [SerializeField]
        private Camera placementCameraOverride;

        [Tooltip("Extra yaw (degrees, world up) after facing the user. World-space UI often needs 180 so the readable side (-Z) points at the camera.")]
        [SerializeField]
        private float yawOffsetDegrees = 180f;

        [Tooltip("Optional pitch tilt (degrees) after yaw. Rarely needed.")]
        [SerializeField]
        private float pitchOffsetDegrees;

        private int _registeredPrefabInstanceId = -1;

        public static void OpenOrFocus(GameObject prefabAsset)
        {
            if (prefabAsset == null)
                return;

            int key = prefabAsset.GetInstanceID();
            if (ByPrefabId.TryGetValue(key, out FloatingMenuFromPrefab existing) && existing != null)
            {
                existing.gameObject.SetActive(true);
                existing.ApplyPlacement();
                return;
            }

            GameObject instance = Object.Instantiate(prefabAsset);
            FloatingMenuFromPrefab comp = instance.GetComponentInChildren<FloatingMenuFromPrefab>(true);
            if (comp == null)
            {
                Debug.LogWarning(
                    $"FloatingMenuFromPrefab: Prefab '{prefabAsset.name}' has no FloatingMenuFromPrefab — add it on the navigation root.");
                return;
            }

            comp.Register(key);
            comp.ApplyPlacement();
        }

        private void Register(int prefabAssetInstanceId)
        {
            _registeredPrefabInstanceId = prefabAssetInstanceId;
            ByPrefabId[prefabAssetInstanceId] = this;
        }

        private void OnDestroy()
        {
            if (_registeredPrefabInstanceId >= 0 &&
                ByPrefabId.TryGetValue(_registeredPrefabInstanceId, out FloatingMenuFromPrefab owner) &&
                owner == this)
                ByPrefabId.Remove(_registeredPrefabInstanceId);
        }

        /// <summary>Call after enabling to move the whole instance in front of the user.</summary>
        public void ApplyPlacement()
        {
            Camera cam = placementCameraOverride != null && placementCameraOverride.isActiveAndEnabled
                ? placementCameraOverride
                : ResolveActiveCamera();
            if (cam == null)
            {
                Debug.LogWarning("FloatingMenuFromPrefab: No camera found for placement.");
                return;
            }

            Transform root = transform.root;
            Vector3 forward = Vector3.ProjectOnPlane(cam.transform.forward, Vector3.up);
            if (forward.sqrMagnitude < 0.0001f)
                forward = cam.transform.forward;
            forward.Normalize();

            Vector3 pos = cam.transform.position + forward * distanceMeters + Vector3.up * heightOffsetMeters;
            Quaternion face = FacingUserRotation(pos, cam.transform.position);
            face *= Quaternion.Euler(pitchOffsetDegrees, yawOffsetDegrees, 0f);
            root.SetPositionAndRotation(pos, face);

            Navigation[] navs = root.GetComponentsInChildren<Navigation>(true);
            for (int i = 0; i < navs.Length; i++)
                navs[i].NotifyMenuPlacedInFrontOfUser();
        }

        private static Quaternion FacingUserRotation(Vector3 panelPosition, Vector3 headPosition)
        {
            Vector3 toUser = headPosition - panelPosition;
            toUser.y = 0f;
            if (toUser.sqrMagnitude < 0.0001f)
                return Quaternion.identity;
            return Quaternion.LookRotation(toUser.normalized, Vector3.up);
        }

        private static Camera ResolveActiveCamera()
        {
            if (Camera.main != null && Camera.main.isActiveAndEnabled)
                return Camera.main;

            Camera[] cameras = Object.FindObjectsOfType<Camera>();
            for (int i = 0; i < cameras.Length; i++)
            {
                Camera c = cameras[i];
                if (c != null && c.isActiveAndEnabled && c.gameObject.activeInHierarchy)
                    return c;
            }

            return null;
        }
    }
}
