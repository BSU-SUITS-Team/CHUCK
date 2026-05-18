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

        [Tooltip("When true, keeps the menu in front of the active camera while enabled. Turn off if this menu should stay fixed after opening.")]
        [SerializeField]
        private bool followCameraWhileOpen;

        [Tooltip("Seconds between follow updates when followCameraWhileOpen is enabled.")]
        [SerializeField]
        private float followUpdateIntervalSeconds = 0.05f;

        [Tooltip("Only re-center while open when the camera has moved at least this many meters since the last placement.")]
        [SerializeField]
        private float followRecenterDistanceMeters = 0.2f;

        [Tooltip("Only re-center while open when camera yaw changes by at least this many degrees since the last placement.")]
        [SerializeField]
        private float followRecenterYawDegrees = 18f;

        [Tooltip("When false, this window always opens centered in front of the camera and is excluded from the horizontal layout row.")]
        [SerializeField]
        private bool participatesInLayout = true;

        private int _registeredPrefabInstanceId = -1;
        private int _layoutSlot = -1;
        private float _followTimer;
        private Vector3 _lastPlacementCameraPos;
        private Vector3 _lastPlacementCameraForward = Vector3.forward;
        private bool _hasPlacementCameraPose;

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

            if (comp.participatesInLayout && comp._layoutSlot < 0)
            {
                Camera cam = ResolveActiveCamera();
                if (cam != null)
                    comp._layoutSlot = WindowArrangementManager.RegisterWindow(
                        comp, cam, comp.distanceMeters, comp.heightOffsetMeters);
            }

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

            if (participatesInLayout && _layoutSlot >= 0)
            {
                WindowArrangementManager.UnregisterWindow(this);
                _layoutSlot = -1;
            }
        }

        private void Update()
        {
            // Layout windows are fixed in world space; follow-camera has no effect on them.
            if (participatesInLayout && _layoutSlot >= 0)
                return;

            if (!followCameraWhileOpen || !gameObject.activeInHierarchy)
                return;
            _followTimer += Time.deltaTime;
            if (_followTimer < Mathf.Max(0.01f, followUpdateIntervalSeconds))
                return;
            _followTimer = 0f;

            Camera cam = ResolvePlacementCamera();
            if (cam == null)
                return;

            if (!_hasPlacementCameraPose)
            {
                ApplyPlacement(notifyNavigation: false, cam);
                return;
            }

            float movedMeters = Vector3.Distance(_lastPlacementCameraPos, cam.transform.position);
            Vector3 lastFlat = Vector3.ProjectOnPlane(_lastPlacementCameraForward, Vector3.up);
            Vector3 nowFlat = Vector3.ProjectOnPlane(cam.transform.forward, Vector3.up);
            if (lastFlat.sqrMagnitude < 1e-6f) lastFlat = Vector3.forward;
            if (nowFlat.sqrMagnitude < 1e-6f) nowFlat = Vector3.forward;
            float yawDelta = Vector3.Angle(lastFlat.normalized, nowFlat.normalized);

            bool movedEnough = movedMeters >= Mathf.Max(0f, followRecenterDistanceMeters);
            bool turnedEnough = yawDelta >= Mathf.Max(0f, followRecenterYawDegrees);
            if (movedEnough || turnedEnough)
                ApplyPlacement(notifyNavigation: false, cam);
        }

        /// <summary>Call after enabling to move the whole instance in front of the user.</summary>
        public void ApplyPlacement()
        {
            ApplyPlacement(notifyNavigation: true, null);
        }

        private Camera ResolvePlacementCamera() =>
            placementCameraOverride != null && placementCameraOverride.isActiveAndEnabled
                ? placementCameraOverride
                : ResolveActiveCamera();

        private void ApplyPlacement(bool notifyNavigation, Camera providedCamera)
        {
            Camera cam = providedCamera != null ? providedCamera : ResolvePlacementCamera();
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

            Vector3 pos;
            if (participatesInLayout && _layoutSlot >= 0 && WindowArrangementManager.IsAnchorEstablished)
                pos = WindowArrangementManager.GetSlotWorldPosition(_layoutSlot);
            else
                pos = cam.transform.position + forward * distanceMeters + Vector3.up * heightOffsetMeters;
            Quaternion face = FacingUserRotation(pos, cam.transform.position);
            face *= Quaternion.Euler(pitchOffsetDegrees, yawOffsetDegrees, 0f);
            root.SetPositionAndRotation(pos, face);
            _lastPlacementCameraPos = cam.transform.position;
            _lastPlacementCameraForward = cam.transform.forward;
            _hasPlacementCameraPose = true;

            if (notifyNavigation)
            {
                Navigation[] navs = root.GetComponentsInChildren<Navigation>(true);
                for (int i = 0; i < navs.Length; i++)
                    navs[i].NotifyMenuPlacedInFrontOfUser();
            }
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
