using System.Collections.Generic;
using UnityEngine.Events;
using UnityEngine.UIElements;
using UnityEngine.XR.ARFoundation;

namespace UnityEngine.XR.Templates.MR
{
    /// <summary>
    /// Utility class used to control various AR features like occlusion, AR bounding boxes, and AR planes.
    /// </summary>
    public class ARFeatureController : MonoBehaviour
    {
        [SerializeField, Tooltip("AR Plane Manager that is in charge of passthrough.")]
        ARCameraManager m_ARCameraManager;

        /// <summary>
        /// AR Camera Manager that is in charge of passthrough.
        /// </summary>
        public ARCameraManager arCameraManager
        {
            get => m_ARCameraManager;
            set => m_ARCameraManager = value;
        }

        [SerializeField, Tooltip("AR Occlusion Manager that is in charge of changing MR Occlusion Features.")]
        OcclusionManager m_OcclusionManager;

        /// <summary>
        /// AR Occlusion Manager that is in charge of changing MR Occlusion Features.
        /// </summary>
        public OcclusionManager occlusionManager
        {
            get => m_OcclusionManager;
            set => m_OcclusionManager = value;
        }

        [SerializeField, Tooltip("AR Plane Manager that is in charge of spawning new AR Plane prefabs into the scene.")]
        ARPlaneManager m_PlaneManager;

        /// <summary>
        /// AR Plane Manager that is in charge of spawning new AR Plane prefabs into the scene.
        /// </summary>
        public ARPlaneManager planeManager
        {
            get => m_PlaneManager;
            set => m_PlaneManager = value;
        }

        [SerializeField, Tooltip("Toggle that dictates whether AR Planes should be visualized at runtime.")]
        bool m_PlaneVisualsEnabled = true;

        /// <summary>
        /// Toggle that dictates whether AR Planes should be visualized at runtime.
        /// </summary>
        public bool PlaneVisualsEnabled => m_PlaneVisualsEnabled;

        [SerializeField, Tooltip("AR Bounding Box Manager that is in charge of spawning new AR Bounding Box prefabs into the scene")]
        ARBoundingBoxManager m_BoundingBoxManager;

        /// <summary>
        /// AR Bounding Box Manager that is in charge of spawning new AR Bounding Box prefabs into the scene.
        /// </summary>
        public ARBoundingBoxManager BoundingBoxManager
        {
            get => m_BoundingBoxManager;
            set => m_BoundingBoxManager = value;
        }

        [SerializeField, Tooltip("Toggle that dictates whether AR Bounding Boxes should be visualized at runtime.")]
        bool m_BoundingBoxVisualsEnabled = true;

        /// <summary>
        /// Toggle that dictates whether AR Bounding Boxes should be visualized at runtime.
        /// </summary>
        public bool BoundingBoxVisualsEnabled => m_BoundingBoxVisualsEnabled;

        [SerializeField, Tooltip("Toggle that dictates whether AR Bounding Box visualizations should show additional debug information.")]
        bool m_BoundingBoxDebugInfoEnabled = true;

        /// <summary>
        /// Toggle that dictates whether AR Bounding Box visualizations should show additional debug information.
        /// </summary>
        public bool boundingBoxDebugInfoEnabled => m_BoundingBoxDebugInfoEnabled;

        [Header("Feature Changed Events")]

        [SerializeField]
        UnityEvent<bool> m_OnARPassthroughFeatureChanged = new UnityEvent<bool>();

        public UnityEvent<bool> onARPassthroughFeatureChanged => m_OnARPassthroughFeatureChanged;

        [SerializeField]
        UnityEvent<bool> m_OnARPlaneFeatureChanged = new UnityEvent<bool>();

        public UnityEvent<bool> onARPlaneFeatureChanged => m_OnARPlaneFeatureChanged;

        [SerializeField]
        UnityEvent<bool> m_OnARPlaneFeatureVisualizationChanged = new UnityEvent<bool>();

        public UnityEvent<bool> onARPlaneFeatureVisualizationChanged => m_OnARPlaneFeatureVisualizationChanged;

        [SerializeField]
        UnityEvent<bool> m_OnARBoundingBoxFeatureChanged = new UnityEvent<bool>();

        public UnityEvent<bool> onARBoundingBoxFeatureChanged => m_OnARBoundingBoxFeatureChanged;

        [SerializeField]
        UnityEvent<bool> m_OnARBoundingBoxFeatureVisualizationChanged = new UnityEvent<bool>();

        public UnityEvent<bool> onARBoundingBoxFeatureVisualizationChanged => m_OnARBoundingBoxFeatureVisualizationChanged;

        [SerializeField]
        UnityEvent<bool> m_OnARBoundingBoxFeatureDebugVisualizationChanged = new UnityEvent<bool>();

        public UnityEvent<bool> onARBoundingBoxFeatureDebugVisualizationChanged => m_OnARBoundingBoxFeatureDebugVisualizationChanged;

        /// <summary>
        /// Allows access to easily see if the AR Features are enabled and there is at least one bounding box
        /// </summary>
        /// <returns>Will return True if there is 1 or more AR Bounding Boxes found in the AR Scene.</returns>
        public bool HasBoundingBoxes() => m_BoundingBoxManager != null && m_BoundingBoxManager.trackables.count > 0;

        bool m_BoundingBoxManagerEnabled;
        bool m_PlaneManagerEnabled;

        readonly List<ARPlane> m_ARPlanes = new List<ARPlane>();
        readonly Dictionary<ARPlane, ARPlaneMeshVisualizer> m_ARPlaneMeshVisualizers = new Dictionary<ARPlane, ARPlaneMeshVisualizer>();

        readonly List<ARBoundingBox> m_ARBoundingBoxes = new List<ARBoundingBox>();
        readonly Dictionary<ARBoundingBox, ARBoundingBoxDebugVisualizer> m_ARBoundingBoxVisualizers = new Dictionary<ARBoundingBox, ARBoundingBoxDebugVisualizer>();

        /// <summary>
        /// Functionally turns AR Passthrough on and off in the scene.
        /// </summary>
        /// <param name="enabled">Whether to enable or disable passthrough.</param>
        public void TogglePassthrough(bool enabled)
        {
            if (m_ARCameraManager == null)
                return;

            m_ARCameraManager.enabled = enabled;
            m_OnARPassthroughFeatureChanged?.Invoke(enabled);
        }

        /// <summary>
        /// Functionally turns AR Planes on and off in a scene.
        /// </summary>
        /// <param name="enabled">Whether to enable or disable the currently detected planes.</param>
        public void TogglePlanes(bool enabled)
        {
            if (m_PlaneManager == null)
                return;

            m_PlaneManagerEnabled = enabled;
            m_OnARPlaneFeatureChanged?.Invoke(m_PlaneManagerEnabled);

            if (m_PlaneManagerEnabled)
            {
                m_PlaneManager.enabled = m_PlaneManagerEnabled;
                m_PlaneManager.SetTrackablesActive(m_PlaneManagerEnabled);
                m_PlaneManager.trackablesChanged.AddListener(OnPlaneChanged);
            }
            else
            {
                m_PlaneManager.trackablesChanged.RemoveListener(OnPlaneChanged);
                m_PlaneManager.SetTrackablesActive(m_PlaneManagerEnabled);
                m_PlaneManager.enabled = m_PlaneManagerEnabled;
            }
        }

        /// <summary>
        /// Toggles the AR plane visualizations in a scene.
        /// </summary>
        /// <param name="enabled">If <see langword="true"/>, AR plane visualizations will be enabled. Otherwise AR plane visualizations be disabled.</param>
        public void TogglePlaneVisualization(bool enabled)
        {
            if (m_PlaneManager == null)
                return;

            m_PlaneVisualsEnabled = enabled;
            m_OnARPlaneFeatureVisualizationChanged?.Invoke(m_PlaneVisualsEnabled);

            var trackables = m_PlaneManager.trackables;
            if (trackables.count != m_ARPlanes.Count)
            {
                RefreshAllPlanes();
            }
            else
            {
                foreach (var visualizer in m_ARPlaneMeshVisualizers.Values)
                {
                    visualizer.enabled = m_PlaneVisualsEnabled;
                }
            }
        }

        /// <summary>
        /// Functionally turns AR Bounding Boxes on and off in a scene.
        /// </summary>
        /// <param name="enabled">Whether to enable or disable the currently detected bounding boxes.</param>
        public void ToggleBoundingBoxes(bool enabled)
        {
            if (m_BoundingBoxManager == null)
                return;

            m_BoundingBoxManagerEnabled = enabled;
            m_OnARBoundingBoxFeatureChanged?.Invoke(m_BoundingBoxManagerEnabled);

            if (m_BoundingBoxManagerEnabled)
            {
                m_BoundingBoxManager.enabled = m_BoundingBoxManagerEnabled;
                m_BoundingBoxManager.SetTrackablesActive(m_BoundingBoxManagerEnabled);
                m_BoundingBoxManager.trackablesChanged.AddListener(OnBoundingBoxesChanged);
            }
            else
            {
                m_BoundingBoxManager.trackablesChanged.RemoveListener(OnBoundingBoxesChanged);
                m_BoundingBoxManager.SetTrackablesActive(m_BoundingBoxManagerEnabled);
                m_BoundingBoxManager.enabled = m_BoundingBoxManagerEnabled;
            }
        }

        /// <summary>
        /// Toggles the AR Bounding Boxes visualizations in a scene.
        /// </summary>
        /// <param name="enabled">If <see langword="true"/>, AR Bounding Boxes visualizations will be enabled. Otherwise AR Bounding Boxes visualizations be disabled.</param>
        public void ToggleBoundingBoxVisualization(bool enabled)
        {
            if (m_BoundingBoxManager == null)
                return;

            m_BoundingBoxVisualsEnabled = enabled;
            m_OnARBoundingBoxFeatureVisualizationChanged?.Invoke(m_BoundingBoxVisualsEnabled);

            var trackables = m_BoundingBoxManager.trackables;
            if (trackables.count != m_ARBoundingBoxes.Count)
            {
                RefreshAllBoundingBoxes();
            }
            else
            {
                foreach (var visualizer in m_ARBoundingBoxVisualizers.Values)
                {
                    visualizer.enabled = m_BoundingBoxVisualsEnabled;
                    visualizer.ShowDebugInfoCanvas(m_BoundingBoxVisualsEnabled && m_BoundingBoxDebugInfoEnabled);
                }
            }
        }

        /// <summary>
        /// Toggles the visualization of the debug information for AR Bounding Boxes.
        /// </summary>
        /// <param name="enabled">If <see langword="true"/>, debug information will be shown for AR Bounding Boxes. Otherwise, debug information will not be shown.</param>
        public void ToggleDebugInfo(bool enabled)
        {
            if (m_BoundingBoxManager == null)
                return;

            m_BoundingBoxDebugInfoEnabled = enabled;
            m_OnARBoundingBoxFeatureDebugVisualizationChanged?.Invoke(m_BoundingBoxDebugInfoEnabled);

            // If general bounding box visuals are not enabled, do not enable the debug info.
            if (!m_BoundingBoxVisualsEnabled)
                return;

            var trackables = m_BoundingBoxManager.trackables;
            foreach (var trackable in trackables)
            {
                if (trackable.TryGetComponent(out ARBoundingBoxDebugVisualizer visualizer))
                {
                    visualizer.ShowDebugInfoCanvas(m_BoundingBoxDebugInfoEnabled);
                }
            }
        }

        void OnPlaneChanged(ARTrackablesChangedEventArgs<ARPlane> eventArgs)
        {
            if (eventArgs.added.Count > 0)
            {
                foreach (var plane in eventArgs.added)
                {
                    m_ARPlanes.Add(plane);
                    if (plane.TryGetComponent<ARPlaneMeshVisualizer>(out var visualizer))
                    {
                        m_ARPlaneMeshVisualizers.Add(plane, visualizer);
                        visualizer.enabled = m_PlaneVisualsEnabled;
                    }
                }
            }

            if (eventArgs.removed.Count > 0)
            {
                foreach (var plane in eventArgs.removed)
                {
                    var planeGameObject = plane.Value;
                    if (planeGameObject == null)
                        continue;

                    if (m_ARPlanes.Contains(planeGameObject))
                        m_ARPlanes.Remove(planeGameObject);

                    if (m_ARPlaneMeshVisualizers.ContainsKey(planeGameObject))
                        m_ARPlaneMeshVisualizers.Remove(planeGameObject);
                }
            }

            // Fallback if the counts do not match after an update
            if (m_PlaneManager.trackables.count != m_ARPlanes.Count)
            {
                RefreshAllPlanes();
            }
        }

        void RefreshAllPlanes()
        {
            m_ARPlanes.Clear();
            m_ARPlaneMeshVisualizers.Clear();

            foreach (var plane in m_PlaneManager.trackables)
            {
                m_ARPlanes.Add(plane);
                if (plane.TryGetComponent<ARPlaneMeshVisualizer>(out var visualizer))
                {
                    m_ARPlaneMeshVisualizers.Add(plane, visualizer);
                    visualizer.enabled = m_PlaneVisualsEnabled;
                }
            }
        }

        void OnBoundingBoxesChanged(ARTrackablesChangedEventArgs<ARBoundingBox> eventArgs)
        {
            if (eventArgs.added.Count > 0)
            {
                foreach (var box in eventArgs.added)
                {
                    m_ARBoundingBoxes.Add(box);
                    if (box.TryGetComponent<ARBoundingBoxDebugVisualizer>(out var visualizer))
                    {
                        m_ARBoundingBoxVisualizers.Add(box, visualizer);
                        visualizer.enabled = m_BoundingBoxVisualsEnabled;
                        visualizer.ShowDebugInfoCanvas(m_BoundingBoxDebugInfoEnabled && m_BoundingBoxDebugInfoEnabled);
                    }
                }
            }

            if (eventArgs.removed.Count > 0)
            {
                foreach (var box in eventArgs.removed)
                {
                    var boxGameObject = box.Value;
                    if (boxGameObject == null)
                        continue;

                    if (m_ARBoundingBoxes.Contains(boxGameObject))
                        m_ARBoundingBoxes.Remove(boxGameObject);

                    if (m_ARBoundingBoxVisualizers.ContainsKey(boxGameObject))
                        m_ARBoundingBoxVisualizers.Remove(boxGameObject);
                }
            }

            // Fallback if the counts do not match after an update
            if (m_BoundingBoxManager.trackables.count != m_ARBoundingBoxes.Count)
            {
                RefreshAllBoundingBoxes();
            }
        }

        void RefreshAllBoundingBoxes()
        {
            m_ARBoundingBoxes.Clear();
            m_ARBoundingBoxVisualizers.Clear();

            foreach (var box in m_BoundingBoxManager.trackables)
            {
                m_ARBoundingBoxes.Add(box);
                if (box.TryGetComponent<ARBoundingBoxDebugVisualizer>(out var visualizer))
                {
                    m_ARBoundingBoxVisualizers.Add(box, visualizer);
                    visualizer.enabled = m_BoundingBoxVisualsEnabled;
                    visualizer.ShowDebugInfoCanvas(m_BoundingBoxDebugInfoEnabled && m_BoundingBoxDebugInfoEnabled);
                }
            }
        }
    }
}
