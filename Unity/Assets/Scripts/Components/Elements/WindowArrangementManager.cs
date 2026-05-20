using System.Collections.Generic;
using UnityEngine;

namespace ARSIS.UI
{
    /// <summary>
    /// Tracks open floating windows on a horizontal arc
    /// Every slot sits at the same radius from the user's head; each additional
    /// window is rotated <see cref="ArcStepDegrees"/> to the right. The layout
    /// resets when all windows are closed.
    /// </summary>
    internal static class WindowArrangementManager
    {
        /// <summary>
        /// Angular separation (degrees) between adjacent window slots, measured
        /// around the world-up axis from the user's head. Increase to spread
        /// windows apart; decrease to pack them tighter.
        /// </summary>
        public static float ArcStepDegrees = 40f;

        private static readonly List<FloatingMenuFromPrefab> _orderedWindows
            = new List<FloatingMenuFromPrefab>();

        // Arc geometry — established once when the first window opens.
        private static Vector3 _arcCenter;       // user's head position at anchor time
        private static Vector3 _layoutForward;   // flattened camera-forward (slot 0 direction)
        private static float   _radius;          // distanceMeters of first window
        private static float   _heightOffset;    // heightOffsetMeters of first window

        private static bool _anchorEstablished;

        // Tracks which slot to assign the next window displaced from center.
        // Starts at -1 (left); after that increments through +1, +2, +3 ...
        private static int _nextDisplacementSlot = -1;

        public static bool IsAnchorEstablished => _anchorEstablished;

        /// <summary>
        /// Register a new window into the layout. On first call, establishes the
        /// arc geometry from the camera's current pose. Returns the slot index
        /// (0 = center, 1 = one step right, etc.).
        /// </summary>
        public static int RegisterWindow(
            FloatingMenuFromPrefab window,
            Camera cam,
            float distanceMeters,
            float heightOffsetMeters)
        {
            if (!_anchorEstablished)
                EstablishAnchor(cam, distanceMeters, heightOffsetMeters);

            // Only the current center window moves — all others stay where they landed.
            if (_orderedWindows.Count > 0)
            {
                int displaced = _nextDisplacementSlot;
                _nextDisplacementSlot = (_nextDisplacementSlot == -1) ? 1 : _nextDisplacementSlot + 1;
                _orderedWindows[0].UpdateLayoutSlotAndReposition(displaced);
            }

            _orderedWindows.Insert(0, window);
            return 0;
        }

        /// <summary>
        /// Promotes an existing side-slot window to center. The current center window
        /// is displaced to the next available side slot, all other windows stay put.
        /// </summary>
        public static void PromoteToCenter(FloatingMenuFromPrefab window, int vacatedSlot)
        {
            int idx = _orderedWindows.IndexOf(window);
            if (idx <= 0) return; // not in layout, or already center

            // Swap: center takes the promoted window's old slot. No new slot is created
            // and _nextDisplacementSlot is not advanced.
            _orderedWindows[0].UpdateLayoutSlotAndReposition(vacatedSlot);

            _orderedWindows.RemoveAt(idx);
            _orderedWindows.Insert(0, window);
            window.UpdateLayoutSlotAndReposition(0);
        }

        /// <summary>
        /// Remove a window from the layout. If the list becomes empty the anchor
        /// is cleared so the next open re-centers on the current camera pose.
        /// </summary>
        public static void UnregisterWindow(FloatingMenuFromPrefab window)
        {
            _orderedWindows.Remove(window);
            if (_orderedWindows.Count == 0)
            {
                _anchorEstablished = false;
                _arcCenter = Vector3.zero;
                _layoutForward = Vector3.forward;
                _radius = 0f;
                _heightOffset = 0f;
                _nextDisplacementSlot = -1;
            }
        }

        /// <summary>
        /// Returns the world-space center position for the given slot index.
        /// All slots are at the same radius from the arc center, so no window
        /// ever appears "behind" another.
        /// </summary>
        public static Vector3 GetSlotWorldPosition(int slotIndex)
        {
            // Negative slotIndex rotates left of center; positive rotates right.
            float angleDeg = slotIndex * ArcStepDegrees;
            Vector3 direction = Quaternion.AngleAxis(angleDeg, Vector3.up) * _layoutForward;
            return _arcCenter + direction * _radius + Vector3.up * _heightOffset;
        }

        private static void EstablishAnchor(Camera cam, float distanceMeters, float heightOffsetMeters)
        {
            // Flatten the camera's forward onto the horizontal plane — identical
            // convention to FloatingMenuFromPrefab.ApplyPlacement.
            Vector3 forward = Vector3.ProjectOnPlane(cam.transform.forward, Vector3.up);
            if (forward.sqrMagnitude < 0.0001f)
                forward = cam.transform.forward;
            forward.Normalize();

            // The arc pivots around the user's head. Storing the camera position
            // (not the first window's world position) ensures every slot is at
            // exactly distanceMeters from the user — no depth offset between slots.
            _arcCenter      = cam.transform.position;
            _layoutForward  = forward;
            _radius         = distanceMeters;
            _heightOffset   = heightOffsetMeters;

            _anchorEstablished = true;
        }

        // Clears static state when domain reload is disabled in Play Mode settings.
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void ResetOnDomainReload()
        {
            _orderedWindows.Clear();
            _anchorEstablished = false;
            _arcCenter            = Vector3.zero;
            _layoutForward        = Vector3.forward;
            _radius               = 0f;
            _heightOffset         = 0f;
            _nextDisplacementSlot = -1;
        }
    }
}
