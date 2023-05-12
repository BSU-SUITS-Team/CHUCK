using UnityEngine;
using UnityEngine.XR.WSA;

public class SpatialMapping : MonoBehaviour
{
    /// Allows this class to behave like a singleton
    public static SpatialMapping Instance;

    /// Used by the GazeCursor as a property with the Raycast call
    internal static int PhysicsRaycastMask;

    /// The layer to use for spatial mapping collisions
    internal int physicsLayer = 31;

    /// Creates environment colliders to work with physics
    //private SpatialMappingCollider spatialMappingCollider;

    /// Initializes this class
    private void Awake()
    {
        // Allows this instance to behave like a singleton
        Instance = this;
    }

    /// Runs at initialization right after Awake method
    void Start()
    {
        // Initialize and configure the collider
        //spatialMappingCollider = gameObject.GetComponent<SpatialMappingCollider>();
        //spatialMappingCollider.surfaceParent = this.gameObject;
        //spatialMappingCollider.freezeUpdates = false;
        //spatialMappingCollider.layer = physicsLayer;

        // define the mask
        PhysicsRaycastMask = 1 << physicsLayer;

        // set the object as active one
        gameObject.SetActive(true);
    }
}
