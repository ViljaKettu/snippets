using UnityEngine;

public class ThirdPersonCamera : MonoBehaviour
{
    #region VARIABLES

    [HideInInspector] public bool enableLooking = true;
    public bool invertY = false;
    public Vector2 sensitivity = new Vector2(1.0f, 1.0f);

    [SerializeField] private Vector2 minMaxPitch = new Vector2(-85.0f, 85.0f);
    [SerializeField] private Transform cameraPivot = null;

    private Vector3 lookDirection = Vector3.zero;
    
    #endregion

    #region UNITY_DEFAULT_METHODS

    void Start()
    {
        if (cameraPivot == null)
        {
            Debug.LogWarning(this + " is missing a camera pivot reference!");
            this.enabled = false;
        }
    }

    void Update()
    {
        if (enableLooking)
        {
            LookAround(Input.GetAxis("Mouse X"), Input.GetAxis("Mouse Y"));
        }
    }
    
    #endregion

    #region CUSTOM_METHODS

    void LookAround(float x, float y)
    {
        lookDirection.x += y * sensitivity.x * (invertY ? 1.0f : -1.0f);
        lookDirection.y += x * sensitivity.y;

        if (lookDirection.x < minMaxPitch.x) { lookDirection.x = minMaxPitch.x; }
        if (lookDirection.x > minMaxPitch.y) { lookDirection.x = minMaxPitch.y; }

        cameraPivot.localRotation = Quaternion.Euler(lookDirection);
    }

    public Vector3 GetLookDirection()
    {
        return lookDirection;
    }

    #endregion
}
