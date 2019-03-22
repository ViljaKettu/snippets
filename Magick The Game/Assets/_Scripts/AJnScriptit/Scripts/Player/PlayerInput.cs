using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerInput : MonoBehaviour
{
    #region VARIABLES

    private Movement playerMovement;
    private ThirdPersonCamera tpCamera;
    [SerializeField] private PauseMenu pauseMenu = null;
    [SerializeField] private GameObject crosshair = null;

    #endregion

    #region UNITY_DEFAULT_METHODS

    void Awake()
    {
        playerMovement = GetComponent<Movement>();
        tpCamera = GetComponent<ThirdPersonCamera>();
    }

    void Update()
    {
        playerMovement.SetInput("Move", new Vector2(Input.GetAxis("Horizontal"), Input.GetAxis("Vertical")));
        playerMovement.SetInput("Jump", Input.GetButtonDown("Jump"));
        playerMovement.SetInput("Dash", Input.GetButtonDown("Fire3"));

        if (Input.GetButtonDown("Cancel"))
        {
            if (pauseMenu != null)
            {
                pauseMenu.FlipPauseState(this);
            }
        }
    }

    void OnEnable()
    {
        EnableControls(true);
    }

    void OnDisable()
    {
        EnableControls(false);
    }

    #endregion

    #region CUSTOM_METHODS

    public void EnableControls(bool b)
    {
        Cursor.lockState = b ?
            CursorLockMode.Locked
            : CursorLockMode.None;
        Cursor.visible = !b;

        playerMovement.enableMovement = b;
        tpCamera.enableLooking = b;

        if (crosshair != null)
        {
            crosshair.SetActive(b);
        }
    }

    #endregion
}
