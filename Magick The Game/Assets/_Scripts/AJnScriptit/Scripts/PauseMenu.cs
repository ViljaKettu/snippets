using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PauseMenu : MonoBehaviour
{
    [SerializeField] private GameObject pauseObject = null;

    private bool isPaused = false;
    private PlayerInput caller;

    void Start()
    {
        pauseObject.SetActive(isPaused);
    }

    //→ PlayerInput calls FlipPauseState
    // → Game pauses, pause menu gets displayed
    //  → FlipPauseState calls back to PlayerInput and enables/disables controls
    public void FlipPauseState(PlayerInput pi)
    {
        caller = pi;
        isPaused = !isPaused;
        pauseObject.SetActive(isPaused);
        Time.timeScale = isPaused ? 0.0f : 1.0f;
        pi.EnableControls(!isPaused);
    }

    //Use previous caller if no PlayerInput is specified
    //(example: pressing an UI button resumes the game)
    public void FlipPauseState()
    {
        if (caller != null)
        {
            FlipPauseState(caller);
        }
    }

    public bool GetPauseState()
    {
        return isPaused;
    }
}
