//using System.Collections;
//using System.Collections.Generic;
using UnityEngine;

public class Health : MonoBehaviour
{
    #region VARIABLES

    [SerializeField] private float health = 100.0f;
    [SerializeField] private float iFrameTime = 1.0f;

    [Header("Is character controller by player or AI?")]
    [SerializeField] private bool getPlayerInput = false;

    [Header("Getting movement controls enables knockback!")]
    [SerializeField] private bool getMovementController = false;

    [Header("Set Game Over Menu only on player!")]
    [SerializeField] private GameOverMenu gameOverMenu = null;

    [Header("Set hurt flash only on player!")]
    [SerializeField] private HurtFlash hurtFlash = null;

    private bool isDead = false;
    private float iFrameTimeTemp = 0.0f;
    private Movement movementController;
    private PlayerInput input;

    #endregion

    #region UNITY_DEFAULT_METHODS

    void Start()
    {
        if (getMovementController)
        {
            if (GetComponent<Movement>() != null)
            {
                movementController = GetComponent<Movement>();
            }
            else
            {
                Debug.LogError(this + " tried to find a movement controller, but failed!");
                getMovementController = false;
            }
        }

        if (getPlayerInput)
        {
            if (GetComponent<PlayerInput>() != null)
            {
                input = GetComponent<PlayerInput>();
            }
            else
            {
                Debug.LogError(this + " tried to find a player input controller, but failed!");
                getPlayerInput = false;
            }
        }
    }

    void Update()
    {
        if (iFrameTimeTemp > 0.0f)
        {
            iFrameTimeTemp -= Time.deltaTime;
        }
    }

    #endregion

    #region CUSTOM_METHODS

    public bool CheckIfDead()
    {
        return isDead;
    }

    void CheckHealth()
    {
        if (health <= 0.0f)
        {
            Dead();
        }
    }

    public void Hit(float amount)
    {
        if (!isDead)
        {
            if (iFrameTimeTemp <= 0.0f)
            {
                iFrameTimeTemp = iFrameTime;
                health -= amount;
                if (hurtFlash != null)
                {
                    hurtFlash.Flash();
                }
                CheckHealth();
            }
        }
    }

    public void Heal(float amount)
    {
        if (!isDead)
        {
            health += amount;
            CheckHealth();
        }
    }

    public void Slay()
    {
        if (!isDead)
        {
            if (hurtFlash != null)
            {
                hurtFlash.Flash();
            }
            Dead();
        }
    }

    void Dead()
    {
        health = 0.0f;
        isDead = true;
        if (getPlayerInput)
        {
            input.EnableControls(false);
        }
        if (gameOverMenu != null)
        {
            gameOverMenu.Activate();
        }
    }

    #endregion
}
