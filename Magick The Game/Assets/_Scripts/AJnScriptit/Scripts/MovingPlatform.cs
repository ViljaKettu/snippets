using UnityEngine;

public class MovingPlatform : MonoBehaviour
{
    [SerializeField] private float moveTime = 5.0f;
    [SerializeField] private float moveSpeed = 1.0f;
    private float moveTimeTemp = 0.0f;
    private bool moveDown = false;

    // Start is called before the first frame update
    void Start()
    {
        moveTimeTemp = moveTime;
    }

    // Update is called once per frame
    void Update()
    {
        if (moveTimeTemp > 0.0f)
        {
            transform.position += (moveDown ? Vector3.down + Vector3.left : Vector3.up + Vector3.right) * moveSpeed * Time.deltaTime;
            moveTimeTemp -= Time.deltaTime;
        }
        else
        {
            moveDown = !moveDown;
            moveTimeTemp = moveTime;
        }
    }
}
