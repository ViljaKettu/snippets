using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Enemy : MonoBehaviour
{
    const float minPathUpdateTime = .2f;
    const float pathUpdateMoveTreshold = 0.5f;

    public Transform target;

    public float speed = 20;
    public float fallSpeed = 20;
    public float turnSpeed = 5;
    public float turnDistance = 5;
    public float stoppingDistance = 10;
    public float height = 1f;
    public float heightPadding = 0.1f;
    public float maxGroundAngle = 145;
    public float attackRange = 2;

    float groundAngle;
    float angle;
    float radius;
    float distanceToGround;
    float closestDistToEnemy;
    float closestDistToPlayer;
    float smoothStep;

    string floorName;

    CapsuleCollider collider = new CapsuleCollider();

    RaycastHit hitDown;
    RaycastHit hitInfo;

    public LayerMask ground;

    bool bCollided = false;
    bool bGrounded;
    bool bMovingToPlayer = false;

    Vector3 currentNormal = Vector3.up;
    Vector3 currentAngle;
    Vector3 forward;
    Vector3 targetPosition;
    Vector3 surfaceNormal = Vector3.down;
    Vector3 colPoint;

    GameObject[] ramps;
    GameObject[] rampWaypoints;
    GameObject floor;

    Path path;

    private void Start()
    {
        currentAngle = transform.eulerAngles;
        collider = GetComponent<CapsuleCollider>();
        radius = collider.radius * 0.9f;

        ramps = GameObject.FindGameObjectsWithTag("Ramp");
        rampWaypoints = GameObject.FindGameObjectsWithTag("Waypoint");
    }

    private void Update()
    {
        smoothStep = 1 * Time.deltaTime;

        DrawDebugLines();

        CalculateForward();
        CalculateGroundAngle();
        CheckGround();
        CheckCollision();
        ApplyGravity();


        if (!bMovingToPlayer && bGrounded && !bCollided)
        {
            if (groundAngle >= maxGroundAngle)
            {
                return;
            }

            GetTargetToMoveTo();
            targetPosition = target.position;
            StartCoroutine(UpdatePath());

            if (targetPosition == target.position)
            {
                bMovingToPlayer = true;
            }
            else
            {
                print("moving to ramp");

            }
        }

        if (bMovingToPlayer && targetPosition != target.position)
        {
            bMovingToPlayer = false;
        }

        if (bCollided)
        {
            //move to avoidance object
            MoveAroundObstacle();
        }
    }

    public void CalculateGroundAngle()
    {
        if (!bGrounded)
        {
            groundAngle = 90;
            return;
        }

        groundAngle = Vector3.Angle(hitDown.normal, transform.forward);

        print("ground angle is: " + groundAngle);

    }

    public void CheckGround()
    {
        if (Physics.Raycast(transform.position, Vector3.down, out hitInfo, height + heightPadding, ground))
        {
            //TODO: fix stutter caused by lerp -> other way to do this?
            transform.position = Vector3.Lerp(transform.position, transform.position + Vector3.up * height, 0.5f * Time.fixedDeltaTime);

            bGrounded = true;
        }
        else
        {
            bGrounded = false;
        }
    }

    public void ApplyGravity()
    {
        if (!bGrounded)
        {
            transform.position += Physics.gravity * Time.deltaTime;
        }
    }

    public void CalculateForward()
    {
        if (!bGrounded)
        {
            forward = transform.forward;
            currentNormal = Vector3.up;
            return;
        }

        forward = Vector3.Cross(transform.right, hitInfo.normal);
    }

    public void CheckCollision()
    {
        float castDistance;
        castDistance = 1f;

        //initialize rays
        Ray rayForward = new Ray(transform.position, transform.forward);
        Ray rayBack = new Ray(transform.position, -transform.forward);
        Ray rayUp = new Ray(transform.position, transform.up);
        Ray rayDown = new Ray(transform.position, -transform.up);
        Ray rayRight = new Ray(transform.position, transform.right);
        Ray rayLeft = new Ray(transform.position, -transform.right);

        //calculating and setting points and distances for capsule cast (and rays)
        float distanceToPoints = collider.height / 2 - collider.radius;
        Vector3 point1 = transform.position + collider.center + Vector3.up * distanceToPoints;
        Vector3 point2 = transform.position + collider.center - Vector3.up * distanceToPoints;
        float radius = collider.radius;

        //shoot capsuleCast
        RaycastHit[] hits = Physics.CapsuleCastAll(point1, point2, radius * 0.8f, forward, castDistance, ground);

        // check collisions
        foreach (RaycastHit objectHit in hits)
        {
            if (Physics.Raycast(rayDown, out hitInfo, castDistance))
            {
                colPoint = hitInfo.point;

                //check if ground angle allows movement
                if (groundAngle < maxGroundAngle)
                {
                    if (Physics.Raycast(transform.position, -surfaceNormal, out hitDown))
                    {
                        print("ground's angle is: " + groundAngle);
                        surfaceNormal = Vector3.Lerp(surfaceNormal, hitDown.normal, 4 * Time.deltaTime);
                    }

                    //TODO: rotate enemy according to groundAngle
                    //THIS DOESN'T WORK!
                    transform.rotation = Quaternion.Lerp(transform.rotation, Quaternion.LookRotation(Vector3.Cross(transform.right, surfaceNormal), hitInfo.normal), smoothStep);

                    //Vector3 targetAngle = new Vector3(0, );

                    //currentAngle = new Vector3(Mathf.LerpAngle(currentAngle.x, targetAngle.x, Time.deltaTime), Mathf.LerpAngle(currentAngle.y, targetAngle.y, Time.deltaTime), Mathf.LerpAngle(currentAngle.z, targetAngle.z, Time.deltaTime));

                    //transform.eulerAngles = currentAngle;

                    if (Physics.Raycast(transform.position, Vector3.down, out hitInfo, height + heightPadding, ground))
                    {
                        transform.position = Vector3.Lerp(transform.position, transform.position + Vector3.up * height, 1 * Time.deltaTime);
                    }
                }
            }

            //check direction and determinate angle for normal and colPoint
            if (Physics.Raycast(rayForward, out hitInfo, (castDistance)))
            {
                print("front collision");
                if (Vector3.Angle(objectHit.normal, hitInfo.normal) > 5)
                {
                    currentNormal = objectHit.normal;
                    colPoint = objectHit.point;
                }
                else
                {
                    currentNormal = hitInfo.normal;
                    colPoint = hitInfo.point;
                }

                bCollided = true;
            }

            //Keep enemy at distance from the object collided with
            if (Vector3.Distance(transform.position, colPoint) < radius + heightPadding)
            {
                transform.position = Vector3.Lerp(transform.position, transform.position + currentNormal * height, 0.5f * Time.deltaTime);
            }
        }
    }

    public void MoveAroundObstacle()
    {
        Transform closest = transform;
        float distance = Mathf.Infinity;

        //go through each ramp to find closest one to both enemy and player
        for (int i = 0; i < rampWaypoints.Length; i++)
        {
            //get distance between player and ramp
            distance = Vector3.Distance(transform.position, rampWaypoints[i].transform.position);

            if (distance < closestDistToEnemy)
            {
                closest = rampWaypoints[i].transform;
                closestDistToEnemy = distance;
            }
        }

        print("distance to waypoint: " + Vector3.Distance(transform.position, closest.transform.position));

        transform.position = Vector3.MoveTowards(transform.position, closest.transform.position, Time.deltaTime * speed);

        if (Vector3.Distance(transform.position, closest.transform.position) <= 0.2f)
        {
            print("can move now");
            bCollided = false;
        }
    }

    public void OnPathFound(Vector3[] waypoints, bool bPathSuccessful)
    {
        if (bPathSuccessful)
        {
            path = new Path(waypoints, transform.position, turnDistance, stoppingDistance);
            StopCoroutine("FollowPath");
            StartCoroutine("FollowPath");
        }
    }

    public void GetTargetToMoveTo()
    {
        RaycastHit hit1;
        RaycastHit hit2;
        float castDistance = collider.height / 2;
        Vector3 dir = Vector3.down;

        //check if enemy is on different floor than player
        if (Physics.SphereCast(transform.position, radius, dir, out hit1, castDistance))
        {
            print("enemy is on " + hit1.transform.name);

            if (hit1.transform.parent != null)
            {
                floorName = hit1.transform.parent.name;
            }
            else
            {
                floorName = hit1.transform.name;
            }


            if (Physics.SphereCast(target.position, radius, dir, out hit2, castDistance))
            {
                print("player is on " + hit2.collider.name);
                string parentFloorName;
                if (hit2.transform.parent != null)
                {
                    parentFloorName = hit2.transform.parent.name;
                }
                else
                {
                    parentFloorName = hit2.transform.name;
                }

                if (floorName == parentFloorName)
                {
                    targetPosition = target.position; // set player's position as targetPosition
                }
                else
                {
                    FindClosestRamp();  // floor is different - find ramp to wanted floor
                }
            }
        }
        else
        {
            Debug.Log("not grounded");
        }

    }

    public void FindClosestRamp()
    {
        //which ones are in same floor?
        //which has connection to player's floor?

        // TODO: check y-position of the waypoint to make sure it's on right floor

        Transform closest = transform;
        float distance = Mathf.Infinity;

        closestDistToEnemy = Vector3.Distance(transform.position, target.position);
        closestDistToPlayer = closestDistToEnemy;

        //go through each ramp to find closest one to both enemy and player
        for (int i = 0; i < rampWaypoints.Length; i++)
        {
            //get distance between player and ramp
            distance = Vector3.Distance(transform.position, rampWaypoints[i].transform.position);

            if (distance < closestDistToPlayer)
            {
                closestDistToPlayer = distance;

                for (int j = 0; j < rampWaypoints.Length; j++)
                {
                    //get distance between enemy and ramp
                    distance = Vector3.Distance(target.position, rampWaypoints[j].transform.position);
                    if (distance < closestDistToEnemy)
                    {
                        closest = rampWaypoints[j].transform;
                        closestDistToEnemy = distance;
                    }
                }
            }
        }

        targetPosition = closest.position;
    }

    IEnumerator UpdatePath()
    {
        if (Time.timeSinceLevelLoad < .3f)
        {
            yield return new WaitForSeconds(0.3f);
        }

        //Request path from current position to target        
        PathRequestManager.RequestPath(new PathRequest(transform.position, targetPosition, OnPathFound));

        float sqrMoveTreshhold = pathUpdateMoveTreshold * pathUpdateMoveTreshold;
        Vector3 targetPosOld = targetPosition;

        while (true)
        {
            yield return new WaitForSeconds(minPathUpdateTime);

            if ((targetPosition - targetPosOld).sqrMagnitude > sqrMoveTreshhold)
            {
                PathRequestManager.RequestPath(new PathRequest(transform.position, targetPosition, OnPathFound));
                targetPosOld = targetPosition;
            }
        }
    }

    public void GoUpDownRamp()
    {
        //TODO: moving up and down the ramps
        //check if should go up or down the ramp

        //go up

        //go down

        //make list of rampwaypoints and move from one to other

        //when finished do another target check to find player/next ramp
    }

    IEnumerator FollowPath()
    {
        bool bFollowingPath = true;
        int pathIndex = 0;
        transform.LookAt(path.lookPoints[0]);


        float speedPercent = 1;

        while (bFollowingPath)
        {
            Vector2 position2d = new Vector2(transform.position.x, transform.position.z);

            while (path.turnBoundaries[pathIndex].HasCrossedLine(position2d))
            {
                if (pathIndex == path.finishedLineIndex)
                {
                    bFollowingPath = false;
                    break;
                }
                else
                {
                    pathIndex++;
                }
            }

            if (bFollowingPath)
            {
                if (!bGrounded)
                {
                    bFollowingPath = false;
                    bMovingToPlayer = false;
                }

                if (pathIndex >= path.slowDownIndex && stoppingDistance > 0)
                {
                    speedPercent = Mathf.Clamp01(path.turnBoundaries[path.finishedLineIndex].DistanceFromThePoint(position2d) / stoppingDistance);

                    if (speedPercent < 0.01f || Physics.Raycast(transform.position, targetPosition, attackRange))
                    {
                        bFollowingPath = false;

                        if (!bMovingToPlayer)
                        {
                            print("arrived to ramp");
                        }
                        else
                        {
                            print("say hello to death, mister wizard");
                        }
                    }
                }

                // Rotate towards next pathPoint
                Quaternion targetRotation = Quaternion.LookRotation(path.lookPoints[pathIndex] - transform.position);
                transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, Time.deltaTime * turnSpeed);

                // Keep unit's own y - position while following path
                Vector3 targetPos = new Vector3(path.lookPoints[pathIndex].x, this.transform.position.y, path.lookPoints[pathIndex].z);
                transform.position = Vector3.MoveTowards(transform.position, targetPos, Time.deltaTime * speed * speedPercent);
            }

            transform.LookAt(target);
            yield return null;
        }
    }

    //FOR TESTING ONLY
    //public void OnDrawGizmos()
    //{
    //    if (path != null)
    //    {
    //        path.DrawWithGizmos();
    //    }
    //}

    void DrawDebugLines()
    {
        Debug.DrawLine(transform.position, transform.position + forward * height * 2, Color.blue);
        Debug.DrawLine(transform.position, transform.position - Vector3.up * height, Color.green);
    }
}
