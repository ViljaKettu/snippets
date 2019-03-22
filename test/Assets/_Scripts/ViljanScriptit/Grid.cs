using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class Grid : MonoBehaviour
{

    public bool bDisplayGrid;

    public LayerMask unWalkable;
    Vector2 gridWorldSize;
    public float nodeRadius;
    public int obstacleProximityPenalty = 10;
    public TerrainType[] walkableTerrains;


    Dictionary<int, int> walkableTerrainDictionary = new Dictionary<int, int>();
    LayerMask walkableMask;
    Node[,] grid;
    GameObject[] floors;
    Renderer renderer = new Renderer();

    float groundSize;
    float nodeDiameter;
    int gridSizeX, gridSizeY;
    int penaltyMin = int.MaxValue;
    int penaltyMax = int.MinValue;


    private void Awake()
    {
        //Get list of floor areas
        if (floors == null)
        {
            floors = GameObject.FindGameObjectsWithTag("Floor");
            print("Amount of floors: " + floors.Length);
        }

       
        //create grid for each floor
        foreach (GameObject floor in floors)
        {
            renderer = floor.GetComponent<Renderer>();

            Debug.Log("Size-x bounds of floor - " + floor.name + " - " + renderer.bounds.size.x);
            Debug.Log("Size-y bounds of floor - " + floor.name + " - " + renderer.bounds.size.z);

            //get size of the floor
            gridWorldSize.x = renderer.bounds.size.x;
            gridWorldSize.y = renderer.bounds.size.z;

            nodeDiameter = nodeRadius * 2;

            //set grid size
            gridSizeX = Mathf.RoundToInt(gridWorldSize.x / nodeDiameter);
            gridSizeY = Mathf.RoundToInt(gridWorldSize.y / nodeDiameter);

            

            CreateGrid(renderer.bounds.center);
        }

        //Go trough walkableTerrains and add them and their terrainpenalty to walkableMask
        foreach (TerrainType terrain in walkableTerrains)
        {
            walkableMask.value = walkableMask | terrain.terrainMask.value;
            walkableTerrainDictionary.Add((int)Mathf.Log(terrain.terrainMask.value, 2), terrain.terrainPenalty);
        }
    }

    public int MaxSize
    {
        get
        {
            return gridSizeX * gridSizeY;
        }
    }

    //create grid over the game area using the center point of the floor
    void CreateGrid(Vector3 center)
    {
        Vector3 worldPoint;

        grid = new Node[gridSizeX, gridSizeY];

        //find bottom left corner of the area
        Vector3 worldBottomLeft = center - Vector3.right * gridWorldSize.x / 2 - Vector3.forward * gridWorldSize.y / 2;

        for (int x = 0; x < gridSizeX; x++)
        {
            for (int y = 0; y < gridSizeY; y++)
            {
                worldPoint = worldBottomLeft + Vector3.right * (x * nodeDiameter + nodeRadius) + Vector3.forward * (y * nodeDiameter + nodeRadius);

                //check if node is walkable
                bool bWalkable = !(Physics.CheckSphere(worldPoint, nodeRadius * 1.2f, unWalkable));


                int movementPenalty = 0;

                if (bWalkable)
                {
                    Ray ray = new Ray(worldPoint + Vector3.up * 50, Vector3.down);
                    RaycastHit hit;
                    if (Physics.Raycast(ray, out hit, 100, walkableMask))
                    {
                        walkableTerrainDictionary.TryGetValue(hit.collider.gameObject.layer, out movementPenalty);

                    }

                    if (!bWalkable)
                    {
                        movementPenalty += obstacleProximityPenalty;
                    }
                }


                grid[x, y] = new Node(bWalkable, worldPoint, x, y, movementPenalty);
            }
        }

        BlurPenaltyMap(3);
    }

    // Blurpelnalty for terrain to priorize centre of the terrain insted of edges
    void BlurPenaltyMap(int blurSize)
    {
        int kernelSize = blurSize * 2 + 1;
        int kernelExtents = (kernelSize - 1) / 2;

        int[,] penaltiesHorizontalPass = new int[gridSizeX, gridSizeY];
        int[,] penaltiesVerticalPass = new int[gridSizeX, gridSizeY];

        // Get horizontal penalties of the grid area
        for (int y = 0; y < gridSizeY; y++)
        {
            for (int x = -kernelExtents; x <= kernelExtents; x++)
            {
                int sampleX = Mathf.Clamp(x, 0, kernelExtents);
                penaltiesHorizontalPass[0, y] += grid[sampleX, y].movementPenalty;
            }

            for (int x = 1; x < gridSizeX; x++)
            {
                int removeIndex = Mathf.Clamp(x - kernelExtents - 1, 0, gridSizeX); //remove previous number from kernel
                int addIndex = Mathf.Clamp(x + kernelExtents, 0, gridSizeX - 1); //add next number to kernel

                // calculate horizontal penalties of the grid
                penaltiesHorizontalPass[x, y] = penaltiesHorizontalPass[x - 1, y] - grid[removeIndex, y].movementPenalty + grid[addIndex, y].movementPenalty;
            }
        }

        // Get vertical penalties of the grid area
        for (int x = 0; x < gridSizeX; x++)
        {
            for (int y = -kernelExtents; y <= kernelExtents; y++)
            {
                int sampleY = Mathf.Clamp(y, 0, kernelExtents);
                penaltiesVerticalPass[x, 0] += penaltiesHorizontalPass[x, sampleY];
            }

            int blurredPenalty = Mathf.RoundToInt((float)penaltiesVerticalPass[x, 0] / (kernelSize * kernelSize));
            grid[x, 0].movementPenalty = blurredPenalty;

            for (int y = 1; y < gridSizeY; y++)
            {
                int removeIndex = Mathf.Clamp(y - kernelExtents - 1, 0, gridSizeX); //remove previous number from kernel
                int addIndex = Mathf.Clamp(y + kernelExtents, 0, gridSizeX - 1); //add next number to kernel

                // calculate vertical penalties of the grid
                penaltiesVerticalPass[x, y] = penaltiesVerticalPass[x, y - 1] - penaltiesHorizontalPass[x, removeIndex] + penaltiesHorizontalPass[x, addIndex];

                //calculate final penalty values
                blurredPenalty = Mathf.RoundToInt((float)penaltiesVerticalPass[x, y] / (kernelSize * kernelSize));
                grid[x, y].movementPenalty = blurredPenalty;

                if (blurredPenalty > penaltyMax)
                {
                    penaltyMax = blurredPenalty;
                }
                if (blurredPenalty < penaltyMin)
                {
                    penaltyMin = blurredPenalty;
                }
            }
        }
    }

    //check neighbouring nodes of the current node
    public List<Node> GetNeighbouringNodes(Node node)
    {
        List<Node> neighbours = new List<Node>();

        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                if (x == 0 && y == 0)
                {
                    continue;
                }

                int checkX = node.gridX + x;
                int checkY = node.gridY + y;

                if (checkX >= 0 && checkX < gridSizeX && checkY >= 0 && checkY < gridSizeY)
                {

                    neighbours.Add(grid[checkX, checkY]);

                }
            }
        }
        return neighbours;
    }

    //convert world position into grid position
    public Node NodeFromWorldPoint(Vector3 worldPosition)
    {
        //convert position to percentile of grid 
        float percentX = (worldPosition.x + gridWorldSize.x / 2) / gridWorldSize.x;
        float percentY = (worldPosition.z + gridWorldSize.y / 2) / gridWorldSize.y;
        percentX = Mathf.Clamp01(percentX);
        percentY = Mathf.Clamp01(percentY);


        int x = Mathf.RoundToInt((gridSizeX - 1) * percentX);
        int y = Mathf.RoundToInt((gridSizeY - 1) * percentY);
        return grid[x, y];
    }

    //draw grid on screen 
    //FOR TESTING ONLY
    void OnDrawGizmos()
    {
        Gizmos.DrawWireCube(transform.position, new Vector3(gridWorldSize.x, 1, gridWorldSize.y));

        if (grid != null && bDisplayGrid)
        {
            foreach (Node node in grid)
            {
                Gizmos.color = Color.Lerp(Color.white, Color.black, Mathf.InverseLerp(penaltyMin, penaltyMax, node.movementPenalty));
                Gizmos.color = (node.bWalkable) ? Gizmos.color : Color.red;
                Gizmos.DrawCube(node.worldPosition, Vector3.one * (nodeDiameter));
            }
        }
    }

    [System.Serializable]
    public class TerrainType
    {
        public LayerMask terrainMask;
        public int terrainPenalty;
    }

}

