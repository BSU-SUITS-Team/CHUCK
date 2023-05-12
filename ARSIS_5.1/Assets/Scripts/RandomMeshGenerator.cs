using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RandomMeshGenerator : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        this.GetComponent<MeshFilter>().mesh = generateRandomGarbageMesh(100);
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public Mesh generateRandomGarbageMesh(int numTriangles)
    {
        Mesh mesh = new Mesh();
        Vector3[] newVertices = new Vector3[numTriangles*3];
        Vector2[] newUV = new Vector2[numTriangles*3];
        int[] newTriangles = new int[numTriangles*3];

        for(int i = 0; i < numTriangles*3; i++)
        {
            newTriangles[i] = i;
        }
        
        for(int i = 0; i < numTriangles; i++)
        {
            Vector3 newPos = Random.insideUnitSphere*10.0f;
            newVertices[i * 3 + 0] = newPos + Random.insideUnitSphere;
            newVertices[i * 3 + 1] = newPos + Random.insideUnitSphere;
            newVertices[i * 3 + 2] = newPos + Random.insideUnitSphere;
            Vector3 newTx = Vector2.zero;
            newTx.x = newVertices[i * 3 + 0].x;
            newTx.y = newVertices[i * 3 + 0].y;
            newUV[i * 3 + 0] = newTx;
            newTx.x = newVertices[i * 3 + 1].x;
            newTx.y = newVertices[i * 3 + 1].y;
            newUV[i * 3 + 1] = newTx;
            newTx.x = newVertices[i * 3 + 2].x;
            newTx.y = newVertices[i * 3 + 2].y;
            newUV[i * 3 + 2] = newTx;
        }

        mesh.SetVertices(newVertices);
        mesh.SetTriangles(newTriangles, 0);
        mesh.SetUVs(0,newUV);
        mesh.RecalculateBounds();
        mesh.RecalculateNormals();
        mesh.RecalculateTangents();
        return mesh;

    }
}
