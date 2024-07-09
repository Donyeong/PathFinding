using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PathDebuger : MonoBehaviour
{
    public MeshFilter mesh;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnDrawGizmos()
    {
        if (null != mesh)
        {
            GameObject go = mesh.gameObject;
            Gizmos.DrawWireMesh(mesh.sharedMesh, 0, go.transform.position + Vector3.up, go.transform.rotation, go.transform.lossyScale);
        }
    }
}
