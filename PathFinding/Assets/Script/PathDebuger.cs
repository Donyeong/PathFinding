using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace DPathFinder
{
    public class PathDebuger : MonoBehaviour
    {
        public MeshFilter mesh;
        public NavMesh nav_mesh;
        //public NavMeshBuilder nav_mesh_builder;
        // Start is called before the first frame update
        void Start()
        {
            nav_mesh = NavMeshBuilder.Build();
        }

        // Update is called once per frame
        void Update()
        {
            if (Input.GetMouseButtonDown(0))
            {
                Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
                for (int i = 0; i < nav_mesh.nav_polys.Count; i++)
                {
                    NavPolygon nav_poly = nav_mesh.nav_polys[i];
                    Vector3 v0 = nav_mesh.vertices[nav_poly.vertex_idx[0]];
                    Vector3 v1 = nav_mesh.vertices[nav_poly.vertex_idx[1]];
                    Vector3 v2 = nav_mesh.vertices[nav_poly.vertex_idx[2]];

                    bool is_hit = nav_mesh.RayIntersectsTriangle(ray, v0, v1, v2, out Vector3 hitPoint);
                    if(is_hit)
                    {
                        Vector3 worldHitPoint = transform.TransformPoint(hitPoint);
                        Debug.Log("Hit Polygon at: " + hitPoint);
                        Debug.Log("Hit Triangle Index: " + i);
                    }
                }
            
            }
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
}
