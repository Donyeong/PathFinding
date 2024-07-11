using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace DPathFinder
{
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
            /*
            if (Input.GetMouseButtonDown(0))
            {
                Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
                for (int i = 0; i < nav_mesh.nav_polys.Count; i++)
                {
                    NavPolygon nav_poly = nav_mesh.nav_polys[i];
                    Vector3 v0 = nav_mesh.vertices[nav_poly.vertex_idx[0]];
                    Vector3 v1 = nav_mesh.vertices[nav_poly.vertex_idx[1]];
                    Vector3 v2 = nav_mesh.vertices[nav_poly.vertex_idx[2]];

                    if (RayIntersectsTriangle(ray, v0, v1, v2, out Vector3 hitPoint))
                    {
                        Vector3 worldHitPoint = transform.TransformPoint(hitPoint);
                        Debug.Log("Hit Polygon at: " + hitPoint);
                        Debug.Log("Hit Triangle Index: " + i);
                        List<NavAstarNode> path = NavLogicAstar.FindPath(current, i, nav_mesh);
                        lr.positionCount = path.Count;
                        for (int p = 0; p < path.Count; p++)
                        {
                            lr.SetPosition(p, path[p].p);
                        }
                        current = i;
                    }
                }
            
            }
            */
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
