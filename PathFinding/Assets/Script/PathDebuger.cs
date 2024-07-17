using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace DPathFinder
{
    public class PathDebuger : MonoBehaviour
    {
        public bool is_builded = false;
        public MeshFilter mesh;
        public NavMesh nav_mesh;
        public PathFinder path_finder;
        public int current = 0;


		public List<Vector3> path_o = new List<Vector3>();
		public List<Vector3> path = new List<Vector3>();
        //public NavMeshBuilder nav_mesh_builder;
        // Start is called before the first frame update
        void Start()
        {
            nav_mesh = NavMeshBuilder.Build(mesh.sharedMesh, mesh.transform);
            is_builded = true;
        }

        // Update is called once per frame
        void Update()
        {
            if (is_builded)
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

                        if(nav_mesh.RayIntersectsTriangle(ray, v0, v1, v2, out Vector3 hit_point))
                        {
                            OnRaycastHit(i, hit_point);
                        }
                    }
                }
            }
        }

        public void OnRaycastHit(int _hit_index, Vector3 _hit_point)
        {
            Vector3 worldHitPoint = transform.TransformPoint(_hit_point);
            Debug.Log("Hit Polygon at: " + _hit_point);
            Debug.Log("Hit Triangle Index: " + _hit_index);

            int prev_curr = current;
            current = _hit_index;
            path.Clear();
            path_o.Clear();

			PathFinder pf = new PathFinder();
            pf.SetNavMesh(nav_mesh);

            List<NavAstarNode> astar_path = NavLogicAstar.FindPath(prev_curr, current, nav_mesh);
            if(astar_path.Count == 0)
            {
                return;
            }

            for(int i = 0; i < astar_path.Count; i++)
            {
                path_o.Add(astar_path[i].p);
			}

            path.Add(astar_path[0].p);
            int prev_point = astar_path[0].poly_idx;
            Vector3 prev_point_pos = astar_path[0].p;

            Debug.Log($"#########Path Find Start==========");
            for (int i = 1; i < astar_path.Count; i++)
            {
                Debug.Log($"Path Logic {i}==========");
                if (i == astar_path.Count - 1)
                {
                    path.Add(astar_path[i].p);
                    continue;
                }
                bool res = pf.NavRayCheck(prev_point_pos, astar_path[i+1].p, prev_point, astar_path[i+1].poly_idx);
                Debug.Log($"Path Ray Res {res}");
                if (!res)
                {
                    path.Add(astar_path[i].p);
                    prev_point = astar_path[i ].poly_idx;
					prev_point_pos = astar_path[i].p;
				}
            }
        }
        private void OnDrawGizmos()
        {
            if (null != mesh)
            {
                GameObject go = mesh.gameObject;
                Gizmos.color = Color.white;
                Gizmos.DrawWireMesh(mesh.sharedMesh, 0, go.transform.position, go.transform.rotation, go.transform.lossyScale);
				Gizmos.color = Color.cyan;
				for (int i = 0; i < path_o.Count - 1; i++)
				{
					Gizmos.DrawLine(path_o[i], path_o[i + 1]);
				}
				Gizmos.color = Color.red;
                for(int i = 0; i < path.Count-1; i++)
				{
                    Gizmos.DrawLine(path[i], path[i+1]);
                }
            }
        }
    }
}
