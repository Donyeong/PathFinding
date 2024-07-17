using System.Collections;
using System.Collections.Generic;
using System.Security.Cryptography;
using UnityEditor.Experimental.GraphView;
using UnityEngine;

namespace DPathFinder
{
	public class NavAstarNode
	{
		public int poly_idx;
		public Vector3 p;
		public NavAstarNode Parent { get; set; }
		public float G { get; set; } // Cost from start to this node
		public float H { get; set; } // Heuristic cost from this node to goal
		public float F => G + H; // Total cost

		public NavAstarNode(int _poly_idx)
		{
			this.poly_idx = _poly_idx;
		}

		public override bool Equals(object _obj)
		{
			if (_obj is NavAstarNode node)
			{
				return poly_idx == node.poly_idx;
			}
			return false;
		}

		public override int GetHashCode()
		{
			return poly_idx;
		}
	}



	public class NavLogicAstar
	{
		public static List<NavAstarNode> FindPath(int _start_idx, int _goal_idx, NavMesh _nav_mesh)
		{
			NavAstarNode[] nodes = new NavAstarNode[_nav_mesh.nav_polys.Count];
			for (int i = 0; i < _nav_mesh.nav_polys.Count; i++)
			{
				int vertex_0 = _nav_mesh.nav_polys[i].vertex_idx[0];
				int vertex_1 = _nav_mesh.nav_polys[i].vertex_idx[1];
				int vertex_2 = _nav_mesh.nav_polys[i].vertex_idx[2];
				Vector3 pos_0 = _nav_mesh.vertices[vertex_0];
				Vector3 pos_1 = _nav_mesh.vertices[vertex_1];
				Vector3 pos_2 = _nav_mesh.vertices[vertex_2];
				nodes[i] = new NavAstarNode(i);
				nodes[i].p = (pos_0 + pos_1 + pos_2) / 3;
				nodes[i].H = 0;
				nodes[i].G = 0;
			}
			NavAstarNode start = nodes[_start_idx];
			NavAstarNode goal = nodes[_goal_idx];
			var openSet = new SortedSet<NavAstarNode>(Comparer<NavAstarNode>.Create((a, b) =>
			{
				int compare = a.F.CompareTo(b.F);
				if (compare == 0)
				{
					compare = a.H.CompareTo(b.H);
				}
				return compare;
			}));

			var closedSet = new HashSet<NavAstarNode>();
			start.G = 0;
			start.H = Heuristic(start, goal);
			openSet.Add(start);
			int debug_c = 0;
			while (openSet.Count > 0)
			{
				debug_c++;
				if (debug_c > 999)
				{
					Debug.LogError("path error");
					return new List<NavAstarNode>(); // No path found
				}
				int openSetC = openSet.Count;
				var current = openSet.Min;
				openSet.Remove(current);


				if (current.Equals(goal))
				{
					return ReconstructPath(current);
				}

				closedSet.Add(current);

				foreach (int i in _nav_mesh.nav_polys[current.poly_idx].link_polygons_idx)
				{
					var neighbor = nodes[i];
					if (closedSet.Contains(neighbor))
					{
						continue;
					}

					float tentativeG = current.G + 1; // Assuming cost between nodes is 1

					if (!openSet.Contains(neighbor))
					{
						neighbor.Parent = current;
						neighbor.G = tentativeG;
						neighbor.H = Heuristic(neighbor, goal);
						openSet.Add(neighbor);
					}
					else if (tentativeG < neighbor.G)
					{
						openSet.Remove(neighbor);
						neighbor.Parent = current;
						neighbor.G = tentativeG;
						// Re-sort the openSet because the cost has changed
						openSet.Add(neighbor);
					}
				}
			}
			return new List<NavAstarNode>(); // No path found

		}



		static float Heuristic(NavAstarNode a, NavAstarNode b)
		{
			return (a.p - b.p).magnitude;
		}

		static List<NavAstarNode> ReconstructPath(NavAstarNode current)
		{
			var path = new List<NavAstarNode>();
			while (current != null)
			{
				path.Add(current);
				current = current.Parent;
			}
			path.Reverse();
			return path;
		}
	}

	public class PathFinder
    {
        private NavMesh m_nav_mesh;

        public void SetNavMesh(NavMesh _nav_mesh)
        {
            m_nav_mesh = _nav_mesh;
        }

		/* 직선상에 경로가 존재하는지 검사 */
		public bool NavRayCheck(Vector3 _start_point, Vector3 _end_point, int _start, int _end)
        {
            List<NavPolygon> nav_polys = m_nav_mesh.nav_polys;
            NavPolygon start_poly = nav_polys[_start];
            NavPolygon end_poly = nav_polys[_end];
            Queue<NavPolygon> queue = new Queue<NavPolygon>();
            HashSet<int> close_nodes = new HashSet<int>();

            queue.Enqueue(start_poly);
            close_nodes.Add(_start);

            while (0 < queue.Count)
            {
                NavPolygon curr_node = queue.Dequeue();
                //도착지점까지 노드 연결됨.
                if (curr_node == end_poly)
                {
                    return true;
                }
                //인접한 폴리곤이 직선상에 포함되는지 검사
                List<int> link_polys = curr_node.link_polygons_idx;
                foreach(int link_poly_idx in link_polys)
                {
                    if(close_nodes.Contains(link_poly_idx))
                    {
                        continue;
                    }
                    NavPolygon link_poly = nav_polys[link_poly_idx];

                    int vtx_idx_0 = link_poly.vertex_idx[0];
                    int vtx_idx_1 = link_poly.vertex_idx[1];
                    int vtx_idx_2 = link_poly.vertex_idx[2];

                    Vector3 v0 = m_nav_mesh.vertices[vtx_idx_0];
                    Vector3 v1 = m_nav_mesh.vertices[vtx_idx_1];
                    Vector3 v2 = m_nav_mesh.vertices[vtx_idx_2];

                    bool is_hit = IsLineIntersectingTriangle(_start_point, _end_point, v0, v1, v2);
                    if (is_hit)
                    {
                        queue.Enqueue(link_poly);
                        close_nodes.Add(link_poly_idx);
                    }
                    //직선상에 포함 되는 노드인 경우 큐에 삽입
                }
            }
            return false;
        }

		private bool DoLineSegmentsIntersect(Vector3 p1, Vector3 p2, Vector3 q1, Vector3 q2)
		{
			////p1.y = 0;
			//p2.y = 0;
			//q1.y = 0;
			//q2.y = 0;
			// 두 벡터의 방향 계산
			Vector3 r = p2 - p1;
			Vector3 s = q2 - q1;

			// 크로스 프로덕트를 사용하여 평행 여부 확인
			float rCrossS = Vector3.Cross(r, s).z;
			float qMinusPCrossR = Vector3.Cross(q1 - p1, r).z;

			// 평행하지 않은 경우
			if (rCrossS != 0)
			{
				float t = Vector3.Cross(q1 - p1, s).z / rCrossS;
				float u = Vector3.Cross(q1 - p1, r).z / rCrossS;

				// t와 u 값이 모두 0과 1 사이에 있으면 교차함
				if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
				{
					return true;
				}
			}
			// 평행하고 일치하는 경우
			else if (qMinusPCrossR == 0)
			{
				// 선분이 겹치는지 확인
				float t0 = Vector3.Dot(q1 - p1, r) / Vector3.Dot(r, r);
				float t1 = Vector3.Dot(q2 - p1, r) / Vector3.Dot(r, r);

				if ((t0 >= 0 && t0 <= 1) || (t1 >= 0 && t1 <= 1) || (t0 < 0 && t1 > 1) || (t0 > 1 && t1 < 0))
				{
					return true;
				}
			}

			return false;
		}
		private bool IsPointInTriangle(Vector3 p, Vector3 v0, Vector3 v1, Vector3 v2)
        {
            float sign(Vector3 p1, Vector3 p2, Vector3 p3)
            {
                return (p1.x - p3.x) * (p2.z - p3.z) - (p2.x - p3.x) * (p1.z - p3.z);
            }

            bool b1, b2, b3;

            b1 = sign(p, v0, v1) < 0.0f;
            b2 = sign(p, v1, v2) < 0.0f;
            b3 = sign(p, v2, v0) < 0.0f;

            return ((b1 == b2) && (b2 == b3));
        }
        public bool IsLineIntersectingTriangle(Vector3 p1, Vector3 p2, Vector3 v0, Vector3 v1, Vector3 v2)
        {
            if (DoLineSegmentsIntersect(p1, p2, v0, v1)) return true;
            if (DoLineSegmentsIntersect(p1, p2, v1, v2)) return true;
            if (DoLineSegmentsIntersect(p1, p2, v2, v0)) return true;

            //if (IsPointInTriangle(p1, v0, v1, v2)) return true;
            //if (IsPointInTriangle(p2, v0, v1, v2)) return true;

            return false;
        }

    }
}
