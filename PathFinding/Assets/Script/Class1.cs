using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace DPathFinder
{
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
            float orientation(Vector3 a, Vector3 b, Vector3 c)
            {
                return (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x);
            }

            bool onSegment(Vector3 a, Vector3 b, Vector3 c)
            {
                return Mathf.Min(a.x, b.x) <= c.x && c.x <= Mathf.Max(a.x, b.x) &&
                       Mathf.Min(a.z, b.z) <= c.z && c.z <= Mathf.Max(a.z, b.z);
            }

            float o1 = orientation(p1, p2, q1);
            float o2 = orientation(p1, p2, q2);
            float o3 = orientation(q1, q2, p1);
            float o4 = orientation(q1, q2, p2);

            if (o1 != o2 && o3 != o4) return true;

            if (o1 == 0 && onSegment(p1, p2, q1)) return true;
            if (o2 == 0 && onSegment(p1, p2, q2)) return true;
            if (o3 == 0 && onSegment(q1, q2, p1)) return true;
            if (o4 == 0 && onSegment(q1, q2, p2)) return true;

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

            if (IsPointInTriangle(p1, v0, v1, v2)) return true;
            if (IsPointInTriangle(p2, v0, v1, v2)) return true;

            return false;
        }

    }
}
