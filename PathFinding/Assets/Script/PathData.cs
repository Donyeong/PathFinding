using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace DPathFinder
{
    public struct NavLinkInfo
	{
        public int link_idx;
        public int size;
	}
    public class NavPolygon
    {
        public int[] vertex_idx;
        public List<int> link_polygons_idx;
        //public List<NavLinkInfo> link_polygons;
    }
    public struct Edge
    {
        public int Vertex1;
        public int Vertex2;

        public Edge(int v1, int v2)
        {
            Vertex1 = Mathf.Min(v1, v2);
            Vertex2 = Mathf.Max(v1, v2);
        }

        public override bool Equals(object obj)
        {
            if (obj is Edge)
            {
                Edge other = (Edge)obj;
                return Vertex1 == other.Vertex1 && Vertex2 == other.Vertex2;
            }
            return false;
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 23 + Vertex1.GetHashCode();
                hash = hash * 23 + Vertex2.GetHashCode();
                return hash;
            }
        }
    }

    public class NavMesh
    {
        public Vector3[] vertices;
        public List<NavPolygon> nav_polys;



        private void AddEdgeToMap(Dictionary<Edge, List<int>> edgeToTriangleMap, Edge edge, int poly_idx)
        {
            if (!edgeToTriangleMap.ContainsKey(edge))
            {
                edgeToTriangleMap[edge] = new List<int>();
            }
            edgeToTriangleMap[edge].Add(poly_idx);
        }
        public void build(MeshFilter _mf)
        {
            //Mesh Build
            Mesh target_mesh = _mf.sharedMesh;
            vertices = target_mesh.vertices;
            nav_polys = new List<NavPolygon>(target_mesh.triangles.Length / 3);
            Dictionary<Edge, List<int>> edgeToTriangleMap = new Dictionary<Edge, List<int>>();

            for (int i = 0; i < target_mesh.triangles.Length; i += 3)
            {
                NavPolygon nav_poly = new NavPolygon();
                nav_poly.vertex_idx = new int[3];

                int t0 = target_mesh.triangles[i + 0];
                int t1 = target_mesh.triangles[i + 1];
                int t2 = target_mesh.triangles[i + 2];
                nav_poly.vertex_idx[0] = t0;
                nav_poly.vertex_idx[1] = t1;
                nav_poly.vertex_idx[2] = t2;

                nav_poly.link_polygons_idx = new List<int>();

                AddEdgeToMap(edgeToTriangleMap, new Edge(t0, t1), i / 3);
                AddEdgeToMap(edgeToTriangleMap, new Edge(t1, t2), i / 3);
                AddEdgeToMap(edgeToTriangleMap, new Edge(t2, t0), i / 3);
                nav_polys.Add(nav_poly);

            }
            for (int i = 0; i < vertices.Length; i++)
            {
                vertices[i] = _mf.transform.TransformPoint(vertices[i]);
            }

            foreach (var kvp in edgeToTriangleMap)
            {
                List<int> list_polys = kvp.Value;
                foreach (var poly in list_polys)
                {
                    foreach (var link_poly in list_polys)
                    {
                        if (poly == link_poly)
                            continue;
                        nav_polys[poly].link_polygons_idx.Add(link_poly);
                    }
                }
            }
        }

        public int GetIndexByPos(Vector3 _pos)
        {
            return 0;
        }

        public int GetIndexByRaycast(Vector3 _begin, Vector3 _direction, float _range)
        {
            return 0;
        }



        bool RayIntersectsTriangle(Ray ray, Vector3 v0, Vector3 v1, Vector3 v2, out Vector3 hitPoint)
        {
            hitPoint = Vector3.zero;

            Vector3 edge1 = v1 - v0;
            Vector3 edge2 = v2 - v0;

            Vector3 h = Vector3.Cross(ray.direction, edge2);
            float a = Vector3.Dot(edge1, h);

            if (a > -0.00001f && a < 0.00001f)
                return false;

            float f = 1.0f / a;
            Vector3 s = ray.origin - v0;
            float u = f * Vector3.Dot(s, h);

            if (u < 0.0f || u > 1.0f)
                return false;

            Vector3 q = Vector3.Cross(s, edge1);
            float v = f * Vector3.Dot(ray.direction, q);

            if (v < 0.0f || u + v > 1.0f)
                return false;

            float t = f * Vector3.Dot(edge2, q);

            if (t > 0.00001f)
            {
                hitPoint = ray.origin + ray.direction * t;
                return true;
            }
            else
                return false;
        }
    }
}
