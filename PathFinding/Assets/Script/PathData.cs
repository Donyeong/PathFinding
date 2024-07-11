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
        public List<NavLinkInfo> link_polygons;
    }

    public class NavMesh
    {
        public Vector3[] vertices;
        public List<NavPolygon> nav_polys;

        public int GetIndexByPos(Vector3 _pos)
        {
            return 0;
        }

        public int GetIndexByRaycast(Vector3 _begin, Vector3 _direction, float _range)
        {
            return 0;
        }
    }
}
