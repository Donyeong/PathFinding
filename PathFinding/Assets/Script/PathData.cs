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
    }
}
