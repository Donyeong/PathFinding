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

        public void NavRayCheck(int _start, int _end)
        {
        }
    }
}
