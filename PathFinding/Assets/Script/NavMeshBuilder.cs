using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace DPathFinder
{
	public class NavMeshBuilder
	{
		public static NavMesh Build(Mesh _shared_mesh, Transform _transform)
		{
			NavMesh nav_mesh = new NavMesh();
			nav_mesh.build(_shared_mesh, _transform);
            return nav_mesh;
		}
	}
}