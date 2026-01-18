#!/usr/bin/env python3
"""
Mesh utility functions for loading and processing 3D mesh files.
"""
import struct
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Point


def load_stl_mesh(filepath):
    """
    Load an STL file and return a Mesh message.
    
    Args:
        filepath (str): Path to the STL file
        
    Returns:
        shape_msgs.msg.Mesh: ROS Mesh message containing vertices and triangles
    """
    mesh = Mesh()
    
    with open(filepath, 'rb') as f:
        f.read(80)  # Skip header
        num_triangles = struct.unpack('<I', f.read(4))[0]
        
        vertices = []
        vertex_map = {}
        
        for _ in range(num_triangles):
            f.read(12)  # Skip normal
            
            triangle = MeshTriangle()
            indices = []
            
            for _ in range(3):
                vx, vy, vz = struct.unpack('<fff', f.read(12))
                vertex_key = (vx, vy, vz)
                
                if vertex_key not in vertex_map:
                    p = Point()
                    p.x = float(vx)
                    p.y = float(vy)
                    p.z = float(vz)
                    vertex_map[vertex_key] = len(vertices)
                    vertices.append(p)
                
                indices.append(vertex_map[vertex_key])
            
            triangle.vertex_indices = indices
            mesh.triangles.append(triangle)
            f.read(2)  # Skip attribute
    
    mesh.vertices = vertices
    return mesh
