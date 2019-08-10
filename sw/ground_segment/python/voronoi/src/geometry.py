#!/usr/bin/env python3
"""
Module containing geometric functions for 2-D shapes
"""
# Standard libraries
import math
# Third-party libraries
import numpy as np
import scipy.integrate
import scipy.spatial


def get_polygon_centroid(vertices):
    """
    Get the X,Y coordinates of the centroid, given the vertices

    This function applies only to convex polygons. The vertices must be
    ordered  according their ocurrence along the perimeter of the
    polygon.
    """
    n_verts = vertices.shape[0]
    signed_area = 0
    centroid = np.array([.0, .0])

    # Iterate through the n vertices and compute the centroid coordinates.
    for i in range(n_verts):
        x0 = vertices[i, 0]
        y0 = vertices[i, 1]
        if i==(n_verts-1):
            x1 = vertices[0, 0]
            y1 = vertices[0, 1]
        else:
            x1 = vertices[i+1, 0]
            y1 = vertices[i+1, 1]
        signed_area += x0*y1 - x1*y0
        centroid[0] += (x0 + x1) * (x0*y1 - x1*y0)
        centroid[1] += (y0 + y1) * (x0*y1 - x1*y0)

    signed_area *= 0.5
    centroid /= (6.0*signed_area)
    return centroid


def sort_polygon_vertices(vertices):
    """
    Sort the vertices in order of appearance alongside the perimeter
    """
    # compute centroid and sort vertices by polar angle.
    inner_point = vertices.sum(axis=0) / len(vertices)
    diff = vertices-inner_point
    order = np.argsort(np.arctan2(diff[:,1], diff[:, 0]))
    ordered_vertices = vertices[order]
    return ordered_vertices


def get_centre_of_mass(vertices, function=lambda x,y: 1):
    """
    Get the mass and centre-of-mass of shape given the density function

    :param np.array vertices: Nx2 array with the coordinates of all the
     vertices defining the shape.
    :param callable function: 2 arguments function representing the
     density function of the shape.
    :returns: float representing the mass of the shape.
    """
    # Divide the cell in triangles, so the density function can be 
    # integrated through the whole shape.
    triangulation = scipy.spatial.Delaunay(vertices)
    total_mass = 0
    centroid = np.array([0.0, 0.0])
    # Calculate the mass and centroid of each triangle and add them up.
    for index, indices in enumerate(triangulation.simplices):
        tri = vertices[indices]
        a = tri[0]
        b = tri[1]
        c = tri[2]
        jacobian = (b[0]-a[0]) * (c[1]-a[1]) - (c[0]-a[0]) * (b[1]-a[1])
        jacobian = np.abs(jacobian)
        # Lambda pullback function of the density
        f_pullback = lambda u,v: function(a[0] + u*(b[0]-a[0]) + v*(c[0]-a[0]),
                                          a[1] + u*(b[1]-a[1]) + v*(c[1]-a[1]))
        # Lambda pullback function of the density multiplied by the position.
        # It represents the weighted coordinates of a mass.
        f_pullback_2_u = lambda u,v: (f_pullback(u, v) * u)
        f_pullback_2_v = lambda u,v: (f_pullback(u, v) * v)
        # Integrate mass throughout the whole shape.
        transformed_mass, _ = scipy.integrate.dblquad(f_pullback, 0, 1, 0,
                                                      lambda u: 1-u)
        c_u, _ = scipy.integrate.dblquad(f_pullback_2_u, 0, 1, 0, lambda u: 1-u)
        c_v, _ = scipy.integrate.dblquad(f_pullback_2_v, 0, 1, 0, lambda u: 1-u)
        c_u /= transformed_mass
        c_v /= transformed_mass
        c_x = a[0] + c_u*(b[0]-a[0]) + c_v*(c[0]-a[0])
        c_y = a[1] + c_u*(b[1]-a[1]) + c_v*(c[1]-a[1])
        mass = transformed_mass * jacobian
        total_mass += mass
        centre = np.array([c_x, c_y])
        centroid += centre * mass
    centroid /= total_mass
    return (total_mass, centroid, triangulation)
