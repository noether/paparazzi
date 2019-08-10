#!/usr/bin/env python3
"""
Library for generating the Voronoi polytope around 1 single site.

The algorithm needs the map boundaries, and the coordinates of all the
agents in the neighbourhood, including the site whose Voronoi polytope
is wanted to be found.

The algorithm finds the midpoints between the agent and the rest of the
sites, as well as the unit vector perpendicular to the segment joining
both sites.

Afterwards, it finds the intersections between every Voronoi segment,
and filters those outside of the map or closer to other sites.
"""
# Standard libraries
import sys
# Third-party libraries
from matplotlib.collections import PatchCollection
from matplotlib.patches import Circle, Polygon
import matplotlib.pyplot as plt
import numpy as np
import yaml


def plot_voronoi(ax, map_vertices, agents_coords, voronoi_vertices, triangles,
                 user_control=False):
    """
    Draw a Voronoi diagram on an already existing figure and axis.

    This function is meant to be used as a live animation of the
    Voronoi diagram created by moving agents.

    :param AxesSubplot ax: axis contained in a matplotlib figure, where
     the passed information will be drawed into.
    :param np.array map_vertices: 2xN array with the coordinates of all
     of the N vetices of the map.
    :param np.array agents_coords: 2xM array with the coordinates of the
     M agents(sites) in the system.
    :param np.array voronoi_vertices: MxV array containing the vertices
     of the Voronoi cells surrounding the M agents. The number V of
     vertices of each cell is variable for each agent.
    """
    ax.cla()
    ax.add_patch(Polygon(map_vertices, True, fill=False, edgecolor="k",
                         linewidth=2))
    for index, polytop_verts in enumerate(voronoi_vertices):
        # Draw Voronoi polytope.
        ax.add_patch(Polygon(polytop_verts, True, fill=False, edgecolor="g",
                             linewidth=2))
        for triangle  in triangles[index]:
            ax.add_patch(Polygon(polytop_verts[triangle], True, fill=False,
                                 edgecolor="y", linewidth=0.2, zorder=10))
    for coords in agents_coords:
        ax.add_patch(Circle(coords, 0.2, color='#FF9900'))
    for agent in voronoi_vertices:
        for vertices in agent:
            ax.add_patch(Circle(vertices, 0.2, color='#0022FF'))
    min_coords = map_vertices.min(axis=0)
    max_coords = map_vertices.max(axis=0)
    ax.set_xlim(min_coords[0]-5, max_coords[0]+5)
    ax.set_ylim(min_coords[1]-5, max_coords[1]+5)
    plt.draw()
    if user_control:
        plt.waitforbuttonpress()
    else:
        plt.pause(0.1)
    return


def get_voronoi_segments(map_vertices, sites, index):
    """
    Get Voronoi segments defined by their parametric equations.

    :param np.array map_vertices: 2xN array with the coordinates of all
     of N the vetices of the map.
    :param np.array sites: 2xM array with the coordinates of the M
     agents(sites) in the system.
    :param int index: index of the agent(site) whose Voronoi cell is
    wanted to be found.
    """
    map_sides = len(map_vertices)
    n_sites = len(sites)
    vertices = np.array(())
    # Initialize the segments list. First element of a segment is a point S_0,
    # and the second element is its unit vector.
    segments = np.zeros((map_sides+(n_sites-1), 2, 2))
    # Add border segments to the segments variable
    diffs = map_vertices - np.roll(map_vertices, -1, axis=0)
    distances = np.linalg.norm(diffs, axis=1).reshape(map_sides, 1)
    unit_vectors = diffs / distances
    segments[:map_sides, 0] = map_vertices
    segments[:map_sides, 1] = unit_vectors

    # Get current agent coords and remove it from the input list
    agent_coords = sites[index]
    sites = np.copy(sites)
    sites = np.delete(sites, index, 0)
    # Add paramters of segments separating agent from other sites.
    mid_points = agent_coords + (sites-agent_coords)/2
    diffs = sites - agent_coords
    distances = np.linalg.norm(diffs, axis=1).reshape(n_sites-1, 1)
    unit_vectors = diffs / distances
    normal_vectors = np.zeros_like(unit_vectors)
    normal_vectors[:, 0] = unit_vectors[:, 1]
    normal_vectors[:, 1] = -unit_vectors[:, 0]
    segments[map_sides:, 0] = mid_points
    segments[map_sides:, 1] = normal_vectors
    return segments


def get_intersections(segments):
    """
    Return all the intersections coordinates between the input segments.

    More info at http://geomalgorithms.com/a05-_intersect-1.html

    :param np.array segments: Sx2x2 array containing the parameters of
     the parametric representation of the S segments in the system. The
     first parameter is a 2x1 array with the coordinates of a point P_0
     in the segment, and the second parameter is a 2x1 array with the
     values of the unit vector of the segment.
    """
    intersections = np.array([])
    for index, segment in enumerate(segments):
        v = segment[1]
        v_perp = np.array([[-v[1], v[0]]])
        for index_2, segment_2 in enumerate(segments[index+1:]):
            u = segment_2[1]
            # Ignore calculations if segments are parallel.
            if u[0]*v[1] == u[1]*v[0]:
                continue
            # Get intersection between both segments.
            w = segment_2[0]-segment[0]
            num = (-np.dot(v_perp, w))
            den = (np.dot(v_perp, u))
            s_I = num / den
            intersection = segment_2[0] + u*s_I
            if not len(intersections):
                intersections = intersection
            else:
                intersections = np.vstack((intersections, intersection))
    return intersections


def filter_vertices(map_vertices, intersections, sites, index=0, shape="rect",
                    tol=0.01):
    """
    Remove intersections that do not belong to Voronoi map.

    :param np.array map_vertices: 2xN array with the coordinates of all
     of N the vetices of the map.
    :param np.array intersections: 2xI array with the coordinates of the
     I Voronoi unfiltered intersections in the system
    :param np.array sites: 2xM array with the coordinates of the M
     agents(sites) in the system.
    :param int index: index of the agent(site) whose Voronoi cell is
     wanted to be found.
    :param str shape: Type of the shape of the map
    """
    filt_intersecs = np.array([])
    agent_coords = sites[index]
    sites = np.copy(sites)
    sites = np.delete(sites, index, 0)
    # Get rectangle min and max coordinates.
    if shape=="rect":
        min_coords = map_vertices.min()
        max_coords = map_vertices.max()
    for i, intersection in enumerate(intersections):
        # Check if the intersection is out of the rectangle
        if (np.any(intersection<min_coords-tol)
                or np.any(intersection>max_coords+tol)):
            continue
        # Check if the intersection is closer to any other site.
        # Subtract a tolerance for avoiding imprecisions.
        sites_dists = np.linalg.norm(intersection-sites, axis=1)
        agent_dist = np.linalg.norm(intersection-agent_coords)
        if np.all(sites_dists>=(agent_dist-tol)):
            if not len(filt_intersecs):
                filt_intersecs = intersection
            else:
                filt_intersecs = np.vstack((filt_intersecs, intersection))
    return filt_intersecs


def get_voronoi_cell(map_vertices, sites, index=0):
    """
    Call functions for building a Voronoi diagram around one single site

    :param np.array map_vertices: 2xN array with the coordinates of all
     of the vetices of the map.
    :param np.array sites: 2xM array with the coordinates of all of the
     agents(sites) in the system.
    :param int index: index of the agent(site) whose Voronoi cell is
    wanted to be found.
    """
    voronoi_segments = get_voronoi_segments(map_vertices, sites, index)
    voronoi_verts = get_intersections(voronoi_segments)
    # Remove intersections out of the map, and those closer to other sites.
    filtered_verts = filter_vertices(map_vertices, voronoi_verts, sites, index)
    return filtered_verts


if __name__ == "__main__":
    # Read the configuration file and parse map and sites coordinates.
    with open("./config/simulation.yaml", "r") as f: 
        config = yaml.load(f)
    # Create an array with map coordinates.
    map_dict = config["Map"]
    map_vertices = np.zeros([len(map_dict), 2])
    for key, value in enumerate(map_dict):
        map_vertices[key] = np.array(map_dict[value])
    # Create an array with the sites coordinates.
    sites_dict = config["Sites"]
    sites = np.zeros([len(sites_dict), 2])
    for key, value in enumerate(sites_dict):
        sites[key] = np.array(sites_dict[value])

    # Generate the Voronoi vertices.
    voronoi_vertices = get_voronoi_cell(map_vertices, sites, 0)
    # Draw vertices in a plot.
    plot_voronoi(map_vertices, sites, voronoi_vertices[0])
