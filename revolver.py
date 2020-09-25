#!/usr/bin/env python
import sys
import os
import argparse
import toml
import numpy as np
from OCC.Core.BRepBuilderAPI import *
from OCC.Core.BRepMesh import *
from OCC.Core.BRepPrimAPI import *
from OCC.Core.BRepTools import *
from OCC.Core.GeomAPI import *
from OCC.Core.gp import *
from OCC.Core.STEPControl import *
from OCC.Core.StlAPI import *
from OCC.Core.TColgp import *

# Use mm internally for length scale.  OpenCascade internally also uses mm=1.0,
# so don't bother converting lengths when sending/receiving from OpenCascade.
mm = 1.0
deg = np.pi/180

def get_parser():
    parser = argparse.ArgumentParser(
        description='Generate surface-of-revolution')
    parser.add_argument(
        'config_filename', metavar='conf-file',
        help='Configuration file')
    return parser

def get_conf():
    parser = get_parser()
    args = parser.parse_args()
    return toml.load(args.config_filename)

def point_list(li):
    pts = TColgp_Array1OfPnt(1, len(li))
    for n,i in enumerate(li):
        pts.SetValue(n+1, i)
    return pts
                        
def revolve(z_vals, r_vals, use_bspline):
    edge_points = [gp_Pnt(r, 0, z) for (r, z) in zip(r_vals, z_vals)]
    if use_bspline:
        bspline2 = GeomAPI_PointsToBSpline(point_list(edge_points)).Curve()
        path_edge = BRepBuilderAPI_MakeEdge(bspline2).Edge()
        path_wire = BRepBuilderAPI_MakeWire(path_edge).Wire()
    else:
        wire = BRepBuilderAPI_MakeWire()
        for i in range(len(edge_points)-1):
            edge = BRepBuilderAPI_MakeEdge(
                edge_points[i], edge_points[i+1]).Edge()
            wire.Add(edge)
        path_wire = wire.Wire()
    face = BRepBuilderAPI_MakeFace(path_wire).Face()
    revolve_axis = gp_OZ()
    revolved_shape = BRepPrimAPI_MakeRevol(
        face, revolve_axis, np.deg2rad(360.)).Shape()
    return revolved_shape

def main():
    conf = get_conf()
    scale = conf['Model']['Scale']
    unit = mm*scale
    z0 = conf['Model']['z0']*unit
    use_bspline = conf['Model']['UseBSpline']
    z, r = (np.array(conf['Model']['r_z']) * unit).T
    # epsilon = 1e-6*unit
    # r[r==0] = epsilon
    z = z0 - z
    surface = revolve(-z, r, use_bspline)

    for outconf in conf['Output']:
        filename = outconf['Filename']
        path = os.path.dirname(filename)
        if path != '':
            os.makedirs(path, exist_ok=True)
        if outconf['Type'] == 'STL':
            # clear out any existing mesh
            breptools_Clean(surface)
            mesh = BRepMesh_IncrementalMesh(
                surface, outconf['LinearDeflection']*mm,
                outconf['IsRelative'], outconf['AngularDeflection']*deg, True)
            stl_writer = StlAPI_Writer()
            stl_writer.SetASCIIMode(False)
            stl_writer.Write(surface, filename)
        elif outconf['Type'] == 'STEP':
            step_writer = STEPControl_Writer()
            step_writer.Transfer(surface, STEPControl_AsIs)
            status = step_writer.Write(filename)

if __name__ == '__main__':
    sys.exit(main())
