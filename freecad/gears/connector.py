# -*- coding: utf-8 -*-
# ***************************************************************************
# *                                                                         *
# * This program is free software: you can redistribute it and/or modify    *
# * it under the terms of the GNU General Public License as published by    *
# * the Free Software Foundation, either version 3 of the License, or       *
# * (at your option) any later version.                                     *
# *                                                                         *
# * This program is distributed in the hope that it will be useful,         *
# * but WITHOUT ANY WARRANTY; without even the implied warranty of          *
# * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
# * GNU General Public License for more details.                            *
# *                                                                         *
# * You should have received a copy of the GNU General Public License       *
# * along with this program.  If not, see <http://www.gnu.org/licenses/>.   *
# *                                                                         *
# ***************************************************************************

import os
import copy
import numpy as np
import FreeCAD
from pygears import __version__
from .features import InvoluteGear, CycloidGear, InvoluteGearRack, CycloidGearRack, InternalInvoluteGear, BaseGear
from pygears.computation import compute_shifted_gears

class ViewProviderGearConnector(object):
    def __init__(self, vobj, icon_fn=None):
        # Set this object to the proxy object of the actual view provider
        vobj.Proxy = self
        dirname = os.path.dirname(__file__)
        self.icon_fn = icon_fn or os.path.join(dirname, "icons", "gearconnector.svg")
            
    def attach(self, vobj):
        self.vobj = vobj

    def getIcon(self):
        return self.icon_fn

    def __getstate__(self):
        return {"icon_fn": self.icon_fn}

    def __setstate__(self, state):
        self.icon_fn = state["icon_fn"]


class GearConnector(object):
    def __init__(self, obj, master_gear, slave_gear):
        obj.addProperty("App::PropertyString", "version", "version", "freecad.gears-version", 1)
        obj.addProperty("App::PropertyLink","master_gear","gear","master gear", 1)
        obj.addProperty("App::PropertyLink","slave_gear","gear","slave gear", 1)
        obj.addProperty("App::PropertyLink","master_gear_connector_parent","gear","master gear parent object for connector animation", 1)
        obj.addProperty("App::PropertyLink","slave_gear_connector_parent","gear","slave gear parent object for connector animation", 1)
        obj.addProperty("App::PropertyAngle", "angle1", "gear", "angle at which second gear is placed", 0)
        obj.addProperty("App::PropertyAngle", "angle2", "gear", "angle at which second gear is placed", 1)
        obj.version = __version__
        obj.master_gear = master_gear
        obj.slave_gear = slave_gear
        obj.master_gear_connector_parent = master_gear
        obj.slave_gear_connector_parent = slave_gear

        if not self.is_gear_subclass(obj.master_gear):
            # print("master gear is non gear instance, searching subtree for gear itself...")
            result = self.search_for_gear_recursive(obj.master_gear)
            if result:
                # print("selected first child canidate master gear %s" % result.Label)
                obj.master_gear = result
        if not self.is_gear_subclass(obj.slave_gear):
            # print("slave gear is non gear instance, searching subtree for gear itself...")
            result = self.search_for_gear_recursive(obj.slave_gear)
            if result:
                # print("selected first child canidate slave gear %s" % result.Label)
                obj.slave_gear = result


        obj.angle1 = 0
        obj.angle2 = 0
        obj.Proxy = self

        assert isinstance(obj.master_gear.Proxy, BaseGear)
        assert isinstance(obj.slave_gear.Proxy, BaseGear)
        # todo possibly only one of the above needs to be a gear, source angle/etc could be taken from anything else
        # and gear info default to the presently available gear, allowing solids to drive gear or vice versa.
        # With custom movement functions one or both elements could be non gears too, eg ellipsoids controlling solids, etc
        # thinking of elongated InternalInvoluteGear or other scenarios including arbatrary physics objects attached to the
        # gear exerting force on arbatrarily shaped secondary objects in systematic and mathematically predicatble ways,
        # which could in turn spur more auxillary connector movements... in any anything is possible chain of update way.
        # share your code!

    def is_gear_subclass(self, possible_gear):
        return 'Proxy' in possible_gear.PropertiesList and isinstance(possible_gear.Proxy, BaseGear)

    def search_for_gear_recursive(self, gear_parent):
        if not self.is_gear_subclass(gear_parent):
            for child in gear_parent.OutList:
                if self.is_gear_subclass(child):
                    return child
                else:
                    match = self.search_for_gear_recursive(child)
                    if self.is_gear_subclass(match):
                        return match
        else:
            return gear_parent

    def onChanged(self, fp, prop):
        # fp.angle2 = fp.master_gear.Placement.Rotation.Angle
        if isinstance(fp.master_gear.Proxy, InvoluteGear) and isinstance(fp.slave_gear.Proxy, InvoluteGear):
            angle_master = fp.master_gear_connector_parent.Placement.Rotation.Angle * sum(fp.master_gear_connector_parent.Placement.Rotation.Axis)
            dw_master = fp.master_gear.dw
            dw_slave = fp.slave_gear.dw
            dist = (dw_master + dw_slave) / 2
            if fp.master_gear.shift != 0 or fp.slave_gear.shift != 0:
                dist, alpha_w = compute_shifted_gears(
                    fp.master_gear.module,
                    np.deg2rad(fp.master_gear.pressure_angle.Value),
                    fp.master_gear.teeth,
                    fp.slave_gear.teeth,
                    fp.master_gear.shift,
                    fp.slave_gear.shift)

            mat0 = FreeCAD.Matrix()  # unity matrix
            trans = FreeCAD.Vector(dist)
            mat0.move(trans)
            rot = FreeCAD.Rotation(FreeCAD.Vector(0,0,1), fp.angle1).toMatrix()
            angle2 = dw_master / dw_slave * fp.angle1.Value
            angle4 = dw_master / dw_slave * np.rad2deg(angle_master)
            rot2 = FreeCAD.Rotation(FreeCAD.Vector(0,0,1), angle2).toMatrix()
            angle3 = abs(fp.slave_gear.teeth % 2 - 1) * 180. / fp.slave_gear.teeth
            rot3 = FreeCAD.Rotation(FreeCAD.Vector(0,0,1), angle3).toMatrix()
            rot4 = FreeCAD.Rotation(FreeCAD.Vector(0,0,1), -angle4).toMatrix()
            mat1 = rot * mat0 * rot2 * rot3 * rot4
            mat1.move(fp.master_gear_connector_parent.Placement.Base)
            fp.slave_gear_connector_parent.Placement = mat1

        if isinstance(fp.master_gear.Proxy, InternalInvoluteGear) and isinstance(fp.slave_gear.Proxy, InvoluteGear):
            angle_master = fp.master_gear_connector_parent.Placement.Rotation.Angle * sum(fp.master_gear_connector_parent.Placement.Rotation.Axis)
            dw_master = fp.master_gear.dw
            dw_slave = fp.slave_gear.dw
            dist = (dw_master - dw_slave) / 2
            if fp.master_gear.shift != 0 or fp.slave_gear.shift != 0:
                dist, alpha_w = compute_shifted_gears(
                    fp.master_gear.module,
                    np.deg2rad(fp.master_gear.pressure_angle.Value),
                    fp.master_gear.teeth,
                    fp.slave_gear.teeth,
                    fp.master_gear.shift,
                    fp.slave_gear.shift)

            mat0 = FreeCAD.Matrix()  # unity matrix
            trans = FreeCAD.Vector(dist)
            mat0.move(trans)
            rot = FreeCAD.Rotation(FreeCAD.Vector(0,0,1), fp.angle1).toMatrix()
            angle2 = -dw_master / dw_slave * fp.angle1.Value
            angle4 = -dw_master / dw_slave * np.rad2deg(angle_master)
            rot2 = FreeCAD.Rotation(FreeCAD.Vector(0,0,1), angle2).toMatrix()
            angle3 = abs(fp.slave_gear.teeth % 2 - 1) * 180. / fp.slave_gear.teeth
            rot3 = FreeCAD.Rotation(FreeCAD.Vector(0,0,1), angle3).toMatrix()
            rot4 = FreeCAD.Rotation(FreeCAD.Vector(0,0,1), -angle4).toMatrix()
            mat1 = rot * mat0 * rot2 * rot3 * rot4
            mat1.move(fp.master_gear_connector_parent.Placement.Base)
            fp.slave_gear_connector_parent.Placement = mat1

        if ((isinstance(fp.master_gear.Proxy, InvoluteGear) and isinstance(fp.slave_gear.Proxy, InvoluteGearRack))
            or (isinstance(fp.master_gear.Proxy, CycloidGear) and isinstance(fp.slave_gear.Proxy, CycloidGearRack))):
            angle_master = fp.master_gear_connector_parent.Placement.Rotation.Angle * sum(fp.master_gear_connector_parent.Placement.Rotation.Axis)
            dw_master = fp.master_gear.dw.Value
            dw_slave = 0
            dist = -(dw_master + dw_slave) / 2
            mat0 = FreeCAD.Matrix()  # unity matrix
            mat0.move(FreeCAD.Vector(dist, 0, 0))
            mat1 = FreeCAD.Matrix()
            mat1.move(FreeCAD.Vector(0, np.deg2rad(fp.angle1.Value) * dw_master / 2, 0))
            mat2 = FreeCAD.Matrix()
            mat2.move(FreeCAD.Vector(0, -np.deg2rad(fp.angle2.Value) * dw_master / 2, 0))
            rot = FreeCAD.Rotation(FreeCAD.Vector(0,0,1), fp.angle1).toMatrix()
            mat3 = rot * mat2 *mat1 * mat0
            mat3.move(fp.master_gear_connector_parent.Placement.Base)
            fp.slave_gear_connector_parent.Placement = mat3

        if isinstance(fp.master_gear.Proxy, CycloidGear) and isinstance(fp.slave_gear.Proxy, CycloidGear):
            angle_master = fp.master_gear_connector_parent.Placement.Rotation.Angle * sum(fp.master_gear_connector_parent.Placement.Rotation.Axis)
            dw_master = fp.master_gear.dw
            dw_slave = fp.slave_gear.dw
            dist = (dw_master + dw_slave) / 2
            mat0 = FreeCAD.Matrix()  # unity matrix
            trans = FreeCAD.Vector(dist, 0, 0)
            mat0.move(trans)
            rot = FreeCAD.Rotation(FreeCAD.Vector(0,0,1), fp.angle1).toMatrix()
            angle2 = dw_master / dw_slave * fp.angle1.Value
            angle4 = dw_master / dw_slave * np.rad2deg(angle_master)
            rot2 = FreeCAD.Rotation(FreeCAD.Vector(0,0,1), angle2).toMatrix()
            angle3 = abs(fp.slave_gear.teeth % 2 - 1) * 180. / fp.slave_gear.teeth
            rot3 = FreeCAD.Rotation(FreeCAD.Vector(0,0,1), angle3).toMatrix()
            rot4 = FreeCAD.Rotation(FreeCAD.Vector(0,0,1), -angle4).toMatrix()
            mat1 = rot * mat0 * rot2 * rot3 * rot4
            mat1.move(fp.master_gear_connector_parent.Placement.Base)
            fp.slave_gear_connector_parent.Placement = mat1

    def execute(self, fp):
        self.onChanged(fp, None)

            



