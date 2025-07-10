import pymel.core as pm
from pymel.core import datatypes

from mgear.shifter import component
import ast

from mgear.core import node, fcurve, applyop, vector, curve
from mgear.core import attribute, transform, primitive, string

#############################################
# COMPONENT
#############################################


class Component(component.Main):
    """Shifter component Class"""
    axes = ["XZ", "XY", "YZ", "YX", "ZY", "ZX"]

    # =====================================================
    # OBJECTS
    # =====================================================
    def addObjects(self):
        """Add all the objects needed to create the component."""

        # joint Description Names
        jd_names = ast.literal_eval(
            self.settings["jointNamesDescription_custom"]
        )
        jdn_neck = jd_names[0]
        jdn_head = jd_names[1]

        axes = ["x", "y", "z", "-x", "-y", "-z"]
        aim_axis = axes[self.settings.get("aimAxis", 1)]
        bend_axis = axes[self.settings.get("bendAxis", 0)]
        if aim_axis[-1] == bend_axis[-1]:
            # naughty naughty
            raise ValueError("Aim and bend axes must not match.")
        normal_axis = next(a for a in "xyz" if a not in aim_axis + bend_axis)
        rot_order = (aim_axis[-1] + bend_axis[-1] + normal_axis[-1]).upper()
        mirror_attrs = ["t"+bend_axis[-1], "r"+aim_axis[-1], "r"+normal_axis[-1]]
        axis_vectors = {
            "x": datatypes.Vector.xAxis,
            "y": datatypes.Vector.yAxis,
            "z": datatypes.Vector.zAxis,
            "-x": datatypes.Vector.xNegAxis,
            "-y": datatypes.Vector.yNegAxis,
            "-z": datatypes.Vector.zNegAxis,
        }
        # default was YX so just make a matrix replacing those axes with aim/bend
        ctrl_rot_offset = datatypes.EulerRotation.decompose(
            datatypes.Matrix(
                axis_vectors[bend_axis],
                axis_vectors[aim_axis],
                axis_vectors[normal_axis],
                datatypes.Vector()
            ),
            "XYZ"
        )

        self.normal = self.guide.blades["blade"].z * -1
        self.up_axis = pm.upAxis(q=True, axis=True)

        # Ik Controlers ------------------------------------
        if self.settings["IKWorldOri"]:
            t = datatypes.TransformationMatrix()
            ik_mirror_attrs = ["tx", "ry", "rz"]
            ik_rot_offset = datatypes.EulerRotation()
        else:
            t = transform.getTransformLookingAt(
                self.guide.pos["tan1"],
                self.guide.pos["neck"],
                self.normal,
                aim_axis + bend_axis,
                self.negate,
            )
            ik_mirror_attrs = mirror_attrs
            ik_rot_offset = ctrl_rot_offset

        t = transform.setMatrixPosition(t, self.guide.pos["neck"])

        self.ik_off = primitive.addTransform(
            self.root, self.getName("ik_off"), t
        )
        # handle Z up orientation offset
        if self.up_axis == "z" and self.settings["IKWorldOri"]:
            self.ik_off.rx.set(90)
            t = transform.getTransform(self.ik_off)

        self.ik_cns = primitive.addTransform(
            self.ik_off, self.getName("ik_cns"), t
        )

        self.ik_ctl = self.addCtl(
            self.ik_cns,
            "ik_ctl",
            t,
            self.color_ik,
            "compas",
            w=self.size * 0.5,
            ro=ik_rot_offset,
            tp=self.parentCtlTag,
        )

        attribute.setKeyableAttributes(self.ik_ctl, self.tr_params)
        attribute.setRotOrder(self.ik_ctl, rot_order)
        attribute.setInvertMirror(self.ik_ctl, ik_mirror_attrs)

        # Tangents -----------------------------------------
        if self.settings["tangentControls"]:
            t = transform.setMatrixPosition(t, self.guide.pos["tan1"])

            self.tan1_loc = primitive.addTransform(
                self.ik_ctl, self.getName("tan1_loc"), t
            )

            self.tan1_ctl = self.addCtl(
                self.tan1_loc,
                "tan1_ctl",
                t,
                self.color_ik,
                "sphere",
                w=self.size * 0.2,
                tp=self.ik_ctl,
            )

            attribute.setKeyableAttributes(self.tan1_ctl, self.t_params)
            attribute.setInvertMirror(self.tan1_ctl, ik_mirror_attrs)

            t = transform.getTransformLookingAt(
                self.guide.pos["root"],
                self.guide.pos["tan0"],
                self.normal,
                aim_axis + bend_axis,
                self.negate,
            )

            t = transform.setMatrixPosition(t, self.guide.pos["tan0"])

            self.tan0_loc = primitive.addTransform(
                self.root, self.getName("tan0_loc"), t
            )

            self.tan0_ctl = self.addCtl(
                self.tan0_loc,
                "tan0_ctl",
                t,
                self.color_ik,
                "sphere",
                w=self.size * 0.2,
                tp=self.ik_ctl,
            )

            attribute.setKeyableAttributes(self.tan0_ctl, self.t_params)
            attribute.setInvertMirror(self.tan0_ctl, ik_mirror_attrs)

            # Curves -------------------------------------------
            self.mst_crv = curve.addCnsCurve(
                self.root,
                self.getName("mst_crv"),
                [self.root, self.tan0_ctl, self.tan1_ctl, self.ik_ctl],
                3,
            )

            self.slv_crv = curve.addCurve(
                self.root,
                self.getName("slv_crv"),
                [datatypes.Vector()] * 10,
                False,
                3,
            )

            self.mst_crv.setAttr("visibility", False)

        else:
            t = transform.setMatrixPosition(t, self.guide.pos["tan1"])
            self.tan1_loc = primitive.addTransform(
                self.ik_ctl, self.getName("tan1_loc"), t
            )

            t = transform.getTransformLookingAt(
                self.guide.pos["root"],
                self.guide.pos["tan0"],
                self.normal,
                aim_axis + bend_axis,
                self.negate,
            )

            t = transform.setMatrixPosition(t, self.guide.pos["tan0"])

            self.tan0_loc = primitive.addTransform(
                self.root, self.getName("tan0_loc"), t
            )

            # Curves -------------------------------------------
            self.mst_crv = curve.addCnsCurve(
                self.root,
                self.getName("mst_crv"),
                [self.root, self.tan0_loc, self.tan1_loc, self.ik_ctl],
                3,
            )

            self.slv_crv = curve.addCurve(
                self.root,
                self.getName("slv_crv"),
                [datatypes.Vector()] * 10,
                False,
                3,
            )

        self.mst_crv.setAttr("visibility", False)
        self.slv_crv.setAttr("visibility", False)
        pm.xform(self.mst_crv, t=(0, 0, 0), ro=(0, 0, 0), s=(1, 1, 1), sh=(0, 0, 0))
        pm.xform(self.slv_crv, t=(0, 0, 0), ro=(0, 0, 0), s=(1, 1, 1), sh=(0, 0, 0))

        # Division -----------------------------------------
        # The user only define how many intermediate division he wants.
        # First and last divisions are an obligation.
        parentdiv = primitive.addTransform(
            self.root,
            self.getName("div_par"),
            transform.getTransform(self.root),
        )
        parentctl = self.root
        self.div_cns = []
        self.fk_ctl = []
        self.fk_npo = []
        self.scl_ref = []

        self.twister = []
        self.ref_twist = []

        # adding 1 for the head
        self.divisions = self.settings["division"] + 1

        parent_twistRef = primitive.addTransform(
            self.root,
            self.getName("reference"),
            transform.getTransform(self.root),
        )

        t = transform.getTransformLookingAt(
            self.guide.pos["root"],
            self.guide.pos["neck"],
            self.normal,
            aim_axis + bend_axis,
            self.negate,
        )

        self.intMRef = primitive.addTransform(
            self.root, self.getName("intMRef"), t
        )

        self.previousCtlTag = self.parentCtlTag
        for i in range(self.divisions):

            # References
            div_cns = primitive.addTransform(
                parentdiv, self.getName("%s_cns" % i), t
            )

            # pair with localizing motionpath to fix scaling
            # pm.setAttr(div_cns + ".inheritsTransform", False)
            self.div_cns.append(div_cns)
            # parentdiv = div_cns

            # Controlers (First and last one are fake)
            if i == self.divisions - 1:
                fk_ctl = primitive.addTransform(
                    parentctl,
                    self.getName("%s_loc" % i),
                    transform.getTransform(parentctl),
                )

                fk_npo = fk_ctl
            else:
                # all but the last one (head)
                fk_npo = primitive.addTransform(
                    parentctl,
                    self.getName("fk%s_npo" % i),
                    transform.getTransform(parentctl),
                )

                fk_ctl = self.addCtl(
                    fk_npo,
                    "fk%s_ctl" % i,
                    transform.getTransform(parentctl),
                    self.color_fk,
                    "cube",
                    w=self.size * 0.2,
                    h=self.size * 0.05,
                    d=self.size * 0.2,
                    ro = ctrl_rot_offset,
                    tp=self.previousCtlTag,
                )

                attribute.setKeyableAttributes(self.fk_ctl)
                attribute.setRotOrder(fk_ctl, rot_order)

                self.previousCtlTag = fk_ctl

                scl_ref = primitive.addTransform(
                    fk_ctl,
                    self.getName("%s_scl_ref" % i),
                    transform.getTransform(parentctl),
                )
                self.scl_ref.append(scl_ref)

                if i == 0:
                    guide_relative = "root"
                else:
                    guide_relative = None

                self.jnt_pos.append(
                    {
                        "obj": scl_ref,
                        "name": string.replaceSharpWithPadding(
                            jdn_neck, i + 1
                        ),
                        "guide_relative": guide_relative,
                        "data_contracts": "Twist,Squash",
                        "leaf_joint": self.settings["leafJoints"],
                    }
                )

            self.fk_ctl.append(fk_ctl)
            self.fk_npo.append(fk_npo)
            parentctl = fk_ctl

            t = transform.getTransformLookingAt(
                self.guide.pos["root"],
                self.guide.pos["neck"],
                self.normal,
                aim_axis + bend_axis,
                self.negate,
            )

            twister = primitive.addTransform(
                parent_twistRef, self.getName("%s_rot_ref" % i), t
            )

            ref_twist = primitive.addTransform(
                parent_twistRef, self.getName("%s_pos_ref" % i), t
            )

            ref_twist.setTranslation(
                axis_vectors[bend_axis], space="preTransform"
            )

            self.twister.append(twister)
            self.ref_twist.append(ref_twist)

        for x in self.fk_ctl[:-1]:
            attribute.setInvertMirror(x, mirror_attrs)

        # Head ---------------------------------------------
        t = transform.getTransformLookingAt(
            self.guide.pos["head"],
            self.guide.pos["eff"],
            self.normal,
            aim_axis + bend_axis,
            self.negate,
        )

        self.head_cns = primitive.addTransform(
            self.root, self.getName("head_cns"), t
        )

        dist = vector.getDistance(
            self.guide.pos["head"], self.guide.pos["eff"]
        )

        self.head_ctl = self.addCtl(
            self.head_cns,
            "head_ctl",
            t,
            self.color_fk,
            "cube",
            w=self.size * 0.5,
            h=dist,
            d=self.size * 0.5,
            po=axis_vectors[aim_axis] * dist * 0.5,
            ro=ctrl_rot_offset,
            tp=self.previousCtlTag,
        )

        head_ref = primitive.addTransform(
                self.head_ctl,
                self.getName("head_scl_ref"),
                t,
            )
        self.scl_ref.append(head_ref)

        attribute.setRotOrder(self.head_ctl, rot_order)
        attribute.setInvertMirror(self.head_ctl, mirror_attrs)

        self.jnt_pos.append(
            {
                "obj": head_ref,
                "name": jdn_head,
                "guide_relative": "neck",
            }
        )

    # =====================================================
    # ATTRIBUTES
    # =====================================================

    def addAttributes(self):
        """Create the anim and setupr rig attributes for the component"""
        # Anim -------------------------------------------
        self.maxstretch_att = self.addAnimParam(
            "maxstretch",
            "Max Stretch",
            "double",
            self.settings["maxstretch"],
            1,
        )

        self.maxsquash_att = self.addAnimParam(
            "maxsquash",
            "MaxSquash",
            "double",
            self.settings["maxsquash"],
            0,
            1,
        )

        self.softness_att = self.addAnimParam(
            "softness", "Softness", "double", self.settings["softness"], 0, 1
        )

        self.lock_ori_att = self.addAnimParam(
            "lock_ori", "Lock Ori", "double", 1, 0, 1
        )

        self.tan0_att = self.addAnimParam("tan0", "Tangent 0", "double", 1, 0)
        self.tan1_att = self.addAnimParam("tan1", "Tangent 1", "double", 1, 0)

        # Volume
        self.volume_att = self.addAnimParam(
            "volume", "Volume", "double", 1, 0, 1
        )

        # Ref
        if self.settings["ikrefarray"]:
            ref_names = self.get_valid_alias_list(
                self.settings["ikrefarray"].split(",")
            )
            if len(ref_names) > 1:
                self.ikref_att = self.addAnimEnumParam(
                    "ikref", "Ik Ref", 0, ref_names
                )

        if self.settings["headrefarray"]:
            ref_names = self.get_valid_alias_list(
                self.settings["headrefarray"].split(",")
            )
            if len(ref_names) > 1:
                ref_names.insert(0, "self")
                self.headref_att = self.addAnimEnumParam(
                    "headref", "Head Ref", 0, ref_names
                )

        # Setup ------------------------------------------
        # Eval Fcurve
        if self.guide.paramDefs["st_profile"].value:
            self.st_value = self.guide.paramDefs["st_profile"].value
            self.sq_value = self.guide.paramDefs["sq_profile"].value
        else:
            self.st_value = fcurve.getFCurveValues(
                self.settings["st_profile"], self.divisions
            )
            self.sq_value = fcurve.getFCurveValues(
                self.settings["sq_profile"], self.divisions
            )

        self.st_att = [
            self.addSetupParam(
                "stretch_%s" % i,
                "Stretch %s" % i,
                "double",
                self.st_value[i],
                -1,
                0,
            )
            for i in range(self.divisions)
        ]

        self.sq_att = [
            self.addSetupParam(
                "squash_%s" % i,
                "Squash %s" % i,
                "double",
                self.sq_value[i],
                0,
                1,
            )
            for i in range(self.divisions)
        ]

    # =====================================================
    # OPERATORS
    # =====================================================
    def addOperators(self):
        """Create operators and set the relations for the component rig

        Apply operators, constraints, expressions to the hierarchy.
        In order to keep the code clean and easier to debug,
        we shouldn't create any new object in this method.

        """
        axes = ["x", "y", "z", "-x", "-y", "-z"]
        aim_axis = axes[self.settings.get("aimAxis", 1)]
        bend_axis = axes[self.settings.get("bendAxis", 3)]

        # Tangent position ---------------------------------
        # common part
        d = vector.getDistance(self.guide.pos["root"], self.guide.pos["neck"])
        dist_node = node.createDistNode(self.root, self.ik_ctl)
        rootWorld_node = node.createDecomposeMatrixNode(
            self.root.attr("worldMatrix")
        )
        div_node = node.createDivNode(
            dist_node + ".distance", rootWorld_node + ".outputScaleX"
        )
        div_node = node.createDivNode(div_node + ".outputX", d)

        # tan0
        # sometimes it's not perfectly vertical - blend ALL translate channels
        mul_node = node.createMulNode(
            [self.tan0_att, self.tan0_att, self.tan0_att],
            list(self.tan0_loc.getAttr("translate"))
        )
        res_node = node.createMulNode(
            mul_node + ".output",
            [div_node + ".outputX",
            div_node + ".outputX",
            div_node + ".outputX"]
        )
        pm.connectAttr(res_node + ".output", self.tan0_loc.attr("translate"))

        # tan1
        mul_node = node.createMulNode(
            [self.tan1_att, self.tan1_att, self.tan1_att],
            list(self.tan1_loc.getAttr("translate"))
        )
        res_node = node.createMulNode(
            mul_node + ".output",
            [div_node + ".outputX",
            div_node + ".outputX",
            div_node + ".outputX"]
        )
        pm.connectAttr(res_node + ".output", self.tan1_loc.attr("translate"))

        # Curves -------------------------------------------
        op = applyop.gear_curveslide2_op(
            self.slv_crv, self.mst_crv, 0, 1.5, 0.5, 0.5
        )
        pm.connectAttr(self.maxstretch_att, op + ".maxstretch")
        pm.connectAttr(self.maxsquash_att, op + ".maxsquash")
        pm.connectAttr(self.softness_att, op + ".softness")

        # Volume driver ------------------------------------
        crv_node = node.createCurveInfoNode(self.slv_crv)

        # Division -----------------------------------------
        for i in range(self.divisions):

            # References
            u = i / (self.divisions - 1.0)

            # localize motion path, better/simpler scaling
            cns = applyop.pathCns(
                self.div_cns[i], self.slv_crv, False, u, True, True
            )
            cns.setAttr("frontAxis", aim_axis.upper()[-1])  # bone axis
            cns.setAttr("inverseFront", "-" in aim_axis)
            cns.setAttr("upAxis", bend_axis.upper()[-1]) # bend axis
            cns.setAttr("inverseUp", "-" in bend_axis)

            # Roll
            # first, offset IK control wm to align with general aim xform (intMRef)
            # can ignore translation because it's not used
            ik_t = transform.getTransform(self.ik_ctl)
            fk_t = transform.getTransform(self.intMRef)
            mm_node = node.createMultMatrixNode(
                fk_t * ik_t.inverse(), self.ik_ctl + ".worldMatrix"
            )
            # localize IK ctrl input
            self.root.wim >> mm_node.i[2]
            # localize base mtx too
            intMatrix = applyop.gear_intmatrix_op(
                self.intMRef + ".matrix", mm_node + ".matrixSum", u
            )
            dm_node = node.createDecomposeMatrixNode(intMatrix + ".output")
            pm.connectAttr(
                dm_node + ".outputRotate", self.twister[i].attr("rotate")
            )

            pm.parentConstraint(
                self.twister[i], self.ref_twist[i], maintainOffset=True
            )

            pm.connectAttr(
                self.ref_twist[i] + ".translate", cns + ".worldUpVector"
            )

            # Squash n Stretch
            op = applyop.gear_squashstretch2_op(
                self.scl_ref[i], self.root, pm.arclen(self.slv_crv), aim_axis
            )

            pm.connectAttr(self.volume_att, op + ".blend")
            pm.connectAttr(crv_node + ".arcLength", op + ".driver")
            pm.connectAttr(self.st_att[i], op + ".stretch")
            pm.connectAttr(self.sq_att[i], op + ".squash")
            op.setAttr("driver_min", 0.1)


            # Controlers
            # ik drive fk
            mtx_cns_node = applyop.gear_matrix_cns(self.div_cns[i], self.fk_npo[i])
            mtx_cns_node.drivenRestMatrix.set(datatypes.Matrix())
            if i == 0:
                # fk0 doesn't need scale inversion, that is all taken care of at the root
                pass
            else:
                par_ctl = self.fk_ctl[i - 1]
                inv_scl_node = node.createDivNode(
                    [1, 1, 1],
                    [par_ctl.sx, par_ctl.sy, par_ctl.sz]
                )
                pm.connectAttr(inv_scl_node.output, mtx_cns_node.drivenInverseScale)
                pm.connectAttr(
                    self.div_cns[i - 1].worldInverseMatrix,
                    mtx_cns_node.drivenParentInverseMatrix,
                    force=True
                )

            # Orientation Lock
            if i == self.divisions - 1:
                fk_t = transform.getTransform(self.div_cns[i])
                # fk ctrl in ik ctrl space give the offset we need to align them
                mm_node = node.createMultMatrixNode(
                    fk_t * ik_t.inverse(), self.ik_ctl + ".worldMatrix"
                )
                # localize IK ctrl input
                self.root.wim >> mm_node.i[2]
                dm_node = node.createDecomposeMatrixNode(mm_node + ".matrixSum")
                blend_node = node.createBlendNode(
                    [dm_node + ".outputRotate%s" % s for s in "XYZ"],
                    [cns + ".rotate%s" % s for s in "XYZ"],
                    self.lock_ori_att,
                )
                self.div_cns[i].attr("rotate").disconnect()

                pm.connectAttr(
                    blend_node + ".output", self.div_cns[i] + ".rotate"
                )

            if self.options["force_SSC"]:
                op.global_scale.disconnect()
                op.global_scale.set(1, 1, 1)

        # Head ---------------------------------------------
        self.fk_ctl[-1].addChild(self.head_cns)

        if self.options["force_SSC"]:
            # do it a little different. want to negate only immediate parent's scale
            pass


    # =====================================================
    # CONNECTOR
    # =====================================================
    def setRelation(self):
        """Set the relation beetween object from guide to rig"""
        self.relatives["root"] = self.fk_ctl[0]
        self.relatives["tan1"] = self.fk_ctl[0]
        self.relatives["tan2"] = self.scl_ref[-1]
        self.relatives["neck"] = self.scl_ref[-1]
        self.relatives["head"] = self.scl_ref[-1]
        self.relatives["eff"] = self.scl_ref[-1]

        self.controlRelatives["root"] = self.fk_ctl[0]
        self.controlRelatives["tan1"] = self.fk_ctl[0]
        self.controlRelatives["tan2"] = self.fk_ctl[-1]
        self.controlRelatives["neck"] = self.head_ctl
        self.controlRelatives["head"] = self.head_ctl
        self.controlRelatives["eff"] = self.head_ctl

        self.jointRelatives["root"] = 0
        self.jointRelatives["tan1"] = 0
        self.jointRelatives["tan2"] = len(self.jnt_pos) - 1
        self.jointRelatives["neck"] = len(self.jnt_pos) - 1
        self.jointRelatives["head"] = len(self.jnt_pos) - 1
        self.jointRelatives["eff"] = len(self.jnt_pos) - 1

        self.aliasRelatives["tan1"] = "root"
        self.aliasRelatives["tan2"] = "head"
        self.aliasRelatives["neck"] = "head"
        self.aliasRelatives["eff"] = "head"

    def connect_standard(self):
        self.connect_standardWithIkRef()

    def connect_standardWithIkRef(self):

        self.parent.addChild(self.root)
        # self.scale_in_root_space.i[0].set(self.root.m.get())
        # self.scale_in_root_space.i[2].set(self.root.im.get())
        # self.parent.m >> self.root_ssc_mtx.imat
        if self.options["force_SSC"]:
            # finally, something that functions identically to SSC
            # ssc_mtx_cns = applyop.gear_matrix_cns(self.root.m)
            ssc_mtx_cns = pm.nt.Mgear_matrixConstraint()
            # don't want PIM connected, it can all be local
            ssc_mtx_cns.driverMatrix.set(self.root.m.get())
            # connect *all*
            ssc_mtx_cns.translate >> self.root.t
            ssc_mtx_cns.rotate >> self.root.r
            ssc_mtx_cns.scale >> self.root.s
            ssc_mtx_cns.shear >> self.root.sh
            # inverted parent scale
            root_scl_inv = pm.nt.MultiplyDivide()
            root_scl_inv.operation.set(2)
            root_scl_inv.input1.set(1, 1, 1)
            self.parent.scale >> root_scl_inv.input2
            root_scl_inv.o >> ssc_mtx_cns.drivenInverseScale

        self.connectRef(self.settings["ikrefarray"], self.ik_cns)

        if not self.settings["chickenStyleIK"]:
            for axis in ["tx", "ty", "tz"]:
                self.ik_cns.attr(axis).disconnect()

        if self.settings["headrefarray"]:
            ref_names = self.settings["headrefarray"].split(",")

            ref = []
            for ref_name in ref_names:
                ref.append(self.rig.findRelative(ref_name))

            ref.append(self.head_cns)
            cns_node = pm.parentConstraint(
                *ref, skipTranslate="none", maintainOffset=True
            )

            cns_attr = pm.parentConstraint(
                cns_node, query=True, weightAliasList=True
            )
            self.head_cns.attr("tx").disconnect()
            self.head_cns.attr("ty").disconnect()
            self.head_cns.attr("tz").disconnect()

            for i, attr in enumerate(cns_attr):
                node_name = pm.createNode("condition")
                pm.connectAttr(self.headref_att, node_name + ".firstTerm")
                pm.setAttr(node_name + ".secondTerm", i + 1)
                pm.setAttr(node_name + ".operation", 0)
                pm.setAttr(node_name + ".colorIfTrueR", 1)
                pm.setAttr(node_name + ".colorIfFalseR", 0)
                pm.connectAttr(node_name + ".outColorR", attr)
