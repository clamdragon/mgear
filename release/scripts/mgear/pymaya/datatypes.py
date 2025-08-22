import math
import types
from maya.api import OpenMaya
from . import util
from .util import degrees


class Const():
    def __init__(self, data):
        self.data = data

    def __get__(self, obj, par_cls):
        return par_cls(self.data)

    def __set__(self, *args):
        raise AttributeError("Cannot set a constant!")

degrees = util.degrees
radians = util.radians
Space = OpenMaya.MSpace

pymel_om_kwarg_map = {
    "tol": "tolerance",
}

def _warp_dt(func):
    def wrapper(*args, **kwargs):
        # do a bit of kwarg adjusting
        for a, b in pymel_om_kwarg_map.items():
            val = kwargs.pop(a, None)
            if val:
                kwargs[b] = val

        res = func(*args, **kwargs)
        # isinstance also catches subclasses, essentially just duplicating
        # the already correct result. we ONLY want API classes.
        # if isinstance(res, OpenMaya.MVector):
        if type(res) is OpenMaya.MVector:
            return Vector(res)
        elif type(res) is OpenMaya.MPoint:
            return Point(res)
        elif type(res) is OpenMaya.MEulerRotation:
            return EulerRotation(res)
        elif type(res) is OpenMaya.MMatrix:
            return Matrix(res)
        elif type(res) is OpenMaya.MQuaternion:
            return Quaternion(res)
        elif type(res) is OpenMaya.MTransformationMatrix:
            return TransformationMatrix(res.asMatrix())
        else:
            return res

    return wrapper






class Vector(OpenMaya.MVector):
    WRAP_FUNCS = []
    l = locals()
    for fn in dir(OpenMaya.MVector):
        if not fn.startswith("_"):
            f = getattr(OpenMaya.MVector, fn)
            if callable(f):
                # class and static methods
                if isinstance(f, types.BuiltinFunctionType):
                    l[fn] = _warp_dt(f)
                else:
                    WRAP_FUNCS.append(fn)

    xAxis = Const(OpenMaya.MVector.kXaxisVector)
    xNegAxis = Const(OpenMaya.MVector.kXnegAxisVector)
    yAxis = Const(OpenMaya.MVector.kYaxisVector)
    yNegAxis = Const(OpenMaya.MVector.kYnegAxisVector)
    zAxis = Const(OpenMaya.MVector.kZaxisVector)
    zNegAxis = Const(OpenMaya.MVector.kZnegAxisVector)

    def __init__(self, *args, **kwargs):
        if len(args) == 1:
            args = list(args)
            if isinstance(args[0], (list, tuple)) and len(args[0]) == 1:
                l = list(args[0])
                args[0] = l * 3
            elif isinstance(args[0], (int, float)):
                args[0] = [args[0]] * 3

        super(Vector, self).__init__(*args, **kwargs)
        for fn in Vector.WRAP_FUNCS:
            setattr(
                self, fn, _warp_dt(super(Vector, self).__getattribute__(fn))
            )

    def __getitem__(self, item):
        return [self.x, self.y, self.z][item]

    def __add__(self, other):
        """Override addition to return a Vector instead of MVector."""
        return Vector(super(Vector, self).__add__(other))

    def __sub__(self, other):
        """Override subtraction to return a Vector instead of MVector."""
        return Vector(super(Vector, self).__sub__(other))

    def __mul__(self, other):
        """Override multiplication to return a Vector or scalar product."""
        result = super(Vector, self).__mul__(other)
        if isinstance(other, (int, float)):  # Scalar multiplication
            return Vector(result)
        return result  # Dot product returns a scalar

    def __truediv__(self, other):
        """Override division to return a Vector."""
        return Vector(super(Vector, self).__truediv__(other))

    def __neg__(self):
        """Override negation to return a Vector."""
        return Vector(super(Vector, self).__neg__())

    def __repr__(self):
        return "Vector({})".format(self.tolist())

    def tolist(self):
        return [self.x, self.y, self.z]

    def get(self):
        return self.tolist()

    def rotateBy(self, *args):
        if args:
            if len(args) == 2 and isinstance(args[0], Vector):
                return Vector(
                    super(Vector, self).rotateBy(
                        Quaternion(float(args[1]), args[0])
                    )
                )
            elif len(args) == 1 and isinstance(args[0], Matrix):
                return Vector(
                    super(Vector, self).rotateBy(
                        TransformationMatrix(args[0]).rotation(True)
                    )
                )
            else:
                return Vector(
                    super(Vector, self).rotateBy(EulerRotation(*args))
                )
        else:
            return self

    def projectionOnto(self, pVector):
        """Project this vector onto anpVector vector.

        Args:
            pVector (Vector): The vector onto which to project this vector.

        Returns:
            Vector: The projected vector.
        """
        pVector = Vector(pVector)  # Ensure input is a Vector
        scale = (self * pVector) / (pVector * pVector)
        return pVector * scale


class Point(OpenMaya.MPoint):
    WRAP_FUNCS = []
    l = locals()
    for fn in dir(OpenMaya.MPoint):
        if not fn.startswith("_"):
            f = getattr(OpenMaya.MPoint, fn)
            if callable(f):
                # class and static methods
                if isinstance(f, types.BuiltinFunctionType):
                    l[fn] = _warp_dt(f)
                else:
                    WRAP_FUNCS.append(fn)

    def __init__(self, *args, **kwargs):
        super(Point, self).__init__(*args, **kwargs)
        for fn in Point.WRAP_FUNCS:
            setattr(
                self, fn, _warp_dt(super(Point, self).__getattribute__(fn))
            )

    def tolist(self):
        return [self.x, self.y, self.z]

    def asVector(self):
        return Vector(self.x, self.y, self.z)

    def __getitem__(self, item):
        return [self.x, self.y, self.z, self.w][item]

    def __repr__(self):
        return "Point({})".format(self.tolist())


class Matrix(OpenMaya.MMatrix):
    WRAP_FUNCS = []
    l = locals()
    for fn in dir(OpenMaya.MMatrix):
        if not fn.startswith("_"):
            f = getattr(OpenMaya.MMatrix, fn)
            if callable(f):
                # class and static methods
                if isinstance(f, types.BuiltinFunctionType):
                    l[fn] = _warp_dt(f)
                else:
                    WRAP_FUNCS.append(fn)

    def __init__(self, *args, **kwargs):
        if len(args) == 16:
            args = (args,)
        self.base = super(Matrix, self)
        self.base.__init__(*args, **kwargs)
        for fn in Matrix.WRAP_FUNCS:
            setattr(
                self, fn, _warp_dt(self.base.__getattribute__(fn))
            )

    def get(self):
        gt = self.base.__getitem__
        return (
            (gt(0), gt(1), gt(2), gt(3)),
            (gt(4), gt(5), gt(6), gt(7)),
            (gt(8), gt(9), gt(10), gt(11)),
            (gt(12), gt(13), gt(14), gt(15)),
        )

    def __setitem__(self, index, value):
        if index < 0 or index > 3:
            raise Exception("list index out of range")

        if len(value) > 4:
            raise Exception("over 4 values given")

        for i, v in enumerate(value):
            self.base.__setitem__(index * 4 + i, v)

    def __getitem__(self, index):
        if index < 0 or index > 3:
            raise Exception("list index out of range")

        gt = self.base.__getitem__
        return [
            gt(index * 4),
            gt(index * 4 + 1),
            gt(index * 4 + 2),
            gt(index * 4 + 3),
        ]

    def __mul__(self, other):
        return Matrix(self.base.__mul__(other))

    def __imul__(self, other):
        return Matrix(self.base.__imul__(other))

    def __rmul__(self, other):
        return Matrix(self.base.__rmul__(other))

    def __repr__(self):
        return "Matrix({})".format([self[0], self[1], self[2], self[3]])

    @property
    def translate(self):
        return Vector(self[3][0:3])

    @translate.setter
    def translate(self, value):
        self[3] = value

    @property
    def rotate(self):
        q = Quaternion()
        q.setValue(self)
        return q

    @rotate.setter
    def rotate(self, value):
        scale = self.scale
        # euler? meh pymel dosn't, so pass
        # if len(value) == 3
        if len(value) == 4:
            m = OpenMaya.MQuaternion(value).asMatrix()
        elif isinstance(value, OpenMaya.MQuaternion):
            m = value.asMatrix()
        else:
            raise ValueError("Must set rotation with a quaternion or 4 numbers [i, j, k, l]")

        self[0] = [m[0] * scale[0], m[1] * scale[0], m[2] * scale[0], 0]
        self[1] = [m[4] * scale[1], m[5] * scale[1], m[6] * scale[1], 0]
        self[2] = [m[8] * scale[2], m[9] * scale[2], m[10] * scale[2], 0]

    @property
    def scale(self):
        v = Vector()
        for i in range(3):
            v[i] = math.sqrt(sum( [math.pow(e, 2) for e in self[i]] ))
        return v

    @scale.setter
    def scale(self, value):
        if not len(value) == 3:
            raise ValueError("Must set scale using three values")

        for i, s in enumerate(value):
            v = self[i]
            curr = math.sqrt(sum(math.pow(vc, 2) for vc in v))
            mult = s / curr
            self[i] = [vc * mult for vc in v]



def _trnsfrommatrix_wrp(func, this):
    def wrapper(*args, **kwargs):
        return getattr(OpenMaya.MTransformationMatrix(this), func)(
            *args, **kwargs
        )

    return _warp_dt(wrapper)


class TransformationMatrix(Matrix):
    WRAP_FUNCS = []
    ORG_MEMS = []
    l = locals()
    for fn in dir(OpenMaya.MTransformationMatrix):
        if not fn.startswith("_"):
            f = getattr(OpenMaya.MTransformationMatrix, fn)
            if callable(f):
                # class and static methods
                if isinstance(f, types.BuiltinFunctionType):
                    l[fn] = _warp_dt(f)
                else:
                    WRAP_FUNCS.append(fn)

    def __init__(self, *args, **kwargs):
        super(TransformationMatrix, self).__init__(*args, **kwargs)
        for fn in TransformationMatrix.WRAP_FUNCS:
            if not hasattr(self, fn):
                setattr(self, fn, _trnsfrommatrix_wrp(fn, self))
        for m in TransformationMatrix.ORG_MEMS:
            setattr(self, m, getattr(OpenMaya.MTransformationMatrix, m))

    def __repr__(self):
        return (
            super(TransformationMatrix, self)
            .__repr__()
            .replace("MMatrix", "TransformationMatrix")
        )

    def get(self):
        return self.asMatrix().get()

    def __copy(self, other):
        if not isinstance(other, Matrix):
            other = Matrix(other)
        self[0] = other[0]
        self[1] = other[1]
        self[2] = other[2]
        self[3] = other[3]

    def setRotationQuaternion(self, x, y, z, w):
        t = OpenMaya.MTransformationMatrix(self)
        t.setRotation(Quaternion(x, y, z, w))
        self.__copy(t.asMatrix())

    def getRotationQuaternion(self):
        q = self.rotation().asQuaternion()
        return (q.x, q.y, q.z, q.w)

    def getRotation(self):
        return self.rotation()

    def setRotation(self, *args):
        if len(args) == 1 and isinstance(args[0], list):
            args = args[0]
        t = OpenMaya.MTransformationMatrix(self)
        t.setRotation(EulerRotation(*[math.radians(x) for x in args]))
        self.__copy(t.asMatrix())

    def getScale(self, space):
        return self.scale(util.to_mspace(space))

    def setScale(self, scale, space):
        t = OpenMaya.MTransformationMatrix(self)
        t.setScale(scale, util.to_mspace(space))
        self.__copy(t.asMatrix())

    def setShear(self, shear, space):
        t = OpenMaya.MTransformationMatrix(self)
        t.setShear(shear, util.to_mspace(space))
        self.__copy(t.asMatrix())

    def getShear(self, space):
        return self.shear(util.to_mspace(space))

    def setTranslation(self, vector, space):
        t = OpenMaya.MTransformationMatrix(self)
        t.setTranslation(vector, util.to_mspace(space))
        self.__copy(t.asMatrix())

    def getTranslation(self, space):
        return self.translation(util.to_mspace(space))


class BoundingBox(OpenMaya.MBoundingBox):
    WRAP_FUNCS = []
    l = locals()
    for fn in dir(OpenMaya.MBoundingBox):
        if not fn.startswith("_"):
            f = getattr(OpenMaya.MBoundingBox, fn)
            if callable(f):
                # class and static methods
                if isinstance(f, types.BuiltinFunctionType):
                    l[fn] = _warp_dt(f)
                else:
                    WRAP_FUNCS.append(fn)

    def __init__(self, *args, **kwargs):
        nargs = []
        for arg in args:
            if isinstance(arg, (list, tuple)):
                arg = Point(arg)
            nargs.append(arg)

        super(BoundingBox, self).__init__(*nargs, **kwargs)
        for fn in BoundingBox.WRAP_FUNCS:
            setattr(
                self,
                fn,
                _warp_dt(super(BoundingBox, self).__getattribute__(fn)),
            )

    def __getitem__(self, index):
        if index == 0:
            return Point(self.min)
        elif index == 1:
            return Point(self.max)
        else:
            raise Exception("Index out of range")


class Quaternion(OpenMaya.MQuaternion):
    WRAP_FUNCS = []
    l = locals()
    for fn in dir(OpenMaya.MQuaternion):
        if not fn.startswith("_"):
            f = getattr(OpenMaya.MQuaternion, fn)
            if callable(f):
                # class and static methods
                if isinstance(f, types.BuiltinFunctionType):
                    l[fn] = _warp_dt(f)
                else:
                    WRAP_FUNCS.append(fn)

    def __init__(self, *args, **kwargs):
        super(Quaternion, self).__init__(*args, **kwargs)

        for fn in Quaternion.WRAP_FUNCS:
            setattr(
                self,
                fn,
                _warp_dt(super(Quaternion, self).__getattribute__(fn)),
            )

    def scaleIt(self, scal):
        return Quaternion(
            self.x * scal, self.y * scal, self.z * scal, self.w * scal
        )

    def __repr__(self):
        return "Quaternion({})".format([self.x, self.y, self.z, self.w])


class EulerRotation(OpenMaya.MEulerRotation):
    WRAP_FUNCS = []
    l = locals()
    for fn in dir(OpenMaya.MEulerRotation):
        if not fn.startswith("_"):
            f = getattr(OpenMaya.MEulerRotation, fn)
            if callable(f):
                # class and static methods attach to class,
                # so in __init__ is not good enough
                if isinstance(f, types.BuiltinFunctionType):
                    l[fn] = _warp_dt(f)
                else:
                    WRAP_FUNCS.append(fn)

    def __init__(self, *args, **kwargs):
        # uiu = OpenMaya.MAngle.uiUnit()
        if args and isinstance(args[0], EulerRotation):
            unit = args[0].unit
        else:
            # alright, a lot has been rewritten to use radians
            # so I guess that should be the default
            unit = "radians"
        # elif isinstance(args[0], OpenMaya.MEulerRotation):
        #     unit = "radians"
        # elif uiu == 2:
        #     unit = "degrees"
        # else:
        #     unit = "radians"

        self._unit = kwargs.pop("unit", unit)

        self.base = super(EulerRotation, self)
        self.base.__init__(*args, **kwargs)

        # double calls, but that's what we want
        if self._unit == "degrees":
            self.x = radians(self.x)
            self.y = radians(self.y)
            self.z = radians(self.z)

        for fn in EulerRotation.WRAP_FUNCS:
            setattr(
                self,
                fn,
                _warp_dt(super(EulerRotation, self).__getattribute__(fn)),
            )

    # def toList(self):
    #     return self._convert_from_api([self.x, self.y, self.z])

    @property
    def unit(self):
        return self._unit

    @unit.setter
    def unit(self, val):
        if not val in ("degrees", "radians"):
            raise ValueError("Unit must be degrees or radians.")
        else:
            # underlying values are the same, no need to do anything
            self._unit = val
            # if not unit == self.unit:
            #     curr = (self.x, self.y, self.z)
            #     if unit == "degrees":


    def _convert_to_api(self, v):
        if self.unit == "degrees":
            return radians(v)
        else:
            return v

    def _convert_from_api(self, v):
        if self.unit == "degrees":
            return degrees(v)
        else:
            return v

    def __getitem__(self, item):
        return self._convert_from_api(self.base.__getitem__(item))

    def __setitem__(self, key, value):
        self.base.__setitem__(key, self._convert_to_api(value))

    def __repr__(self):
        return "EulerRotation({})".format([self.x, self.y, self.z])

    def __mul__(self, other):
        return EulerRotation(self.base.__mul__(other))

    def __imul__(self, other):
        return EulerRotation(self.base.__imul__(other))

    def __rmul__(self, other):
        return EulerRotation(self.base.__rmul__(other))

    @property
    def x(self):
        return self[0]

    @x.setter
    def x(self, val):
        self[0] = val

    @property
    def y(self):
        return self[1]

    @y.setter
    def y(self, val):
        self[1] = val

    @property
    def z(self):
        return self[2]

    @z.setter
    def z(self, val):
        self[2] = val


# they already use the class?
# degrees = _warp_dt(util.degrees)
# radians = _warp_dt(util.radians)

__all__ = [
    "Vector",
    "EulerRotation",
    "Matrix",
    "TransformationMatrix",
    "Quaternion",
    "degrees",
    "radians",
    "Point",
    "BoundingBox",
    "Space",
]
