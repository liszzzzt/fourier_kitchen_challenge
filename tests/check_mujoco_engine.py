"""Check the standalone MJCF against a supplied MuJoCo shared C library.

Usage: python tests/check_mujoco_engine.py /path/to/libmujoco.so.2.3.6
No ROS, GUI, Python MuJoCo bindings, or GR3 body is exercised here.
"""
import ctypes as C
import math
from pathlib import Path
import sys

lib = C.CDLL(str(Path(sys.argv[1]).resolve()))
def api(name, result, *arguments):
    function = getattr(lib, name)
    function.restype, function.argtypes = result, list(arguments)
    return function

ptr, integer = C.c_void_p, C.c_int
load = api('mj_loadXML', ptr, C.c_char_p, ptr, ptr, integer)
make = api('mj_makeData', ptr, ptr)
step = api('mj_step', None, ptr, ptr)
size = api('mj_stateSize', integer, ptr, C.c_uint)
setstate = api('mj_setState', None, ptr, ptr, C.POINTER(C.c_double), C.c_uint)
getstate = api('mj_getState', None, ptr, ptr, C.POINTER(C.c_double), C.c_uint)
delete_data = api('mj_deleteData', None, ptr)
delete_model = api('mj_deleteModel', None, ptr)
version = api('mj_versionString', C.c_char_p)
path = Path(__file__).resolve().parents[1]/'simulation/grippers/robotiq_2f85/standalone.xml'
error = C.create_string_buffer(4096)
model = load(str(path).encode(), None, error, len(error))
if not model:
    raise RuntimeError(error.value.decode())
data = make(model)
if not data:
    delete_model(model)
    raise RuntimeError('mj_makeData failed')
try:
    assert size(model, 2) == 6 and size(model, 32) == 1
    print('MuJoCo C engine:', version().decode())
    q = (C.c_double*6)()
    signs = (1, -1, 1, 1, -1, 1)
    for target in (0., .8, 0.):
        setstate(model, data, (C.c_double*1)(target), 32)
        for _ in range(5000):
            step(model, data)
        getstate(model, data, q, 2)
        assert all(math.isfinite(x) and abs(x-sign*target) < .02 for x, sign in zip(q, signs))
        print('target=', target, 'qpos=', [round(x, 8) for x in q])
    print('PASS: empty-hand open/close/open; no full robot or ROS validation')
finally:
    delete_data(data)
    delete_model(model)
