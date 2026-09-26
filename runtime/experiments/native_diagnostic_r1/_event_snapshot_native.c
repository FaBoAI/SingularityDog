#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <limits.h>
#include <math.h>

/* Owned built-in-only traversal. Never release the GIL or call copy hooks.
 * Counter values use a fast unsigned representation and promote to PyLong
 * when required, preserving the reference's unbounded integer accounting. */
typedef struct { unsigned long long small; PyObject *large; } Count;
typedef struct {
    int max_depth;
    Count nodes_left, bytes_left;
    PyObject *ancestors[65];
    int active;
} Context;

static void count_clear(Count *a) { Py_XDECREF(a->large); a->large = NULL; }

static int count_set(Count *a, PyObject *value) {
    a->large = NULL;
    a->small = PyLong_AsUnsignedLongLong(value);
    if (!PyErr_Occurred()) return 0;
    if (!PyErr_ExceptionMatches(PyExc_OverflowError)) return -1;
    PyErr_Clear();
    Py_INCREF(value); a->large = value; a->small = 0;
    return 0;
}

static PyObject *count_object(const Count *a) {
    if (a->large) { Py_INCREF(a->large); return a->large; }
    return PyLong_FromUnsignedLongLong(a->small);
}

static int count_add(Count *a, const Count *b) {
    if (!a->large && !b->large && b->small <= ULLONG_MAX - a->small) {
        a->small += b->small; return 0;
    }
    PyObject *left = count_object(a), *right = count_object(b), *sum;
    if (!left || !right) { Py_XDECREF(left); Py_XDECREF(right); return -1; }
    sum = PyNumber_Add(left, right);
    Py_DECREF(left); Py_DECREF(right);
    if (!sum) return -1;
    count_clear(a); a->large = sum; a->small = 0;
    return 0;
}

static int count_add_small(Count *a, unsigned long long value) {
    Count b = {value, NULL}; return count_add(a, &b);
}

static int count_scale(Count *out, unsigned long long value, unsigned int factor,
                       unsigned int plus) {
    out->large = NULL;
    if (value <= (ULLONG_MAX - plus) / factor) {
        out->small = value * factor + plus; return 0;
    }
    /* Overflow paths are unreachable with ordinary event sizes, but exact. */
    out->small = value;
    Count original = {value, NULL};
    for (unsigned int i = 1; i < factor; ++i)
        if (count_add(out, &original) < 0) return -1;
    return count_add_small(out, plus);
}

static int consume(Count *budget, const Count *cost, const char *message) {
    if (!budget->large && !cost->large) {
        if (cost->small > budget->small) {
            PyErr_SetString(PyExc_ValueError, message); return -1;
        }
        budget->small -= cost->small; return 0;
    }
    PyObject *left = count_object(budget), *right = count_object(cost), *remainder;
    if (!left || !right) { Py_XDECREF(left); Py_XDECREF(right); return -1; }
    int too_small = PyObject_RichCompareBool(left, right, Py_LT);
    if (too_small) {
        Py_DECREF(left); Py_DECREF(right);
        if (too_small > 0) PyErr_SetString(PyExc_ValueError, message);
        return -1;
    }
    remainder = PyNumber_Subtract(left, right);
    Py_DECREF(left); Py_DECREF(right);
    if (!remainder) return -1;
    count_clear(budget);
    int ok = count_set(budget, remainder);
    Py_DECREF(remainder);
    return ok;
}

static int is_container(PyObject *item) {
    return PyDict_CheckExact(item) || PyList_CheckExact(item) || PyTuple_CheckExact(item);
}

static int scalar_cost(PyObject *item, Count *cost) {
    cost->small = 0; cost->large = NULL;
    if (PyUnicode_CheckExact(item))
        return count_scale(cost, (unsigned long long)PyUnicode_GET_LENGTH(item), 12, 2);
    if (PyLong_CheckExact(item)) {
#if PY_VERSION_HEX < 0x030E0000
        /* CPython-only API, declared in Python.h on supported 3.8--3.13. */
        size_t bits = _PyLong_NumBits(item);
        if (bits == (size_t)-1 && PyErr_Occurred()) return -1;
        return count_scale(cost, (unsigned long long)bits, 1, 2);
#else
        /* Do not depend on private long layout or changed 3.14 internals. */
        PyObject *bits = PyObject_CallMethod(item, "bit_length", NULL);
        if (!bits) return -1;
        int ok = count_set(cost, bits);
        Py_DECREF(bits);
        return ok < 0 ? -1 : count_add_small(cost, 2);
#endif
    }
    if (PyFloat_CheckExact(item)) {
        if (!isfinite(PyFloat_AS_DOUBLE(item))) {
            PyErr_SetString(PyExc_ValueError, "Nonfinite event number"); return -1;
        }
        cost->small = 32; return 0;
    }
    if (item == Py_None) { cost->small = 4; return 0; }
    if (PyBool_Check(item)) { cost->small = 5; return 0; }
    PyErr_SetString(PyExc_TypeError, "Unsupported event value type"); return -1;
}

static PyObject *visit(Context *, PyObject *, int);

static PyObject *copy_child(Context *ctx, PyObject *child, int depth, Count *local) {
    if (is_container(child)) return visit(ctx, child, depth + 1);
    Count cost = {0, NULL};
    int ok = scalar_cost(child, &cost);
    if (ok == 0) ok = count_add(local, &cost);
    count_clear(&cost);
    if (ok < 0) return NULL;
    Py_INCREF(child); return child;
}

static PyObject *visit(Context *ctx, PyObject *item, int depth) {
    PyObject *result = NULL;
    Count local = {0, NULL}, node_cost = {0, NULL};
    int pushed = 0;
    int is_dict = PyDict_CheckExact(item), is_tuple = PyTuple_CheckExact(item);
    Py_ssize_t size = is_dict ? PyDict_Size(item) :
        is_tuple ? PyTuple_GET_SIZE(item) : PyList_GET_SIZE(item);
    if (depth > ctx->max_depth || (size && depth >= ctx->max_depth)) {
        PyErr_SetString(PyExc_ValueError, "Event snapshot depth bound exceeded"); return NULL;
    }
    if (count_scale(&node_cost, (unsigned long long)size, is_dict ? 2 : 1, 0) < 0)
        goto done;
    if (consume(&ctx->nodes_left, &node_cost, "Event snapshot node bound exceeded") < 0)
        goto done;
    if (count_scale(&local, (unsigned long long)size, is_dict ? 4 : 2, 2) < 0)
        goto done;
    for (int i = 0; i < ctx->active; ++i) {
        if (ctx->ancestors[i] == item) {
            PyErr_SetString(PyExc_ValueError, "Cyclic event container"); goto done;
        }
    }
    if (Py_EnterRecursiveCall(" while copying an event snapshot")) goto done;
    ctx->ancestors[ctx->active++] = item; pushed = 1;
    result = is_dict ? PyDict_New() : is_tuple ? PyTuple_New(size) : PyList_New(size);
    if (!result) goto done;
    if (is_dict) {
        Py_ssize_t pos = 0;
        PyObject *key, *child;
        while (PyDict_Next(item, &pos, &key, &child)) {
            if (!PyUnicode_CheckExact(key)) {
                PyErr_SetString(PyExc_TypeError, "Event dictionary keys must be built-in strings");
                goto fail;
            }
            /* Own borrowed references before recursive allocations. */
            Py_INCREF(key); Py_INCREF(child);
            Count key_cost = {0, NULL};
            int ok = count_scale(&key_cost,
                (unsigned long long)PyUnicode_GET_LENGTH(key), 12, 2);
            if (ok == 0) ok = count_add(&local, &key_cost);
            count_clear(&key_cost);
            PyObject *copied = ok < 0 ? NULL : copy_child(ctx, child, depth, &local);
            Py_DECREF(child);
            if (copied) { ok = PyDict_SetItem(result, key, copied); Py_DECREF(copied); }
            else ok = -1;
            Py_DECREF(key);
            if (ok < 0) goto fail;
            if (PyDict_Size(item) != size) {
                PyErr_SetString(PyExc_RuntimeError, "dictionary changed size during iteration");
                goto fail;
            }
        }
    } else {
        for (Py_ssize_t i = 0; i < size; ++i) {
            PyObject *child = is_tuple ? PyTuple_GET_ITEM(item, i) : PyList_GetItem(item, i);
            if (!child) goto fail;
            Py_INCREF(child);
            PyObject *copied = copy_child(ctx, child, depth, &local);
            Py_DECREF(child);
            if (!copied) goto fail;
            if (is_tuple) PyTuple_SET_ITEM(result, i, copied);
            else PyList_SET_ITEM(result, i, copied);
        }
    }
    if (consume(&ctx->bytes_left, &local, "Event snapshot byte bound exceeded") < 0) goto fail;
    goto done;
fail:
    Py_CLEAR(result);
done:
    if (pushed) { --ctx->active; Py_LeaveRecursiveCall(); }
    count_clear(&local); count_clear(&node_cost);
    return result;
}

PyDoc_STRVAR(snapshot_doc,
"snapshot_event($module, value, *, max_depth=24, max_nodes=20000, max_bytes=1048576)\n"
"--\n\n"
"Copy finite exact built-in JSON values with owned mutable descendants.\n"
"Depth, node and conservative byte accounting match event_snapshot.py.");

static PyObject *snapshot_event(PyObject *self, PyObject *args, PyObject *kwargs) {
    (void)self;
    static char *keywords[] = {"value", "max_depth", "max_nodes", "max_bytes", NULL};
    PyObject *value, *depth_arg = NULL, *nodes_arg = NULL, *bytes_arg = NULL;
    Context ctx = {24, {20000, NULL}, {1048576, NULL}, {NULL}, 0};
    PyObject *result = NULL;
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "O|$OOO:snapshot_event", keywords,
            &value, &depth_arg, &nodes_arg, &bytes_arg)) return NULL;
    if (depth_arg) {
        if (!PyLong_CheckExact(depth_arg)) goto invalid;
        long depth = PyLong_AsLong(depth_arg);
        if (PyErr_Occurred()) { PyErr_Clear(); goto invalid; }
        if (depth < 0 || depth > 64) goto invalid;
        ctx.max_depth = (int)depth;
    }
    PyObject *bounds[] = {nodes_arg, bytes_arg};
    Count *budgets[] = {&ctx.nodes_left, &ctx.bytes_left};
    for (int i = 0; i < 2; ++i) {
        if (!bounds[i]) continue;
        if (!PyLong_CheckExact(bounds[i])) goto invalid;
        if (count_set(budgets[i], bounds[i]) < 0) goto done;
        if (budgets[i]->large) {
            PyObject *zero = PyLong_FromLong(0);
            if (!zero) goto done;
            int positive = PyObject_RichCompareBool(bounds[i], zero, Py_GT);
            Py_DECREF(zero);
            if (positive < 0) goto done;
            if (!positive) goto invalid;
        } else if (budgets[i]->small == 0) goto invalid;
    }
    Count one = {1, NULL};
    if (consume(&ctx.nodes_left, &one, "Invalid event snapshot bounds") < 0) goto done;
    if (is_container(value)) result = visit(&ctx, value, 0);
    else {
        Count cost = {0, NULL};
        if (scalar_cost(value, &cost) == 0 &&
            consume(&ctx.bytes_left, &cost, "Event snapshot byte bound exceeded") == 0) {
            Py_INCREF(value); result = value;
        }
        count_clear(&cost);
    }
    goto done;
invalid:
    PyErr_SetString(PyExc_ValueError, "Invalid event snapshot bounds");
done:
    count_clear(&ctx.nodes_left); count_clear(&ctx.bytes_left);
    return result;
}

static PyMethodDef methods[] = {
    {"snapshot_event", (PyCFunction)(void(*)(void))snapshot_event,
        METH_VARARGS | METH_KEYWORDS, snapshot_doc},
    {NULL, NULL, 0, NULL}
};
static struct PyModuleDef module = {
    PyModuleDef_HEAD_INIT, "_event_snapshot_native",
    "CPython owned bounded built-in event copier; no I/O and no GIL release.",
    -1, methods, NULL, NULL, NULL, NULL
};
PyMODINIT_FUNC PyInit__event_snapshot_native(void) { return PyModule_Create(&module); }
