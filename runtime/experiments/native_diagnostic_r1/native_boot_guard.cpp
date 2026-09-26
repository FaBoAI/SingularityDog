#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <cerrno>
#include <climits>
#include <cstring>
#include <sys/stat.h>
#include <unistd.h>

// Deliberately no Py_BEGIN_ALLOW_THREADS: this private diagnostic helper keeps
// the GIL across one fresh, offset-zero, at-most-80-byte pread. Regular files
// are accepted for offline tests, not claimed to be bounded/nonblocking I/O.
// Runtime callers must use only the owned genuine procfs boot-id descriptor.
static constexpr unsigned kMaxEintr = 8;
static bool strip_byte(unsigned char c) {
    return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\v' || c == '\f';
}

static PyObject* fresh_boot_matches(PyObject*, PyObject* args) {
    PyObject *fd_object, *expected;
    if (!PyArg_UnpackTuple(args, "fresh_boot_matches", 2, 2, &fd_object, &expected))
        return nullptr;
    if (!PyLong_CheckExact(fd_object)) {
        PyErr_SetString(PyExc_TypeError, "fd must be a built-in int");
        return nullptr;
    }
    long fd_long = PyLong_AsLong(fd_object);
    if (fd_long == -1 && PyErr_Occurred()) return nullptr;
    if (fd_long < 0 || fd_long > INT_MAX) {
        PyErr_SetString(PyExc_ValueError, "fd must be a nonnegative C int");
        return nullptr;
    }
    if (!PyBytes_CheckExact(expected)) {
        PyErr_SetString(PyExc_TypeError, "expected_bytes must be built-in bytes");
        return nullptr;
    }
    const Py_ssize_t expected_size = PyBytes_GET_SIZE(expected);
    if (expected_size > 80) {
        PyErr_SetString(PyExc_ValueError, "expected_bytes exceeds the 80-byte read bound");
        return nullptr;
    }
    const int fd = static_cast<int>(fd_long);
    struct stat info;
    unsigned interrupted = 0;
    while (fstat(fd, &info) < 0) {
        const int saved_errno = errno;
        if (saved_errno != EINTR) {
            errno = saved_errno;
            return PyErr_SetFromErrno(PyExc_OSError);
        }
        if (PyErr_CheckSignals() < 0) return nullptr;
        if (++interrupted >= kMaxEintr) {
            errno = EINTR;
            return PyErr_SetFromErrno(PyExc_OSError);
        }
    }
    if (!S_ISREG(info.st_mode)) {
        PyErr_SetString(PyExc_ValueError, "fresh_boot_matches requires a regular/proc file descriptor");
        return nullptr;
    }
    char buffer[80];
    ssize_t count;
    interrupted = 0;
    for (;;) {
        count = pread(fd, buffer, sizeof(buffer), 0);
        if (count >= 0) break;
        const int saved_errno = errno;
        if (saved_errno != EINTR) {
            errno = saved_errno;
            return PyErr_SetFromErrno(PyExc_OSError);
        }
        if (PyErr_CheckSignals() < 0) return nullptr;
        if (++interrupted >= kMaxEintr) {
            errno = EINTR;
            return PyErr_SetFromErrno(PyExc_OSError);
        }
    }
    // Match bytes.strip()'s six ASCII whitespace bytes, including empty reads.
    ssize_t first = 0, last = count;
    while (first < last && strip_byte(static_cast<unsigned char>(buffer[first]))) ++first;
    while (last > first && strip_byte(static_cast<unsigned char>(buffer[last - 1]))) --last;
    const bool matches = last - first == expected_size &&
        (expected_size == 0 || std::memcmp(buffer + first, PyBytes_AS_STRING(expected), expected_size) == 0);
    return PyBool_FromLong(matches);
}

static PyMethodDef methods[] = {
    {"fresh_boot_matches", fresh_boot_matches, METH_VARARGS,
     "Fresh bounded-size pread comparison; caller must hold its descriptor ownership lock."},
    {nullptr, nullptr, 0, nullptr}
};
static struct PyModuleDef module = {
    PyModuleDef_HEAD_INIT, "_native_boot_guard", "Private retained-GIL boot-id experiment.",
    -1, methods, nullptr, nullptr, nullptr, nullptr
};
PyMODINIT_FUNC PyInit__native_boot_guard(void) { return PyModule_Create(&module); }
