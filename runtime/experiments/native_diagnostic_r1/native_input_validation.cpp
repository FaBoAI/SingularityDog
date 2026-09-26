#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <cmath>
#include <cstdint>
#include <cstring>

/* Only owned, exact built-in values reach this helper from the Python adapter.
 * No GIL release, device access, hashing, conversion hooks, or fast-math. */
enum Key {
    KIND, OK, CYCLE, PARAMETER, MOTOR_ID, WRITE_ENTERED, WRITE_EXPECTED, WRITE_RETURNED,
    RESULT, BOTH, RAW, WIRE, CAN_ID, TYPE, SOURCE, DESTINATION, FLAGS, DATA,
    MODE, FAULTS, POSITION_U16, VELOCITY_U16, TORQUE_U16, TEMPERATURE_U16,
    POSITION, VELOCITY, TORQUE, TEMPERATURE,
    START, FINISH, RECEIVED, DEADLINE, AVAILABLE,
    APPROVED, CALIBRATION, VELOCITY_SCALE, PIPELINE, CONTROLLER, ENABLE,
    MOTION, CONFIGURATION, RETRY, BITRATE, LATE_REPLY, SENSOR_TIME, UNWRAPPING,
    KEY_COUNT
};
static const char *names[KEY_COUNT] = {
    "kind", "ok", "cycle", "parameter", "motor_id", "write_call_entered",
    "write_expected_bytes", "write_returned_bytes", "result",
    "position_velocity_in_one_reply", "raw_frame", "wire_hex", "can_id", "type",
    "source_id", "destination_id", "flags", "data_hex", "mode_state", "fault_bits",
    "position_u16", "velocity_u16", "torque_u16", "temperature_u16",
    "position_rad_candidate", "velocity_rad_s_candidate", "torque_nm_candidate", "temperature_c",
    "write_started_monotonic_ns", "write_finished_monotonic_ns", "received_monotonic_ns",
    "deadline_monotonic_ns", "available_monotonic_ns",
    "approved_for_runtime", "calibration_verified", "velocity_scale_verified",
    "full_pipeline_20ms_verified", "full_controller_50Hz_verified", "motor_enabling_available",
    "motion_command_available", "configuration_available", "automatic_retry", "can_bitrate_verified",
    "late_same_key_previous_cycle_disambiguation", "sensor_internal_sample_time_verified", "unwrapping_applied"
};
struct State { PyObject *keys[KEY_COUNT]; };
struct Owned {
    PyObject *value;
    explicit Owned(PyObject *p = nullptr) : value(p) {}
    ~Owned() { Py_XDECREF(value); }
    Owned(const Owned &) = delete;
    Owned &operator=(const Owned &) = delete;
    PyObject *release() { PyObject *p=value; value=nullptr; return p; }
};

static bool fail(const char *message) {
    PyErr_SetString(PyExc_ValueError, message); return false;
}
static PyObject *get(State *state, PyObject *dict, Key key) {
    return PyDict_GetItemWithError(dict, state->keys[key]);
}
static bool equal_text(PyObject *value, const char *text) {
    return value && PyUnicode_CheckExact(value) && PyUnicode_CompareWithASCIIString(value,text)==0;
}
static bool equal_integer(PyObject *value, long long expected) {
    if (!value || !PyLong_CheckExact(value)) return false;
    long long actual=PyLong_AsLongLong(value);
    if (PyErr_Occurred()) { PyErr_Clear(); return false; }
    return actual==expected;
}
static bool stamp(PyObject *value, const char *label, uint64_t *out) {
    if (!value || !PyLong_CheckExact(value)) {
        PyErr_Format(PyExc_ValueError,"Invalid %s",label); return false;
    }
    unsigned long long number=PyLong_AsUnsignedLongLong(value);
    if (PyErr_Occurred()) {
        PyErr_Clear(); PyErr_Format(PyExc_ValueError,"Invalid %s",label); return false;
    }
    if (number >= (UINT64_C(1)<<63)) {
        PyErr_Format(PyExc_ValueError,"Invalid %s",label); return false;
    }
    *out=number; return true;
}
static bool finite_equal(PyObject *value, const char *label, double expected) {
    double actual;
    if (value && PyFloat_CheckExact(value)) actual=PyFloat_AS_DOUBLE(value);
    else if (value && PyLong_CheckExact(value)) {
        actual=PyLong_AsDouble(value);
        if (PyErr_Occurred()) {
            PyErr_Clear(); PyErr_Format(PyExc_ValueError,"Nonfinite %s",label); return false;
        }
    } else {
        PyErr_Format(PyExc_ValueError,"Invalid %s",label); return false;
    }
    if (!std::isfinite(actual)) {
        PyErr_Format(PyExc_ValueError,"Nonfinite %s",label); return false;
    }
    /* Expected scales are <6554, so integer-to-double rounding cannot cause
     * an unequal integer to compare equal to any expected result. */
    return actual==expected || fail("Candidate scale differs from raw16");
}
static int hex_digit(Py_UCS4 c) {
    if (c>='0' && c<='9') return int(c-'0');
    if (c>='a' && c<='f') return int(c-'a'+10);
    if (c>='A' && c<='F') return int(c-'A'+10);
    return -1;
}
static bool ascii_space(Py_UCS4 c) {
    return c==' ' || c=='\t' || c=='\n' || c=='\r' || c=='\v' || c=='\f';
}
static bool wire_bytes(PyObject *text, unsigned char (&wire)[17]) {
    if (!text || !PyUnicode_CheckExact(text) || PyUnicode_GET_LENGTH(text)!=34)
        return fail("Invalid raw wire hex");
    Py_ssize_t index=0, count=0;
    while (index<34) {
        Py_UCS4 c=PyUnicode_READ_CHAR(text,index);
        if (ascii_space(c)) { ++index; continue; }
        int high=hex_digit(c);
        if (high<0 || index+1>=34) return fail("Invalid raw wire hex");
        int low=hex_digit(PyUnicode_READ_CHAR(text,index+1));
        if (low<0) return fail("Invalid raw wire hex");
        wire[count++]=static_cast<unsigned char>((high<<4)|low);
        index+=2;
    }
    if (count!=17 || wire[0]!=0x41 || wire[1]!=0x54 || wire[6]!=8 || wire[15]!=13 || wire[16]!=10)
        return fail("Noncanonical raw Type2 wire");
    static const char hex[]="0123456789abcdef";
    for (int i=0;i<17;++i)
        if (PyUnicode_READ_CHAR(text,2*i)!=static_cast<Py_UCS4>(hex[wire[i]>>4]) ||
            PyUnicode_READ_CHAR(text,2*i+1)!=static_cast<Py_UCS4>(hex[wire[i]&15]))
            return fail("Noncanonical raw Type2 wire");
    return true;
}
static PyObject *hex_text(const unsigned char *bytes, int length) {
    static const char hex[]="0123456789abcdef";
    char result[34];
    for (int i=0;i<length;++i) { result[i*2]=hex[bytes[i]>>4]; result[i*2+1]=hex[bytes[i]&15]; }
    return PyUnicode_FromStringAndSize(result,length*2);
}
static bool set_owned(State *state, PyObject *dict, Key key, PyObject *value) {
    Owned owned(value);
    return value && PyDict_SetItem(dict,state->keys[key],value)==0;
}

static PyObject *motor(State *state, PyObject *row, int cycle, int *selected_id) {
    if (!PyDict_CheckExact(row) || !equal_text(get(state,row,KIND),"pipeline_reply") ||
        get(state,row,OK)!=Py_True || !equal_integer(get(state,row,CYCLE),cycle) ||
        !equal_text(get(state,row,PARAMETER),"stop_feedback")) {
        fail("Expected successful combined reply in the selected cycle"); return nullptr;
    }
    PyObject *mid_object=get(state,row,MOTOR_ID);
    long mid=0;
    if (mid_object && PyLong_CheckExact(mid_object)) {
        mid=PyLong_AsLong(mid_object);
        if (PyErr_Occurred()) { PyErr_Clear(); mid=0; }
    }
    if (mid<1 || mid>12) { fail("Invalid motor ID"); return nullptr; }
    if (get(state,row,WRITE_ENTERED)!=Py_True || !equal_integer(get(state,row,WRITE_EXPECTED),17) ||
        !equal_integer(get(state,row,WRITE_RETURNED),17)) {
        fail("Require a complete canonical request write"); return nullptr;
    }
    PyObject *result=get(state,row,RESULT);
    if (!result || !PyDict_CheckExact(result) || get(state,result,OK)!=Py_True ||
        !equal_integer(get(state,result,MOTOR_ID),mid) ||
        !equal_text(get(state,result,PARAMETER),"stop_feedback")) {
        fail("Mismatched decoded result"); return nullptr;
    }
    for (int i=APPROVED;i<=UNWRAPPING;++i)
        if (get(state,result,static_cast<Key>(i))!=Py_False) {
            fail("Require explicit unverified diagnostic flags"); return nullptr;
        }
    if (get(state,result,BOTH)!=Py_True) {
        fail("Expected position and velocity in one reply"); return nullptr;
    }
    PyObject *raw=get(state,result,RAW);
    if (!raw || !PyDict_CheckExact(raw)) { fail("Missing original Type2 raw frame"); return nullptr; }
    unsigned char wire[17];
    if (!wire_bytes(get(state,raw,WIRE),wire)) return nullptr;
    uint32_t encoded=(uint32_t(wire[2])<<24)|(uint32_t(wire[3])<<16)|(uint32_t(wire[4])<<8)|wire[5];
    uint32_t can_id=encoded>>3, flags=encoded&7;
    if (flags!=4 || can_id!=((uint32_t(2)<<24)|(uint32_t(mid)<<8)|0xfd)) {
        fail("Type2 must match ID/host and mode0/fault0"); return nullptr;
    }
    if (wire[7]==0 && wire[8]==0xc4 && wire[9]==0x56) {
        fail("Version-shaped reply is not ordinary Type2"); return nullptr;
    }
    Owned expected_raw(PyDict_New());
    if (!expected_raw.value ||
        !set_owned(state,expected_raw.value,CAN_ID,PyLong_FromUnsignedLong(can_id)) ||
        !set_owned(state,expected_raw.value,TYPE,PyLong_FromLong(2)) ||
        !set_owned(state,expected_raw.value,SOURCE,PyLong_FromLong(mid)) ||
        !set_owned(state,expected_raw.value,DESTINATION,PyLong_FromLong(0xfd)) ||
        !set_owned(state,expected_raw.value,FLAGS,PyLong_FromUnsignedLong(flags)) ||
        !set_owned(state,expected_raw.value,DATA,hex_text(wire+7,8)) ||
        !set_owned(state,expected_raw.value,WIRE,hex_text(wire,17))) return nullptr;
    bool raw_ok=PyDict_Size(raw)==7;
    const Key raw_keys[]={CAN_ID,TYPE,SOURCE,DESTINATION,FLAGS,DATA,WIRE};
    if (raw_ok) for (Key key:raw_keys) {
        PyObject *actual=get(state,raw,key), *expected=get(state,expected_raw.value,key);
        if (!actual || Py_TYPE(actual)!=Py_TYPE(expected) || PyObject_RichCompareBool(actual,expected,Py_EQ)!=1) {
            raw_ok=false; break;
        }
    }
    if (!raw_ok) { fail("Raw-frame metadata differs from wire"); return nullptr; }
    unsigned int p=(unsigned(wire[7])<<8)|wire[8], v=(unsigned(wire[9])<<8)|wire[10];
    unsigned int torque=(unsigned(wire[11])<<8)|wire[12], temp=(unsigned(wire[13])<<8)|wire[14];
    const Key decoded_keys[]={MODE,FAULTS,POSITION_U16,VELOCITY_U16,TORQUE_U16,TEMPERATURE_U16};
    const long decoded_values[]={0,0,long(p),long(v),long(torque),long(temp)};
    for (int i=0;i<6;++i) if (!equal_integer(get(state,result,decoded_keys[i]),decoded_values[i])) {
        fail("Decoded mode/fault or raw16 differs from wire"); return nullptr;
    }
    /* Match the Python multiply/divide/subtract sequence, without contraction. */
    volatile double p_scaled=double(p)*(2.*12.57), v_scaled=double(v)*(2.*50.);
    volatile double t_scaled=double(torque)*(2.*5.5);
    volatile double p_unit=p_scaled/65535., v_unit=v_scaled/65535., t_unit=t_scaled/65535.;
    double position=p_unit-12.57, velocity=v_unit-50., torque_value=t_unit-5.5, temperature=double(temp)/10.;
    const Key scaled_keys[]={POSITION,VELOCITY,TORQUE,TEMPERATURE};
    const double scaled_values[]={position,velocity,torque_value,temperature};
    for (int i=0;i<4;++i)
        if (!finite_equal(get(state,result,scaled_keys[i]),names[scaled_keys[i]],scaled_values[i])) return nullptr;
    const Key stamp_keys[]={START,FINISH,RECEIVED,DEADLINE};
    PyObject *stamp_objects[5]; uint64_t times[5];
    for (int i=0;i<4;++i) {
        stamp_objects[i]=get(state,row,stamp_keys[i]);
        if (!stamp(stamp_objects[i],names[stamp_keys[i]],&times[i])) return nullptr;
    }
    stamp_objects[4]=get(state,row,AVAILABLE);
    if (!stamp_objects[4]) stamp_objects[4]=stamp_objects[2];
    if (!stamp(stamp_objects[4],"reply availability",&times[4])) return nullptr;
    if (!(times[0]<=times[1] && times[1]<=times[2] && times[2]<=times[4] &&
          times[2]<times[3] && times[3]<=times[0]+UINT64_C(250000000))) {
        fail("Noncausal or expired source interval"); return nullptr;
    }
    if (!(std::isfinite(position) && position>=-12.57 && position<=12.57 &&
          std::isfinite(velocity) && velocity>=-50. && velocity<=50.)) {
        fail("Candidate position or velocity outside declared range"); return nullptr;
    }
    Owned values(Py_BuildValue("(iiiddOOOOO)",int(mid),int(can_id),int(flags),position,velocity,
        stamp_objects[0],stamp_objects[1],stamp_objects[2],stamp_objects[3],stamp_objects[4]));
    if (!values.value) return nullptr;
    *selected_id=int(mid);
    return PyTuple_Pack(3,values.value,expected_raw.value,row);
}

static PyObject *validate_motors(PyObject *module, PyObject *args) {
    PyObject *rows, *cycle_object;
    if (!PyArg_ParseTuple(args,"OO:validate_motors",&rows,&cycle_object)) return nullptr;
    int cycle=0;
    if (PyLong_CheckExact(cycle_object)) {
        long n=PyLong_AsLong(cycle_object);
        if (PyErr_Occurred()) PyErr_Clear();
        else if (n>=1 && n<=20) cycle=int(n);
    }
    if (!cycle) { fail("Invalid fixed cycle"); return nullptr; }
    bool list=PyList_CheckExact(rows), tuple=PyTuple_CheckExact(rows);
    if ((!list && !tuple) || (list ? PyList_GET_SIZE(rows) : PyTuple_GET_SIZE(rows))!=12) {
        fail("Exactly twelve successful same-cycle replies are required"); return nullptr;
    }
    State *state=static_cast<State *>(PyModule_GetState(module));
    Owned output(PyTuple_New(12));
    if (!output.value) return nullptr;
    unsigned int seen=0;
    for (Py_ssize_t i=0;i<12;++i) {
        PyObject *row=list ? PyList_GetItem(rows,i) : PyTuple_GET_ITEM(rows,i);
        if (!row) return nullptr;
        Py_INCREF(row); Owned held(row);
        int mid;
        Owned record(motor(state,row,cycle,&mid));
        if (!record.value) return nullptr;
        unsigned int bit=1u<<(mid-1);
        if (seen&bit) { fail("Repeated motor ID"); return nullptr; }
        seen|=bit;
        PyTuple_SET_ITEM(output.value,mid-1,record.release());
    }
    if (seen!=0xfff) { fail("All twelve IDs must be present"); return nullptr; }
    return output.release();
}

static int traverse(PyObject *module, visitproc visit, void *arg) {
    State *state=static_cast<State *>(PyModule_GetState(module));
    for (int i=0;i<KEY_COUNT;++i) Py_VISIT(state->keys[i]);
    return 0;
}
static int clear(PyObject *module) {
    State *state=static_cast<State *>(PyModule_GetState(module));
    for (int i=0;i<KEY_COUNT;++i) Py_CLEAR(state->keys[i]);
    return 0;
}
static void free_module(void *module) { clear(static_cast<PyObject *>(module)); }
static PyMethodDef methods[]={
    {"validate_motors",validate_motors,METH_VARARGS,
     "Validate twelve already-owned built-in reply rows; return ordered motor tuples, canonical raw frames and rows."},
    {nullptr,nullptr,0,nullptr}
};
static PyModuleDef module={
    PyModuleDef_HEAD_INIT,"_native_input_validation",
    "CPython native exact Type2 validation only; no I/O, hashing or GIL release.",
    sizeof(State),methods,nullptr,traverse,clear,free_module
};
PyMODINIT_FUNC PyInit__native_input_validation(void) {
    Owned result(PyModule_Create(&module));
    if (!result.value) return nullptr;
    State *state=static_cast<State *>(PyModule_GetState(result.value));
    for (int i=0;i<KEY_COUNT;++i) {
        state->keys[i]=PyUnicode_InternFromString(names[i]);
        if (!state->keys[i]) return nullptr;
    }
    return result.release();
}
