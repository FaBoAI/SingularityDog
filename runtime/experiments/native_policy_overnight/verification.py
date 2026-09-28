"""Finite CPU replay; no live input, serial, network, or actuator functions."""
import math
import statistics
import time
from .contracts import INPUT_KEYS, require, sha, strict_json


def frame(torch, nominal, index):
    t = index * .11
    gyro = torch.tensor([[.035*math.sin(t), .02*math.cos(t), .025*math.sin(t*.3)]], dtype=torch.float32)
    gravity = torch.tensor([[.12*math.sin(t*.5), .10*math.cos(t*.7), -1.]], dtype=torch.float32)
    gravity /= torch.linalg.vector_norm(gravity, dim=1, keepdim=True)
    commands = ((0., 0., 0.), (.30, 0., 0.), (-.08, 0., 0.), (0., .08, 0.),
                (0., -.08, 0.), (0., 0., .18), (0., 0., -.18), (0., 0., 0.))
    command = torch.tensor([commands[(index//30) % 8]], dtype=torch.float32)
    wave = torch.sin(torch.arange(12, dtype=torch.float32)*.47+t).reshape(1, 12)
    return gyro, gravity, command, nominal+.025*wave, .06*wave, torch.full((1, 12), float((index//60) % 2))


def compare_tensor(torch, a, b, label, maxima, *, exact=False):
    require(a.shape == b.shape and a.dtype == b.dtype and a.device == b.device,
            "Tensor metadata mismatch: " + label)
    require(bool(torch.isfinite(a).all()) and bool(torch.isfinite(b).all()), "Nonfinite tensor: " + label)
    if a.dtype.is_floating_point:
        error = float((a-b).abs().max()) if a.numel() else 0.
        maxima[label] = max(maxima.get(label, 0.), error)
        if not exact:
            tolerance = dict(atol=1e-9, rtol=0.) if a.dtype == torch.float64 else dict(atol=1e-6, rtol=1e-6)
            torch.testing.assert_close(a, b, **tolerance)
            return
    require(torch.equal(a, b), "Exact tensor mismatch: " + label)


def compare_state(torch, a, b, maxima, *, exact=False):
    for accessor in ("named_buffers", "named_parameters"):
        one, two = dict(getattr(a, accessor)()), dict(getattr(b, accessor)())
        require(one.keys() == two.keys(), "State names mismatch: " + accessor)
        for key, value in one.items():
            compare_tensor(torch, value, two[key], accessor+":"+key, maxima,
                           exact=exact or accessor == "named_parameters")


def rejections(torch, base):
    cases = {}
    for index, name in enumerate(("gyro", "gravity", "command", "q", "dq", "h")):
        for suffix, value in (("nan", float("nan")), ("inf", float("inf"))):
            inputs = [x.clone() for x in base]
            inputs[index][0, 0] = value
            cases[name+"_"+suffix] = inputs
        inputs = [x.clone() for x in base]
        inputs[index] = inputs[index][:, :-1]
        cases[name+"_shape"] = inputs
    for name, index, value in (
        ("gravity_norm", 1, [[0., 0., -2.]]), ("gravity_heading_singular", 1, [[1., 0., 0.]]),
        ("mixed_command", 2, [[.1, .1, 0.]]), ("translation_and_yaw", 2, [[.1, 0., .1]]),
        ("forward_limit", 2, [[.6, 0., 0.]]), ("yaw_limit", 2, [[0., 0., .3]]),
        ("negative_h", 5, [[-1.]*12]), ("excess_h", 5, [[2.]*12])):
        inputs = [x.clone() for x in base]
        inputs[index] = torch.tensor(value, dtype=torch.float32)
        cases[name] = inputs
    return cases


def reject_reason(error):
    last = str(error).strip().splitlines()[-1]
    for prefix in ("builtins.ValueError: ", "builtins.RuntimeError: ", "ValueError: ", "RuntimeError: "):
        if last.startswith(prefix):
            return last[len(prefix):]
    return last


def validate(torch, reference, candidate, reloaded):
    """Current eager versus native within pinned tolerance; saved model exact."""
    ids = torch.tensor([0], dtype=torch.long)
    models = (reference, candidate, reloaded)
    nominal = reference.controller.nominal.float().reshape(1, 12).clone()
    frames = [frame(torch, nominal, k) for k in range(240)]
    maxima, reload_maxima = {}, {}
    with torch.inference_mode():
        for k, inputs in enumerate(frames):
            if k in (0, 125):
                for model in models:
                    model.reset(ids)
            saved = [x.clone() for x in inputs]
            values = [model(*inputs) for model in models]
            compare_tensor(torch, values[0], values[1], "target", maxima)
            compare_tensor(torch, values[1], values[2], "target", reload_maxima, exact=True)
            compare_state(torch, reference, candidate, maxima)
            compare_state(torch, candidate, reloaded, reload_maxima, exact=True)
            require(all(torch.equal(a, b) for a, b in zip(saved, inputs)), "Inputs mutated")
        rejected = []
        for name, inputs in rejections(torch, frames[0]).items():
            reasons = []
            for model in models:
                model.reset(ids)
                try:
                    model(*inputs)
                except (ValueError, RuntimeError, torch.jit.Error) as error:
                    reasons.append(reject_reason(error))
                else:
                    raise ValueError("Invalid input accepted: " + name)
            require(len(set(reasons)) == 1, "Rejection reason differs: " + name)
            compare_state(torch, reference, candidate, maxima)
            compare_state(torch, candidate, reloaded, reload_maxima, exact=True)
            rejected.append(dict(case=name, same_reason=True))
        for model in models:
            model.reset(ids)
        compare_state(torch, reference, candidate, maxima)
        compare_state(torch, candidate, reloaded, reload_maxima, exact=True)
    return dict(status="PASS", frames=240, reset_before=[0, 125, 240],
                all_parameters_and_buffers_checked_each_tick=True, input_mutation=False,
                max_errors=maxima, saved_reload_max_errors=reload_maxima,
                rejections=rejected, rejection_count=len(rejected),
                tolerance=dict(float64_atol=1e-9, float64_rtol=0., float32_atol=1e-6,
                               float32_rtol=1e-6, parameters_masks_integers="exact", saved_reload="exact"),
                scope="Finite source-equivalence cases; not proof of all geometric branch boundaries")


def saved_inputs(torch, raw):
    data = strict_json(raw)
    if "observation" in data:
        data = data["observation"]["observer_tick"]["inputs"]
    elif "inference" in data:
        data = data["inference"]["inputs"]
    elif "inputs" in data:
        data = data["inputs"]
    require(type(data) is dict, "Expected six-vector input record")
    values = []
    for key, width in zip(INPUT_KEYS, (3, 3, 3, 12, 12, 12)):
        value = data.get(key)
        require(type(value) is list and len(value) == width and
                all(type(x) in (int, float) and math.isfinite(x) for x in value), "Invalid saved " + key)
        values.append(torch.tensor([value], dtype=torch.float32))
    return tuple(values), sha(raw)


def distribution(rows):
    out = {}
    for key in ("wall_ms", "thread_cpu_ms"):
        values = sorted(row[key] for row in rows)
        out[key] = dict(samples=len(values), median=statistics.median(values),
                        p95=values[math.ceil(.95*len(values))-1], maximum=max(values))
    return out


def benchmark(torch, reference, candidate, inputs, *, samples=60):
    require(type(samples) is int and 3 <= samples <= 120, "samples must be 3..120")
    models = dict(current_eager=reference, native_lean=candidate)
    ids = torch.tensor([0], dtype=torch.long)
    maxima = {}
    timings = {name: [] for name in models}
    with torch.inference_mode():
        for model in models.values():
            model.reset(ids)
            for _ in range(10):
                model(*inputs)
            model.reset(ids)
        for repeat in range(samples):
            outputs = {}
            order = list(models.items())
            if repeat % 2:
                order.reverse()
            for name, model in order:
                model.reset(ids)
                wall, cpu = time.perf_counter_ns(), time.thread_time_ns()
                outputs[name] = model(*inputs)
                cpu_end, wall_end = time.thread_time_ns(), time.perf_counter_ns()
                timings[name].append(dict(wall_ms=(wall_end-wall)/1e6, thread_cpu_ms=(cpu_end-cpu)/1e6))
            compare_tensor(torch, outputs["current_eager"], outputs["native_lean"], "target", maxima)
            compare_state(torch, reference, candidate, maxima)
        for model in models.values():
            model.reset(ids)
    return dict(status="PASS", warmups=10, reset_before_every_sample=True,
                alternating_order=True, raw_timings=timings,
                distributions={name: distribution(rows) for name, rows in timings.items()},
                max_errors=maxima, hardware_opened=False, output_allowed=False,
                scope="CPU forward only, saved input; no sensor acquisition, output, or cycle claim")
