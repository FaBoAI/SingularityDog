"""Bounded IMU-only pose recording; never opens CAN or applies a calibration."""
import argparse
import datetime
import fcntl
import hashlib
import json
import math
from pathlib import Path
import signal
import statistics
import time

from .imu import ICM20948


def capture_summary(samples):
    if not samples:
        return {"samples":0}
    elapsed=(samples[-1]["monotonic_ns"]-samples[0]["monotonic_ns"])/1e9
    norms=[math.sqrt(sum(v*v for v in s["accel_m_s2"])) for s in samples]
    temps=[s["temperature_c"] for s in samples if s.get("temperature_c") is not None]
    result={"samples":len(samples), "duration_s":elapsed,
            "rate_hz":(len(samples)-1)/elapsed if elapsed else None,
            "accel_mean_m_s2":[statistics.fmean(s["accel_m_s2"][i] for s in samples) for i in range(3)],
            "accel_std_m_s2":[statistics.pstdev(s["accel_m_s2"][i] for s in samples) for i in range(3)],
            "gyro_mean_rad_s":[statistics.fmean(s["gyro_rad_s"][i] for s in samples) for i in range(3)],
            "gyro_std_rad_s":[statistics.pstdev(s["gyro_rad_s"][i] for s in samples) for i in range(3)],
            "accel_norm_mean_m_s2":statistics.fmean(norms),
            "gravity_norm_deviation_percent":100*(statistics.fmean(norms)/9.80665-1),
            "temperature_c":({"min":min(temps), "max":max(temps), "mean":statistics.fmean(temps),
                              "first":temps[0], "last":temps[-1]} if temps else None),
            "sensor_frame":True, "calibration_applied":False,
            "stillness_or_orientation_confirmed":False}
    return result


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--execute",action="store_true")
    ap.add_argument("--seconds",type=int,default=15)
    ap.add_argument("--settle-seconds",type=int,default=10)
    ap.add_argument("--face",choices=("unverified","x+","x-","y+","y-","z+","z-"),default="unverified")
    ap.add_argument("--output",type=Path,required=True)
    args=ap.parse_args(argv)
    if not 10<=args.seconds<=120 or not 1<=args.settle_seconds<=60:
        ap.error("seconds must be10..120; settle-seconds must be1..60")
    plan={"face_label":args.face, "face_label_source":"operator_argument",
          "orientation_verified_by_software":False,
          "settle_seconds":args.settle_seconds, "capture_seconds":args.seconds,
          "bus":"/dev/i2c-7", "address":0x68, "can_opened":False,
          "calibration_applied":False, "poll_hz":100}
    if not args.execute:
        print(json.dumps(plan,indent=2))
        return 0
    output=args.output.expanduser().resolve()
    if any((p/".git").exists() for p in (output,*output.parents)):
        ap.error("Capture files must be outside Git")
    output.mkdir(mode=0o700,parents=True,exist_ok=False)
    errors=[]
    interrupted=[]
    handlers={}
    device=None
    samples=[]
    metadata={"started_at":datetime.datetime.now().astimezone().isoformat(), "plan":plan,
              "errors":errors, "source_sha256":{
                  p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in (Path(__file__),Path(__file__).with_name("imu.py"))}}
    def stop(signum,_frame):
        interrupted.append(signum)
    def check_interrupt():
        if interrupted:
            raise InterruptedError(f"signal {interrupted[0]}")
    try:
        for sig in (signal.SIGINT,signal.SIGTERM):
            handlers[sig]=signal.signal(sig,stop)
        with open("/tmp/singularitydog-imu-i2c7-68.lock","a+") as lock, (output/"events.jsonl").open("x") as log:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            device=ICM20948()
            try:
                metadata["configuration"]=device.start()
                metadata["original_registers"]=device.original_registers
                metadata["register_audit_before"]=device.diagnostic_registers()
                log.write(json.dumps({"kind":"capture_metadata",**plan})+"\n")
                log.flush()
                settle_end=time.monotonic()+args.settle_seconds
                while time.monotonic()<settle_end:
                    check_interrupt()
                    device.read_sample()
                    time.sleep(.01)
                end=time.monotonic()+args.seconds
                next_poll=time.monotonic()
                last_sample=next_poll
                while time.monotonic()<end:
                    check_interrupt()
                    value=device.read_sample()
                    if value is not None:
                        last_sample=time.monotonic()
                        record={"kind":"imu",**value}
                        samples.append(record)
                        log.write(json.dumps(record,allow_nan=False)+"\n")
                        log.flush()
                    elif time.monotonic()-last_sample>.5:
                        raise RuntimeError("No fresh IMU sample for0.5s")
                    next_poll+=.01
                    if next_poll<time.monotonic()-.01:
                        next_poll=time.monotonic()
                    time.sleep(max(0,next_poll-time.monotonic()))
                metadata["register_audit_after"]=device.diagnostic_registers()
                if metadata["register_audit_before"]!=metadata["register_audit_after"]:
                    raise RuntimeError("Offset/trim audit changed during recording")
            finally:
                if device is not None:
                    device.close()
    except BaseException as error:
        errors.append(repr(error))
    finally:
        for sig,handler in handlers.items():
            signal.signal(sig,handler)
        if device is not None:
            metadata["restore_status"]=device.restore_status
    if interrupted and not any("signal " in error for error in errors):
        errors.append(f"signal {interrupted[0]}")
    if len(samples)<2:
        errors.append("Insufficient samples")
    metadata.update(completed_at=datetime.datetime.now().astimezone().isoformat(),
                    status="INCOMPLETE" if errors else "RECORDED_NOT_CALIBRATED",
                    summary=capture_summary(samples))
    (output/"summary.json").write_text(json.dumps(metadata,indent=2,allow_nan=False)+"\n")
    print(json.dumps({"output":str(output),"status":metadata["status"],"summary":metadata["summary"],
                      "restore_status":metadata.get("restore_status"),"errors":errors},indent=2))
    return 1 if errors else 0


if __name__=="__main__":
    raise SystemExit(main())
