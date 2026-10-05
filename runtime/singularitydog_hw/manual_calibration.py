"""Interactive manual pose observations; the operator moves every joint.

Only joint_snapshot's Type 0/17 reads are used. No actuation, zero writes,
automatic pose restoration, camera, or runtime calibration export exists.
"""
import argparse
import contextlib
import datetime
import fcntl
import hashlib
import io
import json
import math
import os
from pathlib import Path
import signal
import sys
import termios
import time
import uuid

from . import joint_snapshot
from .pose_record import save_record
from .replacement_evidence import identity_allowed, load_id11_replacement


LEGS = {"FR": ("右前脚", (1, 2, 3)), "FL": ("左前脚", (4, 5, 6)),
        "RR": ("右後脚", (7, 8, 9)), "RL": ("左後脚", (10, 11, 12))}
KINDS = ("l", "calf", "thigh", "hip", "return")
SPAN_LIMIT_DEG = math.degrees(.02)
OTHER_AXIS_LIMIT_DEG = 3.0
RETURN_LIMIT_DEG = 3.0


def make_stages(legs=None, joint=None):
    legs = list(LEGS) if legs is None else list(legs)
    if not legs or len(set(legs)) != len(legs) or any(leg not in LEGS for leg in legs):
        raise ValueError("脚は重複しないFR/FL/RR/RLで指定してください")
    if joint not in (None, "calf", "thigh", "hip"):
        raise ValueError("関節はcalf/thigh/hipで指定してください")
    kinds = KINDS if joint is None else ("l", joint, "return")
    stages = []
    for leg in legs:
        label, ids = LEGS[leg]
        outward = -10 if leg in ("FR", "RR") else 10
        targets = {"l": (0, 0, -90), "calf": (0, 0, -80),
                   "thigh": (0, -10, -90), "hip": (outward, 0, -90),
                   "return": (0, 0, -90)}
        for kind in kinds:
            hip, thigh, calf = targets[kind]
            stages.append({"id": f"{leg.lower()}-{kind}", "leg": leg,
                           "label": label, "kind": kind, "ids": list(ids),
                           "target_nominal_deg": {"hip": hip, "thigh": thigh, "calf": calf},
                           "target_measured": False,
                           "moving_id": dict(calf=ids[0], thigh=ids[1], hip=ids[2]).get(kind)})
    return stages


def guidance(stage):
    kind = stage["kind"]
    common = ["顔・進行方向 = 前。左右は犬自身から見た左右です。",
              "L字: 上脚は鉛直下、膝→足先の球中心は顔側へ水平。",
              "付け根の横リンクは左右へ水平。接触・抵抗があれば無理に動かさず s。"]
    instructions = {
        "l": ["基準L字に合わせて、姿勢を保ったまま Enter。"],
        "calf": ["基準L字から、膝の場所と上脚・付け根を保ちます。",
                 "膝から先だけを回して球中心を約2cm下げます（約10度）。"],
        "thigh": ["いったん基準L字に戻してください。",
                  "膝の曲がり90度と付け根を保ち、上脚を顔側へ約10度傾けます。",
                  "膝中心が顔側へ約2cm移動する形。下脚も一緒に回ります。"],
        "hip": ["いったん基準L字に戻してください。",
                "上脚・膝のL字を保ったまま、付け根で脚全体を胴体から外側へ約10度。",
                "正面から見て脚が外へ開く向きです。膝だけを曲げないでください。"],
        "return": ["元の基準L字に戻してください。最初の値との再現性を確認します。"],
    }
    titles = {"l": "基準のL字", "calf": "膝から先を少し下げる", "thigh": "上脚を少し前へ",
              "hip": "付け根で少し外へ", "return": "L字に戻して照合"}
    h, t, c = (stage["target_nominal_deg"][k] for k in ("hip", "thigh", "calf"))
    side_diagram = ["【基準L字の図】",
                    "       胴体の上脚軸 A       前・顔 →",
                    "                   |  上脚 120mm",
                    "                 膝 K────球 F",
                    "                       下脚 120mm"]
    return "\n".join([f"{stage['label']} / {titles[kind]} / ID {stage['ids']}", *common,
                      *side_diagram, "", *instructions[kind],
                      f"モデル上の目安: 付け根 {h:+}° / 上脚 {t:+}° / 膝 {c:+}°",
                      "10度は目視の目安です。測定済み角度や駆動目標にはしません。"])


def boot_id():
    return Path("/proc/sys/kernel/random/boot_id").read_text().strip()


def atomic_json(path, value):
    data = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    tmp = path.with_name(path.name + ".tmp-" + uuid.uuid4().hex)
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(tmp, path)


def analyse(stage, pose, baseline=None, replacement=None):
    """Checks concern observation quality, never deployment authorization."""
    ids = stage["ids"]
    warnings, blockers, deltas = [], [], {}
    for mid in ids:
        motor = pose["motors"][str(mid)]
        if math.degrees(motor["position"]["peak_to_peak_rad"]) > SPAN_LIMIT_DEG:
            blockers.append(f"ID {mid}: 読み取り中に角度がばらつきました")
        if motor["max_abs_current_A"] > .05:
            blockers.append(f"ID {mid}: 電流が観測されました。脱力状態を確認")
        if motor["max_abs_velocity_rad_s"] > .1:
            warnings.append(f"ID {mid}: 速度の読み値を要確認")
    if 11 in ids:
        if identity_allowed(11, [pose["motors"]["11"]["mcu_uid_hex"]], replacement):
            warnings.append("ID 11: 交換確認済みの個体。今回の新規記録から候補を作成します")
        else:
            warnings.append("ID 11: 旧個体に値飛び記録あり。交換の照合がない個体は候補を保留")
    if baseline is not None:
        for mid in range(1, 13):
            pre, post = baseline["motors"][str(mid)], pose["motors"][str(mid)]
            if pre["mcu_uid_hex"] != post["mcu_uid_hex"]:
                raise ValueError(f"ID {mid}: モーター個体が変化しました")
            deltas[str(mid)] = post["median_deg_raw_shaft"] - pre["median_deg_raw_shaft"]
        if stage["kind"] == "return":
            if any(abs(deltas[str(i)]) > RETURN_LIMIT_DEG for i in ids):
                blockers.append("最初のL字から3度以上ずれています。戻り姿勢または値飛びを確認")
        elif stage["moving_id"]:
            selected = stage["moving_id"]
            delta = abs(deltas[str(selected)])
            if not 3 <= delta <= 20:
                blockers.append(f"ID {selected}: 変化 {delta:.2f}度。目安3〜20度の範囲外")
            if any(abs(deltas[str(i)]) > OTHER_AXIS_LIMIT_DEG for i in ids if i != selected):
                blockers.append("同じ脚の別の関節も3度以上動いています。L字へ戻してやり直し")
    elif stage["kind"] != "l":
        blockers.append("この脚の有効なL字基準がありません")
    return {"warnings": warnings, "blockers": blockers, "delta_deg": deltas,
            "status": "RETRY_RECOMMENDED" if blockers else "OBSERVATION_CANDIDATE",
            "approved_for_runtime": False}


def build_candidates(stages, records, replacement=None):
    """Nominal hand-aligned candidates only; never a motor configuration."""
    by_stage = {}
    for record in records:
        if not record["accepted"]:
            continue
        if record["stage"]["kind"] == "l":
            # A new L reference starts a new observation sequence for this leg.
            # Old direction/return deltas were measured against the old raw
            # reference and must not acquire a new zero by dictionary overwrite.
            leg = record["stage"]["leg"]
            by_stage = {key: value for key, value in by_stage.items()
                        if value["stage"]["leg"] != leg}
        by_stage[record["stage_id"]] = record
    result = []
    for stage in stages:
        if not stage["moving_id"]:
            continue
        mid, leg, kind = stage["moving_id"], stage["leg"].lower(), stage["kind"]
        base, moved, returned = (by_stage.get(f"{leg}-{s}") for s in ("l", kind, "return"))
        entry = {"motor_id": mid, "leg": stage["leg"], "joint": kind,
                 "sign_candidate": None, "offset_candidate_rad": None,
                 "approved_for_runtime": False, "zero_verified": False, "sign_verified": False,
                 "measured_model_angles": False, "status": "INSUFFICIENT_OBSERVATIONS"}
        observed_uids = [r["motors"][str(mid)]["mcu_uid_hex"] for r in (base, moved, returned) if r]
        if mid == 11 and replacement is None:
            entry["status"] = "KNOWN_POSITION_JUMP_REQUIRES_REVIEW"
        elif observed_uids and not identity_allowed(mid, observed_uids, replacement):
            entry["status"] = "IDENTITY_NOT_ELIGIBLE_FOR_CANDIDATE"
        elif base and moved and returned and all(r["quality"]["status"] == "OBSERVATION_CANDIDATE"
                                                for r in (base, moved, returned)):
            delta = moved["quality"]["delta_deg"][str(mid)]
            nominal = stage["target_nominal_deg"][kind] - base["stage"]["target_nominal_deg"][kind]
            sign = 1 if delta * nominal > 0 else -1
            raw = base["motors"][str(mid)]["position"]["median_rad"]
            entry.update(sign_candidate=sign,
                         offset_candidate_rad=math.radians(base["stage"]["target_nominal_deg"][kind])-sign*raw,
                         status="MANUAL_NOMINAL_CANDIDATE_REVIEW_REQUIRED",
                         observed_delta_deg=delta, nominal_delta_deg=nominal,
                         evidence_paths={"baseline": base["path"], "direction": moved["path"],
                                         "return": returned["path"]})
        result.append(entry)
    return result


class Session:
    def __init__(self, output, stages, sweeps=5, get_boot=boot_id, replacement=None):
        self.path = Path(output).expanduser().resolve()
        if any((p / ".git").exists() for p in (self.path, *self.path.parents)):
            raise ValueError("写真・個体情報を含む記録はGit管理外へ保存してください")
        self.get_boot = get_boot
        current_boot = get_boot()
        self.path.mkdir(parents=True, mode=0o700, exist_ok=False)
        self.stages, self.sweeps = stages, sweeps
        self.replacement = replacement
        self.baselines = {}
        self.data = {"schema_version": 1, "started_at": datetime.datetime.now().astimezone().isoformat(),
                     "boot_id": current_boot, "status": "IN_PROGRESS", "stages": stages, "records": [],
                     "identities": None,
                     "replacement_evidence": replacement,
                     "motor_output_available": False, "approved_for_runtime": False,
                     "zero_verified": False, "sign_verified": False, "angle_wrapping_applied": False,
                     "motor_power_cycle_continuity_verified": False,
                     "source_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                       for p in (Path(__file__), Path(joint_snapshot.__file__))}}
        self.save()

    def save(self):
        self.data["candidates"] = build_candidates(self.stages, self.data["records"], self.replacement)
        atomic_json(self.path / "session.json", self.data)

    def capture(self, stage):
        if self.get_boot() != self.data["boot_id"]:
            raise RuntimeError("Jetsonが再起動しています。新しいセッションを開始してください")
        n = len(self.data["records"]) + 1
        attempt = self.path / f"{n:03d}-{stage['id']}"
        attempt.mkdir(mode=0o700)
        raw = attempt / "raw"
        record = {"stage_id": stage["id"], "stage": stage, "accepted": False,
                  "status": "CAPTURING", "path": str(attempt)}
        self.data["records"].append(record)
        self.save()
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                code = joint_snapshot.main(["--execute", "--sweeps", str(self.sweeps), "--output", str(raw)])
            if code:
                raise RuntimeError("読み取りが完了しませんでした。raw/summary.jsonを保存して中断します")
            if self.get_boot() != self.data["boot_id"]:
                raise RuntimeError("取得中にboot IDが変化しました")
            baseline = self.baselines.get(stage["leg"])
            reference = baseline["raw"] if baseline and stage["kind"] != "l" else None
            pose = save_record(raw, stage["id"], guidance(stage), attempt / "pose", reference=reference)
            identities = {mid: motor["mcu_uid_hex"] for mid, motor in pose["motors"].items()}
            if self.replacement and (not identity_allowed(11, [identities["11"]], self.replacement)
                                     or self.replacement["retired_uid"] in identities.values()):
                raise RuntimeError("交換記録と現在の個体が一致しません。旧個体の記録とは混在させません")
            if self.data["identities"] is None:
                self.data["identities"] = identities
            elif identities != self.data["identities"]:
                raise RuntimeError("セッション中にモーター個体とIDの対応が変わりました")
            quality = analyse(stage, pose, baseline["pose"] if reference else None, self.replacement)
            record.update(status="RECORDED", motors=pose["motors"], quality=quality,
                          recorded_at=pose["completed_at"], accepted=not quality["blockers"])
            if stage["kind"] == "l" and record["accepted"]:
                self.baselines[stage["leg"]] = {"raw": raw, "pose": pose}
        except BaseException as error:
            record.update(status="FAILED", error=str(error))
            raise
        finally:
            self.save()
        return record

    def finish(self, status):
        self.data.update(status=status, finished_at=datetime.datetime.now().astimezone().isoformat())
        self.save()
        lines = ["# 手動校正の観測結果", "", "手で合わせた概略角度による候補です。実機制御には未適用。",
                 f"今回選択した{len(self.stages)}姿勢・{len(self.data['candidates'])}関節だけの結果です。",
                 "q_model = sign × raw + offset の候補値。角度折り返し補正なし。",
                 "モーター電源の再投入をまたぐ連続性は未検証。", "",
                 "| ID | 脚・関節 | 符号候補 | offset候補 rad | 状態 |", "|---:|---|---:|---:|---|"]
        for c in self.data["candidates"]:
            offset = "—" if c["offset_candidate_rad"] is None else f"{c['offset_candidate_rad']:.6f}"
            lines.append(f"| {c['motor_id']} | {c['leg']} {c['joint']} | {c['sign_candidate']} | {offset} | {c['status']} |")
        lines += ["", "ID11の旧個体は保留。交換確認済みの新個体は新規校正から候補を作成します。実機設定へは自動適用しません。",
                  "写真撮影はこの端末ツールに含みません。",
                  "基準姿勢の精度、モデル関節との対応、可動域、電源再投入時の値、負荷時の挙動は別途確認が必要です。"]
        (self.path / "RESULT.md").write_text("\n".join(lines) + "\n")


def tty_input(prompt):
    # Discard keystrokes typed during a capture: one fresh Enter per pose.
    termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
    return input(prompt).strip().lower()


def run_interactive(session, read=tty_input, write=print):
    started = time.monotonic()
    write("支持台上・全脚を浮かせて手で合わせるモード / 自動駆動なし")
    write("モーターが脱力し手で動くことを確認。別の駆動プログラムを同時に起動しないでください。")
    write("Enter=今の位置を記録して次へ / s=飛ばす / q=終了。読み取り中は約1秒姿勢を保持。")
    selected_ids = [s["moving_id"] for s in session.stages if s["moving_id"] is not None]
    write(f"今回の対象: ID {', '.join(map(str, selected_ids))} / {len(session.stages)}姿勢。"
          "選択した関節だけの観測です。電源再投入時は終了して最初から。")
    status = "INTERRUPTED"
    try:
        for index, stage in enumerate(session.stages, 1):
            write(f"\n{'=' * 56}\n[{index}/{len(session.stages)}] " + guidance(stage))
            while True:
                command = read("合わせたら Enter [s/q]: ")
                if command == "q":
                    return 0
                if command == "s":
                    write("未記録で次へ。後からこの脚だけ再実行できます。")
                    break
                if command:
                    write("Enter / s / q を入力してください。")
                    continue
                write("読み取り中… そのまま保持してください。")
                record = session.capture(stage)
                for mid in stage["ids"]:
                    m = record["motors"][str(mid)]
                    delta = record["quality"]["delta_deg"].get(str(mid))
                    change = "—" if delta is None else f"{delta:+.2f}°"
                    write(f"  ID {mid:2}: 生角 {m['median_deg_raw_shaft']:.2f}° / Lとの差 {change} / "
                          f"ばらつき {math.degrees(m['position']['peak_to_peak_rad']):.2f}°")
                for warning in record["quality"]["warnings"]:
                    write("  注: " + warning)
                if record["accepted"]:
                    write("保存しました。次の位置へ。")
                    break
                for reason in record["quality"]["blockers"]:
                    write("  再確認: " + reason)
                write("記録は保管済みです。姿勢を調整しEnterで再記録、sで飛ばせます。")
        accepted = {r["stage_id"] for r in session.data["records"] if r["accepted"]}
        status = "OBSERVATIONS_COMPLETE" if len(accepted) == len(session.stages) else "OBSERVATIONS_PARTIAL"
        if status == "OBSERVATIONS_COMPLETE":
            write("\n観測の記録が揃いました（校正候補・実機駆動は未承認）")
            write(f"今回選択した{len(session.stages)}姿勢だけの完了です。他の関節の校正状態は変更しません。")
        else:
            write(f"\n校正は未完了: {len(accepted)}/{len(session.stages)}姿勢の有効な観測を記録しました。")
            titles = {"l": "基準のL字", "calf": "膝から先を少し下げる", "thigh": "上脚を少し前へ",
                      "hip": "付け根で少し外へ", "return": "L字に戻して照合"}
            for stage in session.stages:
                if stage["id"] not in accepted:
                    ids = [stage["moving_id"]] if stage["moving_id"] else stage["ids"]
                    write(f"  未完了: {stage['label']} / {titles[stage['kind']]} / ID "
                          + ", ".join(str(mid) for mid in ids))
            write("原点・方向は未確定、実機駆動は未承認です。保存済みの観測を確認してください。")
        return 0
    except (EOFError, KeyboardInterrupt):
        write("\n終了します。ここまでの記録は保存済みです。")
        return 130
    except Exception as error:
        status = "FAILED"
        write("\n中断: " + str(error))
        return 1
    finally:
        session.finish(status)
        write(f"保存先: {session.path}\n経過: {time.monotonic()-started:.0f}秒 / 結果: RESULT.md")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--execute", action="store_true", help="対話式の読み取りを開始（駆動しません）")
    legs = ap.add_mutually_exclusive_group()
    legs.add_argument("--leg", choices=LEGS, help="この脚だけ。既定は5姿勢")
    legs.add_argument("--legs", nargs="+", choices=LEGS, help="指定した脚を順番に記録。例: FL RR")
    ap.add_argument("--joint", choices=("calf", "thigh", "hip"),
                    help="1関節だけ再確認。各脚でL字→選択関節→L字の3姿勢")
    ap.add_argument("--sweeps", type=int, choices=range(3, 11), default=5)
    ap.add_argument("--output", type=Path)
    ap.add_argument("--replacement-evidence", type=Path,
                    help="ID11交換前後の読み取り記録を指定する非公開JSON")
    args = ap.parse_args(argv)
    try:
        stages = make_stages([args.leg] if args.leg else args.legs, args.joint)
    except ValueError as error:
        ap.error(str(error))
    if not args.execute:
        for index, stage in enumerate(stages, 1):
            print(f"\n[{index}/{len(stages)}] " + guidance(stage))
        print("\n案内のみ。実行: --execute / 操作: Enter保存・次へ、s飛ばす、q終了")
        return 0
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        ap.error("実行はJetsonの対話端末で行ってください。パイプ入力は不可")
    evidence_path = args.replacement_evidence
    if evidence_path is None:
        default_evidence = Path.home() / ".config" / "singularitydog" / "id11-replacement.json"
        if default_evidence.exists():
            evidence_path = default_evidence
    try:
        replacement = load_id11_replacement(evidence_path) if evidence_path else None
    except (OSError, ValueError, KeyError, TypeError) as error:
        ap.error("交換記録を確認できません: " + str(error))
    output = args.output or Path.home() / "singularitydog-logs" / (
        "manual-calibration-" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:6])
    # Per-user session lock persists even while the serial port is closed at the prompt.
    lock_dir = Path.home() / ".cache" / "singularitydog"
    lock_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    with (lock_dir / "manual-calibration.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            ap.error("別の手動校正ツールが実行中です")
        session = Session(output, stages, args.sweeps, replacement=replacement)
        previous = signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
        old_umask = os.umask(0o077)
        try:
            return run_interactive(session)
        finally:
            os.umask(old_umask)
            signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    raise SystemExit(main())
