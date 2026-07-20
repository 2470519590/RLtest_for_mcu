"""Run MuJoCo with STM32F407 as the serial control computer.

This script sends simulated sensor/state values to the MCU as text `STATE` lines,
float32 binary `STATEB` frames, or int16 quantized `STATEQ` frames. It applies
binary `CTRLB` actuator controls back to MuJoCo. Text `OUTI` lines are low-rate
diagnostics only.
"""

from __future__ import annotations

import argparse
import math
import struct
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

CTRL_BINARY_PAYLOAD_SIZE = 18
STATEQ_SCALES = (
    10000.0,  # theta
    1000.0,  # theta_rate
    1000.0,  # x
    1000.0,  # x_rate
    10000.0,  # pitch
    1000.0,  # pitch_rate
    10000.0,  # base_z
    1000.0,  # sim_time_s
    10000.0,  # time_norm
    1.0,  # task forward_jump
    1.0,  # task flight_ramp
    1.0,  # task inplace_jump
    1.0,  # speed zero
    1.0,  # speed low
    1.0,  # speed medium
    1.0,  # speed high
    1000.0,  # yaw_rate
    1000.0,  # yaw_rate_ref
    10000.0,  # roll
    1000.0,  # roll_rate
    10000.0,  # roll_ref
    10000.0,  # left_front_q
    10000.0,  # left_rear_q
    1000.0,  # left_front_dq
    1000.0,  # left_rear_dq
    10000.0,  # right_front_q
    10000.0,  # right_rear_q
    1000.0,  # right_front_dq
    1000.0,  # right_rear_dq
    10.0,  # left_wheel_normal_force
    10.0,  # right_wheel_normal_force
)

from src.robot_smoke.control.lqr import base_pitch_from_qpos
from src.robot_smoke.control.vmc import _drive_joint_position_ctrl
from src.robot_smoke.core.constants import DEFAULT_MODEL, LOCKED_EQUILIBRIUM_QPOS
from src.robot_smoke.core.mujoco_utils import assert_finite, id_by_name, load_mujoco, lock_base_to_qpos
from src.robot_smoke.core.types import SimulatedOdometry
from src.robot_smoke.experiments.equilibrium import _initialize_equilibrium_data
from src.robot_smoke.model.fivebar import equilibrium_analytic_ik_targets
from src.robot_smoke.model.kinematics import update_simulated_odometry
from src.robot_smoke.model.mechanics import _contact_normal_force_for_wheel, _standing_base_qpos_for_virtual_leg_target
from src.robot_smoke.runner import _flight_test_model_xml


@dataclass
class SerialStats:
    tx_state_lines: int = 0
    rx_lines: int = 0
    out_lines: int = 0
    ready_lines: int = 0
    last_rx_text: str = ""
    last_tick: int = 0
    last_seq: int = 0
    last_rl_last_us: float = 0.0
    last_torque_common: float = 0.0
    last_pitch_torque: float = 0.0
    last_left_force: float = 0.0
    last_right_force: float = 0.0
    last_mcu_airborne: float = 0.0
    last_sensor_left_force: float = 0.0
    last_sensor_right_force: float = 0.0
    last_control_overruns: int = 0
    last_ai_errors: int = 0
    last_binary_frames: int = 0
    last_binary_errors: int = 0
    last_text_frames: int = 0
    last_no_input_ticks: int = 0
    last_fivebar_errors: int = 0
    ctrlb_frames: int = 0
    ctrlb_errors: int = 0

def _import_serial():
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise SystemExit("pyserial is required: pip install pyserial") from exc
    return serial


def _joint_qpos_qvel(mujoco, model, data, joint_name: str) -> tuple[float, float]:
    joint_id = id_by_name(mujoco, model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    qpos_id = int(model.jnt_qposadr[joint_id])
    dof_id = int(model.jnt_dofadr[joint_id])
    return float(data.qpos[qpos_id]), float(data.qvel[dof_id])


def _actuator_id(mujoco, model, actuator_name: str) -> int:
    return id_by_name(mujoco, model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)


def _set_initial_pose(mujoco, model, data, args: argparse.Namespace) -> None:
    if args.initial_mode == "equilibrium":
        equilibrium = _initialize_equilibrium_data(
            mujoco,
            model,
            args.initial_length,
            0.0,
            "auto",
            0.35,
            21,
            "upright-ik",
            args.equilibrium_drop_steps,
        )
        data.qpos[:] = equilibrium.qpos
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        mujoco.mj_forward(model, data)
        return

    if args.initial_mode == "locked":
        data.qpos[:] = LOCKED_EQUILIBRIUM_QPOS
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        mujoco.mj_forward(model, data)
        return

    length = args.initial_length
    prewarm_steps = args.prewarm_steps
    target = (length, 0.0)
    left_targets = equilibrium_analytic_ik_targets(mujoco, model, "left", target)
    right_targets = equilibrium_analytic_ik_targets(mujoco, model, "right", target)
    if left_targets is None or right_targets is None:
        raise RuntimeError(f"cannot initialize five-bar pose at L={length}")

    data.qpos[:7] = _standing_base_qpos_for_virtual_leg_target(mujoco, model, target, target)
    for side, targets in (("left", left_targets), ("right", right_targets)):
        for suffix, q in zip(("front", "rear"), targets):
            joint_id = id_by_name(mujoco, model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_{suffix}_drive")
            data.qpos[int(model.jnt_qposadr[joint_id])] = q
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)
    base_qpos = data.qpos[:7].copy()
    for _ in range(prewarm_steps):
        lock_base_to_qpos(mujoco, model, data, base_qpos)
        data.ctrl[:] = 0.0
        _drive_joint_position_ctrl(mujoco, model, data, "left", left_targets[0], left_targets[1], 55.0, 2.0)
        _drive_joint_position_ctrl(mujoco, model, data, "right", right_targets[0], right_targets[1], 55.0, 2.0)
        mujoco.mj_step(model, data)
        assert_finite("serial bridge prewarm qpos", data.qpos)
        assert_finite("serial bridge prewarm qvel", data.qvel)
    lock_base_to_qpos(mujoco, model, data, base_qpos)
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)


def _task_onehot(task: str) -> tuple[float, float, float]:
    order = ("forward_jump", "flight_ramp", "inplace_jump")
    return tuple(1.0 if task == name else 0.0 for name in order)


def _speed_onehot(speed: str) -> tuple[float, float, float, float]:
    order = ("zero", "low", "medium", "high")
    return tuple(1.0 if speed == name else 0.0 for name in order)


def _state_values(
    mujoco,
    model,
    data,
    odometry: SimulatedOdometry,
    args: argparse.Namespace,
    stats: SerialStats,
) -> tuple[float, ...]:
    update_simulated_odometry(mujoco, model, data, odometry)
    pitch = base_pitch_from_qpos(data.qpos)
    pitch_rate = float(data.qvel[4]) if model.nv > 4 else 0.0
    yaw_rate = float(data.qvel[5]) if model.nv > 5 else 0.0
    roll = _base_roll_from_qpos(data.qpos)
    roll_rate = float(data.qvel[3]) if model.nv > 3 else 0.0
    left_force = _contact_normal_force_for_wheel(mujoco, model, data, "left")
    right_force = _contact_normal_force_for_wheel(mujoco, model, data, "right")
    stats.last_sensor_left_force = left_force
    stats.last_sensor_right_force = right_force

    left_front_q, left_front_dq = _joint_qpos_qvel(mujoco, model, data, "left_front_drive")
    left_rear_q, left_rear_dq = _joint_qpos_qvel(mujoco, model, data, "left_rear_drive")
    right_front_q, right_front_dq = _joint_qpos_qvel(mujoco, model, data, "right_front_drive")
    right_rear_q, right_rear_dq = _joint_qpos_qvel(mujoco, model, data, "right_rear_drive")
    task = _task_onehot(args.task)
    speed = _speed_onehot(args.speed)
    values = (
        0.0,
        0.0,
        odometry.position,
        odometry.speed,
        pitch,
        pitch_rate,
        float(data.qpos[2]),
        float(data.time),
        min(float(data.time) / max(args.episode_seconds, 1.0e-6), 1.0),
        *task,
        *speed,
        yaw_rate,
        args.yaw_rate_ref,
        roll,
        roll_rate,
        args.roll_ref,
        left_front_q,
        left_rear_q,
        left_front_dq,
        left_rear_dq,
        right_front_q,
        right_rear_q,
        right_front_dq,
        right_rear_dq,
        left_force,
        right_force,
    )
    return values


def _state_line(mujoco, model, data, odometry: SimulatedOdometry, args: argparse.Namespace, stats: SerialStats) -> str:
    values = _state_values(mujoco, model, data, odometry, args, stats)
    return "STATE " + " ".join(f"{v:.9g}" for v in values) + "\n"


def _state_binary_frame(
    mujoco,
    model,
    data,
    odometry: SimulatedOdometry,
    args: argparse.Namespace,
    stats: SerialStats,
    seq: int,
) -> bytes:
    values = _state_values(mujoco, model, data, odometry, args, stats)
    payload = struct.pack("<H31f", seq & 0xFFFF, *values)
    checksum = 0
    for byte in payload:
        checksum ^= byte
    return b"\xA5\x5A" + payload + bytes((checksum,))


def _quantize_i16(value: float, scale: float) -> int:
    quantized = int(round(float(value) * scale))
    return max(-32768, min(32767, quantized))


def _state_quantized_frame(
    mujoco,
    model,
    data,
    odometry: SimulatedOdometry,
    args: argparse.Namespace,
    stats: SerialStats,
    seq: int,
) -> bytes:
    values = _state_values(mujoco, model, data, odometry, args, stats)
    raw = [_quantize_i16(value, scale) for value, scale in zip(values, STATEQ_SCALES)]
    payload = struct.pack("<H31h", seq & 0xFFFF, *raw)
    checksum = 0
    for byte in payload:
        checksum ^= byte
    return b"\xA6\x6A" + payload + bytes((checksum,))


def _base_roll_from_qpos(qpos: np.ndarray) -> float:
    w, x, y, z = (float(value) for value in qpos[3:7])
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    return math.atan2(sin_roll, cos_roll)


def _parse_out(text: str, stats: SerialStats) -> tuple[float, float, float, float, float, float] | None:
    stats.rx_lines += 1
    stats.last_rx_text = text
    if text.startswith("READY,"):
        stats.ready_lines += 1
        print(f"mcu: {text}")
        return None
    if text.startswith("OUTI,"):
        parts = text.split(",")
        if len(parts) < 19:
            return None
        try:
            stats.out_lines += 1
            stats.last_tick = int(parts[1])
            stats.last_seq = int(parts[2])
            stats.last_rl_last_us = float(parts[4]) / 10.0
            stats.last_torque_common = float(parts[6]) / 1000.0
            stats.last_pitch_torque = float(parts[7]) / 1000.0
            stats.last_left_force = float(parts[8]) / 1000.0
            stats.last_right_force = float(parts[9]) / 1000.0
            stats.last_mcu_airborne = float(parts[10]) / 1000.0
            stats.last_control_overruns = int(parts[17])
            stats.last_ai_errors = int(parts[18])
            if len(parts) >= 24:
                stats.last_binary_frames = int(parts[19])
                stats.last_binary_errors = int(parts[20])
                stats.last_text_frames = int(parts[21])
                stats.last_no_input_ticks = int(parts[22])
                stats.last_fivebar_errors = int(parts[23])
            return tuple(float(parts[i]) / 1000.0 for i in range(11, 17))  # type: ignore[return-value]
        except ValueError:
            return None
    if not text.startswith("OUT,"):
        return None
    parts = text.split(",")
    if len(parts) < 19:
        return None
    try:
        stats.out_lines += 1
        stats.last_tick = int(parts[1])
        stats.last_seq = int(parts[2])
        stats.last_rl_last_us = float(parts[4])
        stats.last_torque_common = float(parts[6])
        stats.last_pitch_torque = float(parts[7])
        stats.last_left_force = float(parts[8])
        stats.last_right_force = float(parts[9])
        stats.last_mcu_airborne = float(parts[10])
        stats.last_control_overruns = int(parts[17])
        stats.last_ai_errors = int(parts[18])
        return tuple(float(parts[i]) for i in range(11, 17))  # type: ignore[return-value]
    except ValueError:
        return None


def _parse_ctrlb_payload(payload: bytes, checksum: int, stats: SerialStats) -> tuple[float, float, float, float, float, float] | None:
    expected = 0
    for byte in payload:
        expected ^= byte
    if expected != checksum:
        stats.ctrlb_errors += 1
        return None
    tick, seq, *raw_ctrls = struct.unpack("<IH6h", payload)
    stats.ctrlb_frames += 1
    stats.last_tick = int(tick)
    stats.last_seq = int(seq)
    return tuple(float(value) / 1000.0 for value in raw_ctrls)  # type: ignore[return-value]


def _consume_serial_rx(
    rx_buffer: bytearray,
    stats: SerialStats,
) -> tuple[float, float, float, float, float, float] | None:
    latest_ctrls: tuple[float, float, float, float, float, float] | None = None
    frame_size = 2 + CTRL_BINARY_PAYLOAD_SIZE + 1
    while rx_buffer:
        binary_index = rx_buffer.find(b"\xC3\x3C")
        newline_index = rx_buffer.find(b"\n")
        if binary_index < 0 and newline_index < 0:
            if len(rx_buffer) > 512:
                del rx_buffer[:-2]
            break
        if binary_index >= 0 and (newline_index < 0 or binary_index < newline_index):
            if binary_index > 0:
                del rx_buffer[:binary_index]
            if len(rx_buffer) < frame_size:
                break
            payload = bytes(rx_buffer[2 : 2 + CTRL_BINARY_PAYLOAD_SIZE])
            checksum = int(rx_buffer[2 + CTRL_BINARY_PAYLOAD_SIZE])
            del rx_buffer[:frame_size]
            parsed = _parse_ctrlb_payload(payload, checksum, stats)
            if parsed is not None:
                latest_ctrls = parsed
            continue
        if newline_index >= 0:
            raw_line = bytes(rx_buffer[:newline_index])
            del rx_buffer[: newline_index + 1]
            text = raw_line.decode("ascii", errors="ignore").strip()
            if not text:
                continue
            parsed = _parse_out(text, stats)
            if parsed is not None:
                latest_ctrls = parsed
            continue
        break
    return latest_ctrls


def _apply_actuator_ctrls(mujoco, model, data, ctrls: tuple[float, float, float, float, float, float]) -> None:
    names = (
        "left_front_motor",
        "left_rear_motor",
        "left_wheel_motor",
        "right_front_motor",
        "right_rear_motor",
        "right_wheel_motor",
    )
    for name, ctrl in zip(names, ctrls):
        actuator_id = _actuator_id(mujoco, model, name)
        low, high = model.actuator_ctrlrange[actuator_id]
        data.ctrl[actuator_id] = float(np.clip(ctrl, low, high))


def run(args: argparse.Namespace) -> None:
    serial_mod = _import_serial()
    mujoco = load_mujoco()
    model_path = Path(args.model)
    if not model_path.is_absolute():
        model_path = Path.cwd() / model_path
    if args.task == "flight_ramp":
        model = mujoco.MjModel.from_xml_string(_flight_test_model_xml(model_path))
        print(f"scene: flight_ramp {args.speed}, full-width ramp injected from {model_path}")
    else:
        model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    _set_initial_pose(mujoco, model, data, args)
    odometry = SimulatedOdometry()
    actuator_ctrls = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    stats = SerialStats()
    rx_buffer = bytearray()

    viewer = None
    if args.visualize:
        import mujoco.viewer  # pylint: disable=import-outside-toplevel

        viewer = mujoco.viewer.launch_passive(model, data)

    realtime = args.realtime or (args.visualize and not args.fast)
    with serial_mod.Serial(args.port, args.baudrate, timeout=0.0, write_timeout=0.01) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(args.boot_wait_seconds)
        ser.write(b"RESET\n")
        time.sleep(args.reset_wait_seconds)
        ser.reset_input_buffer()
        if args.probe_only:
            for index in range(args.probe_count):
                ser.write(b"PING\n")
                deadline = time.perf_counter() + args.probe_timeout
                rx = bytearray()
                while time.perf_counter() < deadline:
                    waiting = int(getattr(ser, "in_waiting", 0))
                    if waiting > 0:
                        rx.extend(ser.read(waiting))
                        if b"\n" in rx:
                            break
                    time.sleep(0.005)
                print(f"probe[{index}] rx={bytes(rx)!r}")
                time.sleep(0.1)
            return
        steps = int(round(args.episode_seconds / float(model.opt.timestep)))
        wall_start = time.perf_counter()
        last_print = time.perf_counter()
        state_period_steps = max(1, int(round(1.0 / (float(model.opt.timestep) * args.state_hz))))
        state_seq = 0
        last_state_tx_step = -state_period_steps
        last_state_tx_ctrlb_frames = -1

        def send_state_frame() -> None:
            nonlocal state_seq, last_state_tx_step, last_state_tx_ctrlb_frames
            if args.protocol == "quantized":
                ser.write(_state_quantized_frame(mujoco, model, data, odometry, args, stats, state_seq))
            elif args.protocol == "binary":
                ser.write(_state_binary_frame(mujoco, model, data, odometry, args, stats, state_seq))
            else:
                ser.write(_state_line(mujoco, model, data, odometry, args, stats).encode("ascii"))
            state_seq = (state_seq + 1) & 0xFFFF
            stats.tx_state_lines += 1
            last_state_tx_step = step
            last_state_tx_ctrlb_frames = stats.ctrlb_frames

        for step in range(steps):
            deadline = time.perf_counter() + args.reply_wait_seconds
            while time.perf_counter() < deadline:
                waiting = int(getattr(ser, "in_waiting", 0))
                if waiting <= 0:
                    continue
                rx_buffer.extend(ser.read(waiting))
                parsed = _consume_serial_rx(rx_buffer, stats)
                if parsed is not None:
                    actuator_ctrls = parsed
            if args.state_pacing == "periodic":
                if step % state_period_steps == 0:
                    send_state_frame()
            elif (
                stats.tx_state_lines == 0
                or (
                    stats.ctrlb_frames != last_state_tx_ctrlb_frames
                    and (step - last_state_tx_step) >= state_period_steps
                )
            ):
                send_state_frame()
            _apply_actuator_ctrls(mujoco, model, data, actuator_ctrls)
            mujoco.mj_step(model, data)
            if viewer is not None:
                viewer.sync()
            if realtime:
                target_wall = wall_start + float(data.time) / max(args.sim_rate, 1.0e-6)
                sleep_s = target_wall - time.perf_counter()
                if sleep_s > 0.0:
                    time.sleep(min(sleep_s, 0.02))
            now = time.perf_counter()
            if now - last_print >= args.print_period:
                last_print = now
                print(
                    f"t={data.time:.3f}s tx={stats.tx_state_lines} rx={stats.rx_lines} "
                    f"out={stats.out_lines} ready={stats.ready_lines} "
                    f"mcu_tick={stats.last_tick} seq={stats.last_seq} rl_us={stats.last_rl_last_us:.1f} ctrl="
                    + ",".join(f"{v:+.3f}" for v in actuator_ctrls)
                    + f" T={stats.last_torque_common:+.3f} Tp={stats.last_pitch_torque:+.3f}"
                    + f" Fl=({stats.last_left_force:+.1f},{stats.last_right_force:+.1f})"
                    + f" overrun={stats.last_control_overruns} aierr={stats.last_ai_errors}"
                    + f" bin=({stats.last_binary_frames}/{stats.last_binary_errors})"
                    + f" ctrlb=({stats.ctrlb_frames}/{stats.ctrlb_errors})"
                    + f" text={stats.last_text_frames} noinput={stats.last_no_input_ticks}"
                    + f" fivebar={stats.last_fivebar_errors}"
                    + f" mcu_air={stats.last_mcu_airborne:.0f}"
                    + f" Fn_sensor=({stats.last_sensor_left_force:.1f},{stats.last_sensor_right_force:.1f})"
                )
                if stats.rx_lines == 0:
                    print("warning: no serial RX lines yet; check COM port and USART1 PA9/PA10 wiring/baudrate.")
                elif stats.out_lines == 0:
                    print(f"warning: RX exists but no OUT parsed yet; last_rx={stats.last_rx_text!r}")
                if args.print_raw:
                    print(f"last_rx={stats.last_rx_text!r}")

    if viewer is not None:
        viewer.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Serial port, for example COM5")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--protocol", choices=["quantized", "binary", "text"], default="quantized")
    parser.add_argument("--state-hz", type=float, default=100.0)
    parser.add_argument("--state-pacing", choices=["ctrlb", "periodic"], default="ctrlb")
    parser.add_argument("--model", default=str(DEFAULT_MODEL))
    parser.add_argument("--episode-seconds", type=float, default=10.0)
    parser.add_argument("--initial-mode", choices=["equilibrium", "locked", "ik"], default="equilibrium")
    parser.add_argument("--initial-length", type=float, default=0.24)
    parser.add_argument("--equilibrium-drop-steps", type=int, default=0)
    parser.add_argument("--prewarm-steps", type=int, default=500)
    parser.add_argument("--task", choices=["forward_jump", "flight_ramp", "inplace_jump"], default="flight_ramp")
    parser.add_argument("--speed", choices=["zero", "low", "medium", "high"], default="medium")
    parser.add_argument("--yaw-rate-ref", type=float, default=0.0)
    parser.add_argument("--roll-ref", type=float, default=0.0)
    parser.add_argument("--reply-wait-seconds", type=float, default=0.0005)
    parser.add_argument("--boot-wait-seconds", type=float, default=0.2)
    parser.add_argument("--reset-wait-seconds", type=float, default=0.05)
    parser.add_argument("--print-period", type=float, default=0.5)
    parser.add_argument("--print-raw", action="store_true")
    parser.add_argument("--probe-only", action="store_true", help="Only send PING and print raw serial replies.")
    parser.add_argument("--probe-count", type=int, default=5)
    parser.add_argument("--probe-timeout", type=float, default=0.5)
    parser.add_argument("--realtime", action="store_true", help="Throttle MuJoCo to wall-clock time.")
    parser.add_argument("--fast", action="store_true", help="Do not throttle viewer mode to wall-clock time.")
    parser.add_argument("--sim-rate", type=float, default=1.0, help="Simulation speed multiplier when throttled.")
    parser.add_argument("--visualize", action="store_true")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
