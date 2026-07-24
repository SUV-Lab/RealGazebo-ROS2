#!/usr/bin/env python3
"""Publish an IMU fault scenario to a RealGazebo vehicle.

This node MUST stay alive for as long as the fault should remain latched.
DDS keeps a transient_local sample in the publishing process, not in the
network, so `ros2 topic pub --once` latches nothing at all — a vehicle that
spawns later would never see the spec. Run this in a terminal and leave it.

Run it where the Gazebo server's DDS domain is reachable: inside the gazebo
service container in multi-container mode, on the host (or in the sim
container) in monolithic mode. The plugin's ROS node lives with gzserver, not
with PX4.

Examples:
    # inject TABLE II and hold it
    ./inject_imu_fault.py --vehicle x500_0 --scenario scenarios/imu_table2.yaml

    # clear the fault
    ./inject_imu_fault.py --vehicle x500_0 --clear
"""

import argparse
import math
import os
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from realgazebo_msgs.msg import FaultChannel, ImuFaultSpec
from std_msgs.msg import Float32MultiArray

CHANNELS = {"ax": 0, "ay": 1, "az": 2, "p": 3, "q": 4, "r": 5}
CHANNEL_NAMES = ["ax", "ay", "az", "p", "q", "r"]

TYPES = {
    "none": FaultChannel.T_NONE,
    "bias": FaultChannel.T_BIAS,
    "drift": FaultChannel.T_DRIFT,
    "oscillation": FaultChannel.T_OSCILLATION,
}

EXITS = {
    "snap": FaultChannel.EXIT_SNAP,
    "hold_last": FaultChannel.EXIT_HOLD_LAST,
}

FRAMES = {
    "frd": ImuFaultSpec.FRAME_FRD,
    "flu": ImuFaultSpec.FRAME_FLU,
}

TIME_REFS = {
    "on_command": ImuFaultSpec.REF_ON_COMMAND,
    "on_spawn": ImuFaultSpec.REF_ON_SPAWN,
    "sim_time_abs": ImuFaultSpec.REF_SIM_TIME_ABS,
}

# Units per channel, for the human-readable summary only.
UNITS = ["m/s^2", "m/s^2", "m/s^2", "rad/s", "rad/s", "rad/s"]


def _lookup(table, key, what):
    k = str(key).strip().lower()
    if k not in table:
        raise ValueError(
            "unknown {} '{}'; expected one of {}".format(
                what, key, ", ".join(sorted(table))
            )
        )
    return table[k]


TOP_LEVEL_KEYS = {"enable", "frame", "time_ref", "defaults", "channels"}
CHANNEL_KEYS = {
    "channel",
    "type",
    "p0",
    "p1",
    "p2",
    "t_start",
    "t_end",
    "ramp_in",
    "on_exit",
}


def _reject_unknown(keys, allowed, where):
    """A misspelled key would otherwise fall back to a default in silence."""
    unknown = sorted(set(keys) - allowed)
    if unknown:
        raise ValueError(
            "unknown key(s) {} in {}; allowed: {}".format(
                ", ".join(repr(k) for k in unknown), where, ", ".join(sorted(allowed))
            )
        )


def build_spec(scenario, seq):
    """Turn the parsed YAML into an ImuFaultSpec."""
    if not isinstance(scenario, dict):
        raise ValueError("scenario must be a mapping")

    _reject_unknown(scenario.keys(), TOP_LEVEL_KEYS, "the scenario")

    spec = ImuFaultSpec()
    spec.enable = bool(scenario.get("enable", True))
    spec.spec_seq = seq
    spec.frame = _lookup(FRAMES, scenario.get("frame", "frd"), "frame")
    spec.time_ref = _lookup(
        TIME_REFS, scenario.get("time_ref", "on_command"), "time_ref"
    )

    defaults = scenario.get("defaults") or {}
    _reject_unknown(defaults.keys(), CHANNEL_KEYS, "defaults:")

    for index, raw in enumerate(scenario.get("channels") or []):
        _reject_unknown(raw.keys(), CHANNEL_KEYS, "channels[{}]".format(index))

        merged = dict(defaults)
        merged.update(raw)

        ch = FaultChannel()
        ch.channel = _lookup(CHANNELS, merged["channel"], "channel")
        ch.type = _lookup(TYPES, merged.get("type", "none"), "type")
        ch.p0 = float(merged.get("p0", 0.0))
        ch.p1 = float(merged.get("p1", 0.0))
        ch.p2 = float(merged.get("p2", 0.0))
        ch.t_start = float(merged.get("t_start", 0.0))
        ch.t_end = float(merged.get("t_end", 0.0))
        ch.ramp_in = float(merged.get("ramp_in", 0.0))
        ch.on_exit = _lookup(EXITS, merged.get("on_exit", "snap"), "on_exit")

        spec.channels.append(ch)

    return spec


def describe(spec):
    """One line per channel, so a typo in the YAML is visible before takeoff."""
    type_names = {v: k for k, v in TYPES.items()}
    exit_names = {v: k for k, v in EXITS.items()}
    frame_names = {v: k for k, v in FRAMES.items()}
    ref_names = {v: k for k, v in TIME_REFS.items()}

    lines = [
        "frame={} time_ref={} seq={}".format(
            frame_names[spec.frame].upper(), ref_names[spec.time_ref], spec.spec_seq
        )
    ]

    for ch in spec.channels:
        name = CHANNEL_NAMES[ch.channel]
        unit = UNITS[ch.channel]
        kind = type_names.get(ch.type, "?")

        if kind == "bias":
            mag = "{:g} {}".format(ch.p0, unit)
        elif kind == "drift":
            mag = "{:g} {}/s".format(ch.p0, unit)
        elif kind == "oscillation":
            mag = "{:g} {} @ {:g} Hz (omega={:g} rad/s)".format(
                ch.p0, unit, ch.p1, 2.0 * math.pi * ch.p1
            )
            # Every parameter that changes the waveform must be visible here,
            # or the summary cannot catch the typo it exists to catch.
            if ch.p2:
                mag += " phase={:g} rad".format(ch.p2)
        else:
            mag = "-"

        if ch.ramp_in:
            mag += " ramp={:g}s".format(ch.ramp_in)

        window = (
            "{:g}<t".format(ch.t_start)
            if ch.t_end <= ch.t_start
            else "{:g}<t<{:g}".format(ch.t_start, ch.t_end)
        )

        lines.append(
            "  f_{:<2} {:<12} {:<44} {:<14} exit={}".format(
                name, kind, mag, window, exit_names.get(ch.on_exit, "?")
            )
        )

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--vehicle",
        required=True,
        help="model name as it appears in Gazebo, e.g. x500_0",
    )
    parser.add_argument("--scenario", help="path to a scenario YAML")
    parser.add_argument(
        "--clear",
        action="store_true",
        help="publish enable=false (full passthrough) instead of a scenario",
    )
    parser.add_argument(
        "--seq",
        type=int,
        default=None,
        help="spec_seq; the plugin re-latches its fault clock only when this "
        "changes. Defaults to a fresh value per run, so re-running always "
        "restarts the fault clock. Pass an explicit value only to deliberately "
        "update parameters WITHOUT restarting the window.",
    )
    parser.add_argument(
        "--linger",
        type=float,
        default=0.0,
        help="seconds to stay alive after publishing; 0 means until Ctrl-C. "
        "The latched sample dies with this process.",
    )
    args = parser.parse_args()

    if not args.clear and not args.scenario:
        parser.error("either --scenario or --clear is required")

    if args.clear:
        scenario = {"enable": False, "channels": []}
    else:
        path = args.scenario
        if not os.path.isabs(path):
            candidate = os.path.join(os.path.dirname(os.path.abspath(__file__)), path)
            if os.path.exists(candidate):
                path = candidate

        with open(path, "r") as handle:
            scenario = yaml.safe_load(handle)

    # A constant default would make every re-run look like a duplicate delivery
    # to the plugin, which then keeps the PREVIOUS run's fault clock: with
    # hold_last the new scenario would fire at its terminal value instantly.
    seq = args.seq if args.seq is not None else int(time.time()) & 0xFFFFFFFF

    try:
        spec = build_spec(scenario, seq)
    except (AttributeError, KeyError, TypeError, ValueError) as exc:
        print("scenario error: {}".format(exc), file=sys.stderr)
        return 2

    topic = "/{}/imu_fault/spec".format(args.vehicle)
    applied_topic = "/{}/imu_fault/applied".format(args.vehicle)

    rclpy.init()
    node = Node("inject_imu_fault")

    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    publisher = node.create_publisher(ImuFaultSpec, topic, qos)

    state = {"acked": False}

    def on_applied(msg):
        if state["acked"] or not msg.data:
            return
        if any(abs(v) > 1e-9 for v in msg.data):
            state["acked"] = True
            node.get_logger().info(
                "ack: fault is live on the vehicle -> {}".format(
                    ["{:.4g}".format(v) for v in msg.data]
                )
            )

    node.create_subscription(Float32MultiArray, applied_topic, on_applied, 10)

    publisher.publish(spec)

    if args.clear:
        print("cleared fault on {}".format(topic))
    else:
        print("published to {}\n{}".format(topic, describe(spec)))
        print(
            "\nHolding the latched sample. Watch {} for the applied fault.\n"
            "Ctrl-C only drops the latched DDS sample — the plugin keeps "
            "injecting (hold_last channels stay at their terminal value).\n"
            "Run with --clear to actually stop the fault.".format(applied_topic)
        )

    try:
        if args.linger > 0.0:
            end = node.get_clock().now().nanoseconds + int(args.linger * 1e9)
            while rclpy.ok() and node.get_clock().now().nanoseconds < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
