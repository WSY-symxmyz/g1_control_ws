import argparse
import statistics
import sys

import numpy as np

from .native_optimization_ik import NativeOptimizationIK


def main():
    parser = argparse.ArgumentParser(
        description="Run a robot-free consistency and timing check for G1 arm IK."
    )
    parser.add_argument("--arm-model", choices=("G1ARM5", "G1ARM7"), default="G1ARM5")
    parser.add_argument("--samples", type=int, default=200)
    parser.add_argument("--rate-hz", type=float, default=30.0)
    args = parser.parse_args()

    if args.samples < 1 or args.rate_hz <= 0.0:
        parser.error("samples and rate-hz must be positive")

    ik = NativeOptimizationIK(
        args.arm_model,
        max_output_step_rad=0.0,
        filter_weights=(1.0,),
    )
    q = np.clip(np.zeros(ik.model.nq), ik.lower, ik.upper)
    amplitudes = np.linspace(0.08, 0.20, ik.model.nq)
    phases = np.linspace(0.0, np.pi, ik.model.nq)
    elapsed_ms = []
    position_errors = []
    rotation_errors = []
    accepted = 0

    for index in range(args.samples):
        phase = 2.0 * np.pi * index / max(args.samples, 2)
        q_target = amplitudes * np.sin(phase + phases)
        q_target = np.clip(q_target, ik.lower + 1e-3, ik.upper - 1e-3)
        left_target, right_target = ik.forward_kinematics(q_target)
        q, _ = ik.solve_ik(left_target, right_target, q)
        diag = ik.diagnostic_snapshot()
        elapsed_ms.append(diag["elapsed_ms"])
        position_errors.extend(
            [diag["left_position_error_m"], diag["right_position_error_m"]]
        )
        rotation_errors.extend(
            [diag["left_rotation_error_rad"], diag["right_rotation_error_rad"]]
        )
        accepted += int(diag["accepted"])

    p95_ms = float(np.percentile(elapsed_ms, 95))
    period_ms = 1000.0 / args.rate_hz
    print(f"arm_model: {args.arm_model}")
    print(f"joints: {ik.model.nq}")
    print(f"samples: {args.samples}")
    print(f"accepted: {accepted}/{args.samples}")
    print(f"solve_mean_ms: {statistics.fmean(elapsed_ms):.3f}")
    print(f"solve_p95_ms: {p95_ms:.3f}")
    print(f"solve_max_ms: {max(elapsed_ms):.3f}")
    print(f"max_position_error_m: {max(position_errors):.5f}")
    print(f"max_rotation_error_rad: {max(rotation_errors):.5f}")
    print(f"control_period_ms: {period_ms:.3f}")
    print(f"timing_pass: {p95_ms < period_ms}")

    if accepted != args.samples or p95_ms >= period_ms:
        sys.exit(1)


if __name__ == "__main__":
    main()
