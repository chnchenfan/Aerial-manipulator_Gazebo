#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
悬停统计脚本：
从 PX4 控制台日志中提取并统计：
1) mean(e_xyz)
2) mean(w_d_y)
3) mean(int_y)

支持时间窗口过滤与时间模式：
- auto: 优先尝试行内时间戳；不可用则回退 synthetic
- timestamp: 强制使用行内时间戳
- synthetic: 使用样本索引 * sample_period
"""

import argparse
import csv
import json
import re
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional


FLOAT_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"

# 例：
# INFO  [eso_rate_control] rate: e(w-sp)[0.02 0.03 0.00] w[...] w_d[-0.05 -0.02 0.00]
RATE_RE = re.compile(
    rf"rate:\s*e\(w-sp\)\[(?P<e>[^\]]+)\].*?w_d\[(?P<wd>[^\]]+)\]"
)

# 例：
# INFO  [eso_rate_control] rate term1: iner[...] int[0.005 -0.108 0.000]
TERM1_RE = re.compile(
    rf"rate\s+term1:\s*iner\[[^\]]+\]\s*int\[(?P<int>[^\]]+)\]"
)

# ROS 风格时间戳，提取第二个（仿真时间）：
# [1772627415.155126396, 1.528000000]
ROS_TS_RE = re.compile(
    rf"\[\s*{FLOAT_RE}\s*,\s*(?P<sim>{FLOAT_RE})\s*\]"
)


@dataclass
class RateSample:
    line_no: int
    timestamp: Optional[float]
    e: List[float]
    w_d: List[float]


@dataclass
class Term1Sample:
    line_no: int
    timestamp: Optional[float]
    int_vec: List[float]


def _parse_vec3(raw: str) -> Optional[List[float]]:
    parts = raw.strip().split()
    if len(parts) != 3:
        return None
    try:
        return [float(parts[0]), float(parts[1]), float(parts[2])]
    except ValueError:
        return None


def _extract_line_timestamp(line: str) -> Optional[float]:
    m = ROS_TS_RE.search(line)
    if not m:
        return None
    try:
        return float(m.group("sim"))
    except ValueError:
        return None


def parse_log(path: Path) -> Dict[str, object]:
    rate_samples: List[RateSample] = []
    term1_samples: List[Term1Sample] = []
    warnings: List[str] = []
    dirty_line_count = 0

    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for idx, line in enumerate(f, start=1):
            ts = _extract_line_timestamp(line)

            rm = RATE_RE.search(line)
            if rm:
                e = _parse_vec3(rm.group("e"))
                w_d = _parse_vec3(rm.group("wd"))

                if e is None or w_d is None:
                    dirty_line_count += 1
                else:
                    rate_samples.append(RateSample(line_no=idx, timestamp=ts, e=e, w_d=w_d))
                continue

            tm = TERM1_RE.search(line)
            if tm:
                int_vec = _parse_vec3(tm.group("int"))

                if int_vec is None:
                    dirty_line_count += 1
                else:
                    term1_samples.append(Term1Sample(line_no=idx, timestamp=ts, int_vec=int_vec))

    if dirty_line_count > 0:
        warnings.append(f"检测到 {dirty_line_count} 行目标日志格式不完整，已跳过。")

    return {
        "rate_samples": rate_samples,
        "term1_samples": term1_samples,
        "warnings": warnings,
    }


def _has_full_timestamps_rate(samples: List[RateSample]) -> bool:
    return len(samples) > 0 and all(s.timestamp is not None for s in samples)


def _has_full_timestamps_term1(samples: List[Term1Sample]) -> bool:
    return len(samples) > 0 and all(s.timestamp is not None for s in samples)


def _apply_time_to_rate(
    samples: List[RateSample],
    time_mode: str,
    sample_period: float,
    warnings: List[str],
) -> str:
    if not samples:
        return time_mode

    if time_mode == "timestamp":
        if not _has_full_timestamps_rate(samples):
            raise SystemExit("time-mode=timestamp 但 rate 样本缺少行内时间戳，无法继续。")
        return "timestamp"

    if time_mode == "synthetic":
        for i, s in enumerate(samples):
            s.timestamp = i * sample_period
        return "synthetic"

    # auto
    if _has_full_timestamps_rate(samples):
        return "timestamp"

    for i, s in enumerate(samples):
        s.timestamp = i * sample_period
    warnings.append("rate 样本未提供完整行内时间戳，已自动回退 synthetic 时间轴。")
    return "synthetic"


def _apply_time_to_term1(
    samples: List[Term1Sample],
    time_mode_used: str,
    sample_period: float,
    warnings: List[str],
) -> None:
    if not samples:
        return

    if time_mode_used == "timestamp":
        if not _has_full_timestamps_term1(samples):
            warnings.append("term1 样本缺少行内时间戳，int_y 统计将改用 synthetic 时间轴过滤。")
            for i, s in enumerate(samples):
                s.timestamp = i * sample_period
        return

    for i, s in enumerate(samples):
        s.timestamp = i * sample_period


def _filter_by_window(samples, start_sec: float, end_sec: Optional[float]):
    out = []
    for s in samples:
        t = s.timestamp
        if t is None:
            continue
        if t < start_sec:
            continue
        if end_sec is not None and t > end_sec:
            continue
        out.append(s)
    return out


def _mean_or_none(arr: List[float]) -> Optional[float]:
    return statistics.fmean(arr) if arr else None


def build_result(
    rate_samples: List[RateSample],
    term1_samples: List[Term1Sample],
    time_mode_used: str,
    start_sec: float,
    end_sec: Optional[float],
    warnings: List[str],
) -> Dict[str, object]:
    ex = [s.e[0] for s in rate_samples]
    ey = [s.e[1] for s in rate_samples]
    ez = [s.e[2] for s in rate_samples]
    wd_y = [s.w_d[1] for s in rate_samples]
    int_y = [s.int_vec[1] for s in term1_samples]

    result = {
        "mean_e": [
            _mean_or_none(ex),
            _mean_or_none(ey),
            _mean_or_none(ez),
        ],
        "mean_w_d_y": _mean_or_none(wd_y),
        "mean_int_y": _mean_or_none(int_y),
        "sample_count_rate": len(rate_samples),
        "sample_count_term1": len(term1_samples),
        "time_mode_used": time_mode_used,
        "window": {
            "start_sec": start_sec,
            "end_sec": end_sec,
        },
        "warnings": warnings,
    }
    return result


def dump_csv(path: Path, rate_samples: List[RateSample], term1_samples: List[Term1Sample]) -> None:
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["type", "line_no", "t_sec", "e_x", "e_y", "e_z", "w_d_y", "int_y"])

        for s in rate_samples:
            writer.writerow(
                [
                    "rate",
                    s.line_no,
                    f"{s.timestamp:.6f}" if s.timestamp is not None else "",
                    f"{s.e[0]:.6f}",
                    f"{s.e[1]:.6f}",
                    f"{s.e[2]:.6f}",
                    f"{s.w_d[1]:.6f}",
                    "",
                ]
            )

        for s in term1_samples:
            writer.writerow(
                [
                    "term1",
                    s.line_no,
                    f"{s.timestamp:.6f}" if s.timestamp is not None else "",
                    "",
                    "",
                    "",
                    "",
                    f"{s.int_vec[1]:.6f}",
                ]
            )


def print_summary(result: Dict[str, object]) -> None:
    mean_e = result["mean_e"]
    print("\n=== 悬停统计结果 ===")
    print(f"mean(e_xyz): [{mean_e[0]}, {mean_e[1]}, {mean_e[2]}]")
    print(f"mean(w_d_y): {result['mean_w_d_y']}")
    print(f"mean(int_y): {result['mean_int_y']}")
    print(f"sample_count_rate: {result['sample_count_rate']}")
    print(f"sample_count_term1: {result['sample_count_term1']}")
    print(f"time_mode_used: {result['time_mode_used']}")
    print(f"window: {result['window']}")

    warnings = result.get("warnings", [])
    if warnings:
        print("warnings:")
        for w in warnings:
            print(f"- {w}")


def main() -> int:
    parser = argparse.ArgumentParser(description="PX4 悬停日志统计：mean(e_xyz)、mean(w_d_y)、mean(int_y)")
    parser.add_argument("--log", required=True, help="控制台日志路径")
    parser.add_argument("--start-sec", type=float, default=0.0, help="统计起始时间（秒）")
    parser.add_argument("--end-sec", type=float, default=None, help="统计结束时间（秒）")
    parser.add_argument(
        "--time-mode",
        choices=["auto", "timestamp", "synthetic"],
        default="auto",
        help="时间模式：auto|timestamp|synthetic",
    )
    parser.add_argument(
        "--sample-period",
        type=float,
        default=0.5,
        help="synthetic 模式采样间隔（秒）",
    )
    parser.add_argument("--out", default=None, help="输出 JSON 路径")
    parser.add_argument("--dump-csv", default=None, help="导出样本 CSV 路径")
    args = parser.parse_args()

    if args.end_sec is not None and args.start_sec >= args.end_sec:
        raise SystemExit("参数错误：start-sec 必须小于 end-sec。")

    if args.sample_period <= 0:
        raise SystemExit("参数错误：sample-period 必须 > 0。")

    log_path = Path(args.log).expanduser().resolve()
    if not log_path.exists():
        raise SystemExit(f"日志文件不存在：{log_path}")

    parsed = parse_log(log_path)
    rate_samples: List[RateSample] = parsed["rate_samples"]
    term1_samples: List[Term1Sample] = parsed["term1_samples"]
    warnings: List[str] = list(parsed["warnings"])

    if not rate_samples:
        raise SystemExit("无有效速率样本（未找到 rate: e(w-sp)... w_d... 行）。")

    time_mode_used = _apply_time_to_rate(
        samples=rate_samples,
        time_mode=args.time_mode,
        sample_period=args.sample_period,
        warnings=warnings,
    )
    _apply_time_to_term1(
        samples=term1_samples,
        time_mode_used=time_mode_used,
        sample_period=args.sample_period,
        warnings=warnings,
    )

    rate_samples_filtered = _filter_by_window(rate_samples, args.start_sec, args.end_sec)
    term1_samples_filtered = _filter_by_window(term1_samples, args.start_sec, args.end_sec)

    if not rate_samples_filtered:
        raise SystemExit("过滤后无有效 rate 样本，请检查 start-sec/end-sec。")

    if not term1_samples_filtered:
        warnings.append("过滤后未找到 term1 样本，mean_int_y 将输出 null。")

    result = build_result(
        rate_samples=rate_samples_filtered,
        term1_samples=term1_samples_filtered,
        time_mode_used=time_mode_used,
        start_sec=args.start_sec,
        end_sec=args.end_sec,
        warnings=warnings,
    )

    print_summary(result)

    if args.out:
        out_path = Path(args.out).expanduser().resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"\nJSON 已输出：{out_path}")

    if args.dump_csv:
        csv_path = Path(args.dump_csv).expanduser().resolve()
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        dump_csv(csv_path, rate_samples_filtered, term1_samples_filtered)
        print(f"CSV 已输出：{csv_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
