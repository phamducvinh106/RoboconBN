#!/usr/bin/env python3
"""Unified test runner for RoboconBN drawing robot."""

from __future__ import annotations

import argparse
import glob
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parent
JAVA_SRC = ROOT / "TeamCode" / "src" / "main" / "java"
CORE = JAVA_SRC / "org" / "firstinspires" / "ftc" / "teamcode" / "core"
CORE_DRIVE = CORE / "drive"
CORE_ODOMETRY = CORE / "odometry"
CORE_PEN = CORE / "pen"
TEST = JAVA_SRC / "org" / "firstinspires" / "ftc" / "teamcode" / "test"
BUILD = ROOT / "build" / "test-runner"
PASS_RE = re.compile(r"RESULT:\s*(\d+)\s+passed,\s*(\d+)\s+failed", re.I)


def _bootstrap_toolchain() -> None:
    if not _java_tool_ok("java"):
        brew_java = Path("/opt/homebrew/opt/openjdk@17/libexec/openjdk.jdk/Contents/Home")
        if (brew_java / "bin" / "java").is_file():
            os.environ["JAVA_HOME"] = str(brew_java)
            os.environ["PATH"] = f"{brew_java / 'bin'}:{os.environ.get('PATH', '')}"
    if "ANDROID_HOME" not in os.environ:
        android_sdk = Path.home() / "Library" / "Android" / "sdk"
        if android_sdk.is_dir():
            os.environ["ANDROID_HOME"] = str(android_sdk)
    local_props = ROOT / "local.properties"
    if not local_props.is_file() and "ANDROID_HOME" in os.environ:
        local_props.write_text(f"sdk.dir={os.environ['ANDROID_HOME']}\n", encoding="utf-8")


@dataclass(frozen=True)
class Suite:
    name: str
    kind: str  # java_offline | java_ftc | python
    description: str
    main_class: str | None = None
    sources: tuple[str, ...] = ()
    python_module: str | None = None


SUITES: tuple[Suite, ...] = (
    Suite(
        "localizer-math",
        "java_offline",
        "Localizer integration math fixtures",
        main_class="org.firstinspires.ftc.teamcode.test.LocalizerMathTest",
        sources=("test/LocalizerMathTest.java",),
    ),
    Suite(
        "draw-path",
        "java_offline",
        "Draw path JSON parse, mm→cm, pen transition order",
        main_class="org.firstinspires.ftc.teamcode.test.DrawPathTest",
        sources=(
            "pen/DrawPathConfig.java",
            "pen/DrawPathLoader.java",
            "pen/RobotConfig.java",
            "pen/RobotConfigLoader.java",
            "test/DrawPathTest.java",
        ),
    ),
    Suite(
        "localizer-calibration",
        "java_ftc",
        "Localizer calibration contract (needs FTC SDK classpath)",
        main_class="org.firstinspires.ftc.teamcode.test.LocalizerCalibrationTest",
    ),
    Suite(
        "mecanum",
        "java_ftc",
        "Mecanum kinematics + simulated goToPosition (needs FTC SDK)",
        main_class="org.firstinspires.ftc.teamcode.test.MecanumDriveTest",
    ),
)


def _which(name: str) -> str | None:
    return shutil.which(name)


def _java_tool_ok(tool: str) -> bool:
    path = _which(tool)
    if not path:
        return False
    result = _run([path, "-version"])
    combined = (result.stdout + result.stderr).strip()
    return result.returncode == 0 and bool(combined)


def _run(cmd: list[str], *, cwd: Path = ROOT, env: dict | None = None) -> subprocess.CompletedProcess[str]:
    merged = os.environ.copy()
    if env:
        merged.update(env)
    return subprocess.run(
        cmd,
        cwd=str(cwd),
        env=merged,
        text=True,
        capture_output=True,
    )


def _print_header(title: str) -> None:
    print(f"\n{'=' * 72}")
    print(title)
    print(f"{'=' * 72}")


def _resolve_sources(suite: Suite) -> list[Path]:
    paths: list[Path] = []
    for rel in suite.sources:
        if rel.startswith("pen/"):
            paths.append(CORE_PEN / rel.removeprefix("pen/"))
        elif rel.startswith("drive/"):
            paths.append(CORE_DRIVE / rel.removeprefix("drive/"))
        elif rel.startswith("odometry/"):
            paths.append(CORE_ODOMETRY / rel.removeprefix("odometry/"))
        elif rel.startswith("core/"):
            paths.append(CORE / rel.removeprefix("core/"))
        elif rel.startswith("test/"):
            paths.append(TEST / rel.removeprefix("test/"))
        else:
            paths.append(JAVA_SRC / rel)
    return paths


def _compile_offline(suite: Suite, out_dir: Path) -> tuple[bool, str]:
    if not _java_tool_ok("javac"):
        return False, "javac not found — install a JDK to run offline Java suites"
    sources = _resolve_sources(suite)
    missing = [str(path) for path in sources if not path.is_file()]
    if missing:
        return False, "missing sources:\n  " + "\n  ".join(missing)
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [_which("javac") or "javac", "-encoding", "UTF-8", "-d", str(out_dir), *[str(path) for path in sources]]
    result = _run(cmd)
    if result.returncode != 0:
        return False, (result.stdout + result.stderr).strip() or "javac failed"
    return True, ""


def _run_java(main_class: str, classpath: Path) -> tuple[bool, str, str]:
    if not _java_tool_ok("java"):
        return False, "", "java not found — install a JDK"
    result = _run([_which("java") or "java", "-cp", str(classpath), main_class])
    output = (result.stdout + result.stderr).strip()
    if result.returncode != 0:
        return False, output, "process exited non-zero"
    match = PASS_RE.search(output)
    if match and int(match.group(2)) == 0:
        return True, output, ""
    if "PASS" in output and "FAIL " not in output:
        return True, output, ""
    if result.returncode == 0 and output and "FAIL " not in output:
        return True, output, ""
    return False, output, "output did not report success"


def _gradle_compile() -> tuple[bool, str]:
    gradlew = ROOT / "gradlew"
    if not gradlew.is_file():
        return False, "gradlew not found"
    result = _run([str(gradlew), ":TeamCode:compileDebugJavaWithJavac", "-q"])
    if result.returncode != 0:
        return False, (result.stdout + result.stderr).strip() or "gradle compile failed"
    return True, ""


def _ftc_classpath() -> list[Path]:
    entries: list[Path] = []
    patterns = [
        "TeamCode/build/intermediates/javac/**/classes",
        "FtcRobotController/build/intermediates/javac/**/classes",
    ]
    for pattern in patterns:
        entries.extend(Path(path) for path in glob.glob(str(ROOT / pattern), recursive=True))
    lib_dirs = [
        ROOT / "FtcRobotController" / "libs",
        ROOT / "TeamCode" / "libs",
        ROOT / "libs",
    ]
    for lib_dir in lib_dirs:
        if lib_dir.is_dir():
            entries.extend(lib_dir.glob("*.jar"))
    seen: set[str] = set()
    unique: list[Path] = []
    for entry in entries:
        key = str(entry.resolve())
        if key not in seen and entry.exists():
            seen.add(key)
            unique.append(entry)
    return unique


def _run_ftc_suite(suite: Suite, *, gradle_compiled: bool) -> tuple[bool, str]:
    if not gradle_compiled:
        ok, message = _gradle_compile()
        if not ok:
            return False, message
    classpath_entries = _ftc_classpath()
    if not classpath_entries:
        return False, "FTC compile classpath not found — run with --gradle first"
    cp = os.pathsep.join(str(path) for path in classpath_entries)
    if not _java_tool_ok("java"):
        return False, "java not found — install a JDK"
    result = _run([_which("java") or "java", "-cp", cp, suite.main_class or ""])
    output = (result.stdout + result.stderr).strip()
    if result.returncode != 0:
        return False, output or "java process failed"
    match = PASS_RE.search(output)
    if match and int(match.group(2)) == 0:
        return True, output
    if "FAIL " in output:
        return False, output
    if "PASS " in output:
        return True, output
    return True, output


def run_suite(suite: Suite, *, gradle_compiled: bool = False) -> tuple[bool, str]:
    if suite.kind == "python":
        return False, "no python suites configured"
    if suite.kind == "java_ftc":
        ok, output = _run_ftc_suite(suite, gradle_compiled=gradle_compiled)
        return ok, output
    out_dir = BUILD / suite.name
    ok, message = _compile_offline(suite, out_dir)
    if not ok:
        return False, message
    passed, output, error = _run_java(suite.main_class or "", out_dir)
    if not passed:
        return False, output or error
    return True, output


def list_suites() -> None:
    print("Available test suites:\n")
    for suite in SUITES:
        print(f"  {suite.name:<22} [{suite.kind:<12}] {suite.description}")
    print("\nRun examples:")
    print("  python run_tests.py --all")
    print("  python run_tests.py --suite draw-path")
    print("  python run_tests.py --suite localizer-math --suite mecanum")


def main(argv: list[str] | None = None) -> int:
    _bootstrap_toolchain()
    parser = argparse.ArgumentParser(description="RoboconBN unified test runner")
    parser.add_argument("--all", action="store_true", help="run every suite (default)")
    parser.add_argument("--list", action="store_true", help="list suites and exit")
    parser.add_argument("--suite", action="append", default=[], help="run one or more named suites")
    parser.add_argument("--java", action="store_true", help="run all Java suites")
    parser.add_argument("--offline", action="store_true", help="run javac-only suites")
    parser.add_argument("--gradle", action="store_true", help="only compile TeamCode with Gradle")
    parser.add_argument("--verbose", action="store_true", help="print full suite output")
    args = parser.parse_args(argv)

    if args.list:
        list_suites()
        return 0

    if args.gradle:
        _print_header("Gradle compile")
        ok, message = _gradle_compile()
        if ok:
            print("OK: :TeamCode:compileDebugJavaWithJavac")
            return 0
        print(message)
        return 1

    selected: list[Suite]
    if args.suite:
        names = {name.strip() for name in args.suite}
        selected = [suite for suite in SUITES if suite.name in names]
        unknown = names - {suite.name for suite in selected}
        if unknown:
            print(f"Unknown suite(s): {', '.join(sorted(unknown))}", file=sys.stderr)
            print("Use --list to see available suites.", file=sys.stderr)
            return 2
    elif args.java:
        selected = [suite for suite in SUITES if suite.kind.startswith("java")]
    elif args.offline:
        selected = [suite for suite in SUITES if suite.kind == "java_offline"]
    else:
        selected = list(SUITES)

    needs_ftc = any(suite.kind == "java_ftc" for suite in selected)
    gradle_compiled = False
    if needs_ftc:
        _print_header("Gradle compile (FTC SDK suites)")
        ok, message = _gradle_compile()
        if not ok:
            print(f"FAIL gradle compile\n{message}")
            return 1
        gradle_compiled = True
        print("OK")

    passed = 0
    failed = 0
    results: list[tuple[str, bool, str]] = []

    for suite in selected:
        _print_header(f"SUITE: {suite.name}")
        ok, output = run_suite(suite, gradle_compiled=gradle_compiled)
        results.append((suite.name, ok, output))
        if ok:
            passed += 1
            print(f"PASS {suite.name}")
        else:
            failed += 1
            print(f"FAIL {suite.name}")
        if args.verbose or not ok:
            if output:
                print(output)

    _print_header("SUMMARY")
    print(f"Passed: {passed}")
    print(f"Failed: {failed}")
    for name, ok, _ in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
