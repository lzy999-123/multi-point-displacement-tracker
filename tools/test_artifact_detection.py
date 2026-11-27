"""Run the production C++ detector/monitor regressions.

Build and deploy first from host/: build.cmd release tests/monitor_test.pro
Then: powershell -File deploy.ps1 -Executables monitor_test

This replaces the independent Python/Otsu demonstration, which did not test
production detector code.
"""
from pathlib import Path
import subprocess


def main():
    executable = Path(__file__).resolve().parents[1] / "host/build/run/monitor_test.exe"
    if not executable.is_file():
        raise SystemExit("Build and deploy host/tests/monitor_test.pro first; no result was produced.")
    return subprocess.run([str(executable), "-o", "-,txt"], cwd=executable.parent).returncode


if __name__ == "__main__":
    raise SystemExit(main())
