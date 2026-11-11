"""Compatibility launcher; USB/WLAN lifetime is managed by the Qt app."""
import subprocess
from pathlib import Path


def main():
    executable = Path(__file__).resolve().parents[1] / 'host/build/run/DisplacementTracker.exe'
    if not executable.exists():
        raise RuntimeError('Build and deploy the Qt host first')
    return subprocess.run([str(executable)], cwd=executable.parent).returncode


if __name__ == '__main__':
    raise SystemExit(main())
