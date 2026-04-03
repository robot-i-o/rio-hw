import subprocess
import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).parent


def _available_scripts() -> dict[str, Path]:
    return {p.stem: p for p in sorted(SCRIPTS_DIR.glob("*.sh"))}


def _print_usage(scripts: dict[str, Path]) -> None:
    print("Usage: rio_hw.setup <script> [--sudo] [args...]")
    print()
    print("Options:")
    print("  --sudo   Run the setup script with sudo")
    print("  --list   List available setup scripts")
    print("  --help   Show this help message")
    print()
    print("Available setup scripts:")
    for name in scripts:
        print(f"  {name}")


def main() -> None:
    scripts = _available_scripts()
    args = sys.argv[1:]

    if not args or args[0] == "--list":
        print("Available setup scripts:")
        for name in scripts:
            print(f"  {name}")
        return

    if args[0] in ("--help", "-h"):
        _print_usage(scripts)
        return

    name = args[0]
    extra_args = args[1:]

    if name not in scripts:
        print(f"Error: unknown script '{name}'")
        print("Available setup scripts:")
        for n in scripts:
            print(f"  {n}")
        sys.exit(1)

    use_sudo = "--sudo" in extra_args
    if use_sudo:
        extra_args = [a for a in extra_args if a != "--sudo"]

    cmd = ["bash", str(scripts[name]), *extra_args]
    if use_sudo:
        cmd = ["sudo"] + cmd
    returncode = subprocess.run(cmd, check=False).returncode
    if returncode != 0 and not use_sudo:
        print(
            f"\nHint: if this failed due to permissions, try: rio_hw.setup {name} --sudo"
        )
    sys.exit(returncode)


if __name__ == "__main__":
    main()
