# PlatformIO pre-build script: injects the current git short commit hash as
# the GATEWAY_FW_VERSION preprocessor macro, so the gateway's HA discovery
# "sw" field always reflects the exact firmware build running (see
# buildGatewayDiscoveryMessages()/buildGatewayCommandDiscoveryMessages() in
# src/payload_parser.cpp) instead of a hand-maintained version string that
# someone has to remember to bump.
#
# Falls back to "dev" if git isn't available or this isn't a git checkout
# (e.g. a source archive with no .git directory) -- payload_parser.cpp also
# defines this fallback itself via #ifndef, so a build only misses the real
# hash if this script doesn't run at all, never a hard build failure.

import subprocess

Import("env")


def get_git_short_hash():
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=5,
        )
        if result.returncode != 0:
            return "dev"
        git_hash = result.stdout.decode("utf-8").strip()
        return git_hash if git_hash else "dev"
    except Exception:
        return "dev"


git_version = get_git_short_hash()

# CPPDEFINES value needs the escaped quotes so the compiler sees a proper C
# string literal (-DGATEWAY_FW_VERSION=\"abc1234\"), not a bareword that
# would fail to compile as `doc["sw"] = GATEWAY_FW_VERSION;`.
env.Append(CPPDEFINES=[("GATEWAY_FW_VERSION", '\\"%s\\"' % git_version)])

print("inject_git_version.py: GATEWAY_FW_VERSION = %s" % git_version)
