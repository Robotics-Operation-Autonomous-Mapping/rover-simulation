#!/usr/bin/env bash
set -euo pipefail
export GZ_PARTITION="${GZ_PARTITION:-roam_rover_wheels}"
LINEAR="${1:-0.4}"; ANGULAR="${2:-0.0}"; DURATION="${3:-3}"
python3 - "$LINEAR" "$ANGULAR" "$DURATION" <<'PYTEST'
import sys,time,subprocess
linear, angular, duration=map(float,sys.argv[1:])
if not (-1 <= linear <= 1 and -1.5 <= angular <= 1.5 and 0 < duration <= 60):
    sys.exit('Expected linear +/-1, angular +/-1.5, duration (0,60]')
def publish(v,w):
    payload=f'linear: {{x: {v}}}, angular: {{z: {w}}}'
    subprocess.run(['gz','topic','-t','/rover/cmd_vel','-m','gz.msgs.Twist','-p',payload],check=True)
try:
    finish=time.monotonic()+duration
    while time.monotonic()<finish:
        publish(linear,angular)
        time.sleep(0.2)
finally:
    for _ in range(3): publish(0,0);time.sleep(0.1)
PYTEST
