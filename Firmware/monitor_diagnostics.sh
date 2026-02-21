#!/bin/bash
# Monitor KH controller for new diagnostics and download with timestamp
DEVICE="http://khcontrollerv3.local"
INTERVAL=900  # 15 minutes
OUTDIR="$(cd "$(dirname "$0")" && pwd)"
LAST_TS_FILE="/tmp/kh_monitor_last_ts"

last_ts=$(cat "$LAST_TS_FILE" 2>/dev/null || echo "0")
echo "Monitoring $DEVICE for new diagnostics (every ${INTERVAL}s)..."
echo "Saving to: $OUTDIR"
echo "Last known timestamp: $last_ts"

while true; do
  json=$(curl -sf --connect-timeout 5 "$DEVICE/api/diagnostics")
  if [ $? -ne 0 ] || [ -z "$json" ]; then
    echo "$(date '+%H:%M:%S') Device unreachable"
    sleep "$INTERVAL"
    continue
  fi

  ts=$(echo "$json" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('measurement',{}).get('timestamp',0))" 2>/dev/null)
  if [ -z "$ts" ] || [ "$ts" = "0" ]; then
    sleep "$INTERVAL"
    continue
  fi

  if [ "$ts" != "$last_ts" ]; then
    datetime=$(date -r "$ts" '+%Y%m%d-%H%M' 2>/dev/null || date -d "@$ts" '+%Y%m%d-%H%M' 2>/dev/null)
    filename="kh_diagnostics-${datetime}.json"
    echo "$json" | python3 -m json.tool > "$OUTDIR/$filename"
    echo "$(date '+%H:%M:%S') NEW: $filename"
    last_ts="$ts"
    echo "$last_ts" > "$LAST_TS_FILE"
  fi

  sleep "$INTERVAL"
done
