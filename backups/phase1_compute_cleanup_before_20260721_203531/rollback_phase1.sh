#!/usr/bin/env bash
set -eu

backup_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
project_dir=$(CDPATH= cd -- "$backup_dir/../.." && pwd)
remote_host=${1:-go2wired}
remote_agent=/home/unitree/go2_fastlio_ws/scripts/go2_saas_agent.py
remote_tmp=/home/unitree/go2_fastlio_ws/scripts/.go2_saas_agent.py.rollback.tmp

echo "WARNING: rollback restores the old always-recording video behavior."

cp "$backup_dir/local_before/tools/patrol_console/server.py" "$project_dir/tools/patrol_console/server.py"
cp "$backup_dir/local_before/tools/patrol_console/static/index.html" "$project_dir/tools/patrol_console/static/index.html"
cp "$backup_dir/local_before/orin_go2_fastlio_ws/scripts/go2_saas_agent.py" "$project_dir/orin_go2_fastlio_ws/scripts/go2_saas_agent.py"

scp "$backup_dir/remote_before/go2_saas_agent.py" "$remote_host:$remote_tmp"
ssh "$remote_host" "python3 -m py_compile '$remote_tmp' && chmod 755 '$remote_tmp' && mv '$remote_tmp' '$remote_agent'; video_pid=\$(systemctl show -p MainPID --value go2-saas-video.service); command_pid=\$(systemctl show -p MainPID --value go2-saas-command.service); [ \"\$video_pid\" -le 1 ] || kill -TERM \"\$video_pid\"; [ \"\$command_pid\" -le 1 ] || kill -TERM \"\$command_pid\""

echo "Code rollback complete. Restart the local patrol console to load restored files."
echo "Deleted video files are not recoverable from this backup."
