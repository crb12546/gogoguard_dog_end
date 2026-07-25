# Route recorder switch backup

Created at `2026-07-24T22:02:21+08:00` before switching the active Go2 CSV
route recorder to the desktop snapshot
`go2_original_linux_code_20260708`.

The main Go2 project and `/home/unitree/go2_fastlio_ws` were not Git
repositories at backup time. This directory is the rollback source for the
switch.

Files:

- `route_recorder.current.py`: active quality-gated recorder before the switch.
- `route_quality.current.py`: geometry and validation dependency used by it.
- `test_route_quality.current.py`: its pure-Python unit tests.
- `patrol_console_server.current.py`: local console backend before adapting the
  caller to the original recorder.
- `patrol_console_index.current.html`: local console UI before adapting its
  recording descriptions and result handling.
- `setup.current.py`: ROS 2 Python package entry-point definition.
- `route_recorder.target_original_20260708.py`: exact desktop source selected as
  the replacement.
- `SHA256SUMS.txt`: hashes of the captured files.
- `SWITCH_RECORD.md`: Git audit, active-file hashes, caller changes, build, and
  smoke-test evidence for this switch.

The robot-side copy is stored under:

`/home/unitree/go2_fastlio_ws/backups/route_recorder_current_before_original_20260724_220221`

Rollback should restore the current recorder and route-quality module, rebuild
`go2_fastlio_patrol`, and restore the two patrol-console files.
