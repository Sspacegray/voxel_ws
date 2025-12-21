# test.agent (robot debug helper)

This folder contains a tmux-based “debug agent” that can:

- Launch the simulator + Nav2 using a selected controller (`pp`, `rpp`, or `dwb`)
- Listen to `nav_msgs/Path` (default `waypoint_path`) and auto-send a `FollowPath` goal
- Evaluate path tracking quality and write a Markdown report
- Keep running after you close the terminal / interrupt the chat (tmux)

## Quick start

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

tools/test_agent/test_agent.sh start --controller pp
tools/test_agent/test_agent.sh attach
```

## Typical workflow with waypoint_editor

1. Start the agent (`start --controller pp|rpp|dwb`)
2. In RViz, use `waypoint_editor` to create a fixed path
3. The tool publishes `nav_msgs/Path` on `waypoint_path`
4. The agent auto-sends a Nav2 `FollowPath` goal and starts evaluating
5. Read the report: `tools/test_agent/test_agent.sh report`

## Controller choices

- `pp`: `pp_controller::Nav2PurePursuitController` (your custom PP controller plugin)
- `rpp`: Nav2 built-in `RegulatedPurePursuitController`
- `dwb`: default DWB in the wpr_simulation2 config

## Where logs/reports go

Each run creates a session directory:

`tools/test_agent/sessions/<timestamp>_<controller>/`

It contains:
- `nav2_params.generated.yaml` (the params file actually used)
- `follow_path_on_topic.log`, `path_tracking_eval.log` (tool logs)
- `report.md` (evaluation result)

## Resume after interruption

The agent uses tmux session name `test_agent` by default:

- Attach: `tools/test_agent/test_agent.sh attach`
- Check: `tools/test_agent/test_agent.sh status`
- Stop: `tools/test_agent/test_agent.sh stop`
- Show last report: `tools/test_agent/test_agent.sh report`

## Notes / limitations

- The evaluator computes cross-track error in `map` frame using TF (`map -> base_link`).
  If your robot uses different frames, pass `--map-frame` / `--base-frame` to `path_tracking_eval.py`
  (or update the script defaults).
- The agent defaults to `wpr_simulation2 robocup_home.launch.py` (includes Gazebo client).
  If you want a headless mode, we can add a `--headless` option to use only gzserver.
