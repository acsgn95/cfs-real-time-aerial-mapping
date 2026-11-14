# OrthoStream ground system (COSMOS)

OpenC3 COSMOS 7.4.1, as a separate instance on this machine (ADR-021).

```bash
scripts/cosmos_setup.sh                                   # once: fetch and configure
ground/cosmos/runtime/orthostream-cosmos/openc3.sh run    # start, http://localhost:2910
mission/build.sh && scripts/cosmos_plugin.sh              # plugin from the EDS build
ground/cosmos/runtime/orthostream-cosmos/openc3.sh stop   # stop, keep the data
```

| Port | Use |
| --- | --- |
| 2910 (TCP) | web interface |
| 2244 (UDP) | telemetry from `TO_LAB` |
| 1244 (UDP) | commands to `CI_LAB` on the host |

After cFS starts, enable the telemetry output once with the command
`ORTHOSTREAM TO_LAB_CMD_ENABLE_OUTPUT` and `DEST_IP 127.0.0.1`.

* `compose.override.yaml`: ports and host address of this instance, merged on top of upstream's
  `compose.yaml`.
* `plugin/`: template of the plugin; the packet definitions are generated
  (`tools/ground/generate_cosmos_plugin.py`) and never edited by hand.
* `runtime/`: the upstream COSMOS project, not tracked.

## Recording without COSMOS

`tools/ground/tlm_capture.py` records every packet of a run to JSON lines,
for tests and measurements. It points `TO_LAB` to 127.0.0.2:2244 (so it
does not compete with COSMOS for 127.0.0.1:2244), decodes the packets with
the generated plugin definitions and points `TO_LAB` back to 127.0.0.1 on
exit. COSMOS receives nothing while it runs.

```bash
tools/ground/tlm_capture.py --out run.jsonl --duration 60 --packets ALIGN_APP_IMAGE_POSE ALIGN_APP_HK
tools/ground/compare_image_poses.py run.jsonl data/processed/dorfberg_f01/poses_optimized.txt
```

`compare_image_poses.py` compares the `ImagePose` messages of `ALIGN_APP`
with the poses given to the injector.
