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
