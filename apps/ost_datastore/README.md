# ost_datastore

The shared data store of the OrthoStream applications (ADR-013) as a cFS
library module (`CFE_LIB`). It wraps the C++ library `libs/datastore`, links
it statically and exports only `OST_DS_*` (ADR-011 section 8).

```text
CFE_LIB, ost_datastore, OST_DS_Init, OST_DATASTORE, 0, 0, 0x0, 0;
```

* Interface: [`fsw/public_inc/ost_datastore.h`](fsw/public_inc/ost_datastore.h),
  including the rule for handing references along the data path.
* Sizes: [`config/default_ost_datastore_platform_cfg.h`](config/default_ost_datastore_platform_cfg.h);
  a mission overrides them with `ost_datastore_platform_cfg.h` in its
  `*_defs` directory.
* Requires the installed OrthoStream libraries (`scripts/build_libs.sh`) and
  `ost_common` (EDS type `OST_ObjectRef_t`).
