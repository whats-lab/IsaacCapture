<!--
SPDX-FileCopyrightText: Copyright (c) 2026 WHATs LAB Corp. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# AirGlove plugin

Injects AirGlove gloves as OpenXR hand tracking. The Spine app streams the glove hands to this host;
`libairglove_client` (from [AirGlove-Client](https://github.com/whats-lab/AirGlove-Client), fetched at
configure time) receives them as 26 wrist-relative `XrHandJointEXT` joints, and the plugin places them at a wrist
pose from optical hand tracking or a mounted controller (`plugin_utils::WristPoseSource`).

```bash
cmake -B build -DBUILD_PLUGINS=ON -DBUILD_PLUGIN_AIRGLOVE=ON
cmake --build build --target airglove_plugin
cmake --install build --component airglove

source ~/.cloudxr/run/cloudxr.env
./install/plugins/airglove/airglove_plugin
```

Docs: `docs/source/device/airglove.rst`.
