.. SPDX-FileCopyrightText: Copyright (c) 2026 WHATs LAB Corp. All rights reserved.
.. SPDX-License-Identifier: Apache-2.0

AirGlove
========

The AirGlove plugin injects WHATs LAB AirGlove gloves as OpenXR hand tracking, so ``HandsSource`` and every hand
retargeter consume them like optical hands.

Data flow
---------

The gloves connect to the Spine app, which solves 26 ``XrHandJointEXT`` joints per hand relative to the wrist.
``libairglove_client`` (`AirGlove-Client <https://github.com/whats-lab/AirGlove-Client>`_) receives them on this host.
The plugin places each hand at a wrist pose from ``plugin_utils::WristPoseSource`` — optical hand tracking when the
runtime tracks the hand, otherwise a controller mounted on the glove plus a rigid offset — and pushes it through
``plugin_utils::HandInjector``. A hand with no data for ``--stale-ms`` (default 200) is withdrawn.

Build
-----

.. code-block:: bash

   cmake -B build -DBUILD_PLUGINS=ON -DBUILD_PLUGIN_AIRGLOVE=ON
   cmake --build build --target airglove_plugin
   cmake --install build --component airglove

Configure fetches the pinned AirGlove-Client revision (``AIRGLOVE_CLIENT_TAG``, default ``v0.1.0``) and links its prebuilt
``native/linux-{x64,arm64}/libairglove_client.so``. To build offline, point ``AIRGLOVE_CLIENT_DIR`` at a checkout.

Running
-------

.. code-block:: bash

   python -m isaaccapture.cloudxr.service start
   source ~/.cloudxr/run/cloudxr.env
   ./install/plugins/airglove/airglove_plugin

Start the Spine app with hand output enabled. Options: ``--address``/``--port`` (receive, default
``127.0.0.1:4040``), ``--spine-address``/``--spine-port`` (default ``127.0.0.1:4042``), ``--stale-ms``.

Wrist pose
----------

``AIRGLOVE_WRIST_SOURCE`` selects ``auto`` (default), ``hand_tracking`` or ``controller``.
``AIRGLOVE_AIM_TO_WRIST_LEFT`` / ``AIRGLOVE_AIM_TO_WRIST_RIGHT`` (``px,py,pz,qx,qy,qz,qw``) override the controller
mount offset.
