.. _snippet-rtt-tracing:

SystemView RTT Tracing Snippet (rtt-tracing)
############################################

.. code-block:: console

   west build -S rtt-tracing [...]

Overview
********

This snippet enables SEGGER SystemView support with the tracing subsystem and
redirects the serial console output to SEGGER RTT.

Requirements
************

Hardware support for:

- :kconfig:option:`CONFIG_HAS_SEGGER_RTT`
- :kconfig:option:`CONFIG_CONSOLE`
