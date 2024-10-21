#
# Copyright 2024 NXP
#
# SPDX-License-Identifier: Apache-2.0
#

if (CONFIG_SOC_MIMXRT798S_CPU0 OR CONFIG_SOC_MIMXRT798S_CPU1)
  add_subdirectory(cm33)
endif()
