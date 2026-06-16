# Copyright (c) 2023 Nordic Semiconductor
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause

target_sources(app PRIVATE main.c)
target_sources(app PRIVATE fuel_gauge.c)

# EMS stimulation engine (ported verbatim from the DUSQ project)
target_sources(app PRIVATE ems_hw/ems_engine.c)
target_include_directories(app PRIVATE ems_hw)
