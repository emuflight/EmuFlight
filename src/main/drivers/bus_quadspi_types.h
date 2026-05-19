/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// quadSpiResource_t is an opaque data type which represents a QUADSPI
// peripheral, implemented differently across MCU families.
typedef struct quadSpiResource_s quadSpiResource_t;

// octoSpiResource_t is an opaque data type which represents an OCTOSPI
// peripheral, implemented differently across MCU families.
typedef struct octoSpiResource_s octoSpiResource_t;

// Opaque HAL handle type for QSPI.
typedef struct qspiHalHandle_s qspiHalHandle_t;
