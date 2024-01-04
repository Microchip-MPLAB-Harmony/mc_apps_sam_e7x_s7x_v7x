---
title: Release notes
nav_order: 99
---

![Microchip logo](https://raw.githubusercontent.com/wiki/Microchip-MPLAB-Harmony/Microchip-MPLAB-Harmony.github.io/images/microchip_logo.png)
![Harmony logo small](https://raw.githubusercontent.com/wiki/Microchip-MPLAB-Harmony/Microchip-MPLAB-Harmony.github.io/images/microchip_mplab_harmony_logo_small.png)

# Microchip MPLAB® Harmony 3 Release Notes

## Microchip MPLAB® Harmony 3 Motor Control Application Examples for SAM E7x/S7x/V7x family v3.4.1


### Applications

| Development Board | Number of Applications | 
| --- | --- | 
| [dsPICDEM™ MCLV-2 Low Voltage Development Board](https://www.microchip.com/DevelopmentTools/ProductDetails/DM330021-2) | 3 |
| [dsPICDEM™ MCHV-3 High Voltage Development Board](https://www.microchip.com/developmenttools/ProductDetails/dm330023-3) | 3 |

### Bug fixes
* Updated the package.yml file

### Known Issues

* same as v3.1.0

### Required MPLAB Harmony v3 Modules
* motor_control v3.10.0
* x2c v1.4.0

### Development Tools

* [MPLAB X IDE v6.10](https://www.microchip.com/mplab/mplab-x-ide)
* [MPLAB XC32 C/C++ Compiler v4.30](https://www.microchip.com/mplab/compilers)
* MPLAB X IDE plug-ins:
  * MPLAB Code Configurator (MCC) v5.3.7
  * X2CScope v1.3.3.

## Microchip MPLAB® Harmony 3 Motor Control Application Examples for SAM E7x/S7x/V7x family v3.4.0


### Applications

| Development Board | Number of Applications | 
| --- | --- | 
| [dsPICDEM™ MCLV-2 Low Voltage Development Board](https://www.microchip.com/DevelopmentTools/ProductDetails/DM330021-2) | 3 |
| [dsPICDEM™ MCHV-3 High Voltage Development Board](https://www.microchip.com/developmenttools/ProductDetails/dm330023-3) | 3 |

### New Features
* MISRA-C Compliance for motor control files
* Deprecated static projects for PLL based sensorless FOC
* Deprecated static projects for encoder based sensored FOC

### Known Issues

* same as v3.1.0

### Required MPLAB Harmony v3 Modules
* motor_control v3.10.0
* x2c v1.4.0

### Development Tools

* [MPLAB X IDE v6.10](https://www.microchip.com/mplab/mplab-x-ide)
* [MPLAB XC32 C/C++ Compiler v4.30](https://www.microchip.com/mplab/compilers)
* MPLAB X IDE plug-ins:
  * MPLAB Code Configurator (MCC) v5.3.7
  * X2CScope v1.3.3.


## Microchip MPLAB® Harmony 3 Motor Control Application Examples for SAM E7x/S7x/V7x family v3.3.0


### Applications

| Development Board | Number of Applications | 
| --- | --- | 
|[dsPICDEM™ MCLV-2 Low Voltage Development Board](https://www.microchip.com/DevelopmentTools/ProductDetails/DM330021-2) | 5 |
| [dsPICDEM™ MCHV-3 High Voltage Development Board](https://www.microchip.com/developmenttools/ProductDetails/dm330023-3) | 5 |

### New Features
* Added PLL based sensorless FOC for MCHV3 in Harmony QSpin
* Added Encoder based sensored FOC for MCHV3 in Harmony QSpin

### Known Issues

* same as v3.1.0

### Required MPLAB Harmony v3 Modules
* bsp v3.14.0
* motor_control v3.9.0
* x2c v1.3.0
* mcc H3 Library v1.1.5

### Development Tools

* [MPLAB X IDE v6.00](https://www.microchip.com/mplab/mplab-x-ide)
* [MPLAB XC32 C/C++ Compiler v4.10](https://www.microchip.com/mplab/compilers)
* MPLAB X IDE plug-ins:
  * MPLAB Code Configurator (MCC) v5.1.17
  * X2CScope v1.3.3.

  ## Hardware
* For MCHV3 board programming and debugging, use [High Voltage Motor Control Isolated Debugger Card](https://www.microchip.com/en-us/development-tool/dv43t08a)

## Microchip MPLAB® Harmony 3 Motor Control Application Examples for SAM E7x/S7x/V7x family v3.2.0


### Applications

| Development Board | Number of Applications | 
| --- | --- | 
|[dsPICDEM™ MCLV-2 Low Voltage Development Board](https://www.microchip.com/DevelopmentTools/ProductDetails/DM330021-2) | 5 |
| [dsPICDEM™ MCHV-3 High Voltage Development Board](https://www.microchip.com/developmenttools/ProductDetails/dm330023-3) | 3 |

### New Features
* Added **Motor Control Plant** generated motor control firmware for PLL based sensorless FOC for MCLV2
* Added **Motor Control Plant** generated motor control firmware for Encoder based sensored FOC for MCLV2
* Migrated MHC generated code to MCC generated code

### Known Issues

* same as v3.1.0

### Required MPLAB Harmony v3 Modules
* csp v3.11.0
* x2c v1.4.0
* bsp v3.11.1
* motor_control v3.8.0
* dev_packs v3.11.0
* mcc v1.1.0

### Development Tools

* [MPLAB X IDE v6.00](https://www.microchip.com/mplab/mplab-x-ide)
* [MPLAB XC32 C/C++ Compiler v4.00](https://www.microchip.com/mplab/compilers)
* MPLAB X IDE plug-ins:
  * MPLAB Code Configurator (MCC) v5.1.2
  * X2CScope v1.3.0.


## Microchip MPLAB® Harmony 3 Motor Control Application Examples for SAM E7x/S7x/V7x family v3.1.0


### Applications

| Development Board | Number of Applications | 
| --- | --- | 
|[dsPICDEM™ MCLV-2 Low Voltage Development Board](https://www.microchip.com/DevelopmentTools/ProductDetails/DM330021-2) | 3 |
| [dsPICDEM™ MCHV-3 High Voltage Development Board](https://www.microchip.com/developmenttools/ProductDetails/dm330023-3) | 3 |

### New Features
None

### Known Issues

* Isolated EDBG Card 
  * The Isolated EDBG Card may appear "grayed out" (disabled) under the list of tools in MPLABX v5.50. In order to resolve this issue, please go to Tools -> Options ->Embedded->Generic Settings and enable "Exclude device checks for kits" by selecting the check box. 
  
  ![Exclude_Device_Checks](apps/docs/images/mchv3/exclude_device_checks_edbg.jpg)

  * If programming failure occurs with message "java.lang.RuntimeException:RDDI_DAP_OPERATION_FAILED", then reset the Isolated EDBG Card's configuration by Go to File -> Project Properties -> EDBG -> Reset 

* pmsm_foc_encoder_sam_e70_mchv3 application running on dsPICDEM MCHV-3 requires increasing bandwidth of the quadrature encoder signal filter to maintain signal integrity of quadrature sensor signals at higher motor speeds. Without these modifications, motor operation may fail at higher speeds.
  * Reduce the capacitance value of C25, C26 and C27 from 100pF to 10pF 50V NPO 0805

### Required MPLAB Harmony v3 Modules
* csp v3.9.1
* x2c v1.1.4
* motor_control v3.7.0
* dev_packs v3.9.0
* mhc v3.8.0

### Development Tools

* [MPLAB X IDE v5.50](https://www.microchip.com/mplab/mplab-x-ide)
* [MPLAB XC32 C/C++ Compiler v3.01](https://www.microchip.com/mplab/compilers)
* MPLAB X IDE plug-ins:
  * MPLAB Harmony Configurator (MHC) v3.6.4
  * X2CScope v1.3.0.


## Microchip MPLAB® Harmony 3 Motor Control Application Examples for SAM E7x/S7x/V7x family v3.0.0

### Applications

Applications migrated from motor_control repository to this application repository for SAME7x/S7x/V7x family. 

| Development Board | Number of Applications | 
| --- | --- | 
|[dsPICDEM™ MCLV-2 Low Voltage Development Board](https://www.microchip.com/DevelopmentTools/ProductDetails/DM330021-2) | 3 |
| [dsPICDEM™ MCHV-3 High Voltage Development Board](https://www.microchip.com/developmenttools/ProductDetails/dm330023-3) | 3 |


### Required MPLAB Harmony v3 Modules
* csp v3.8.3
* x2c v1.1.3
* motor_control v3.6.0
* dev_packs v3.8.0
* mhc v3.6.5

### Known Issues

* Isolated EDBG Card 
  * The Isolated EDBG Card may appear "grayed out" (disabled) under the list of tools in MPLABX v5.45. In order to resolve this issue, please go to Tools -> Options ->Embedded->Generic Settings and enable "Exclude device checks for kits" by selecting the check box. 
  
  ![Exclude_Device_Checks](apps/docs/images/mchv3/exclude_device_checks_edbg.jpg)

  * If programming failure occurs with message "java.lang.RuntimeException:RDDI_DAP_OPERATION_FAILED", then reset the Isolated EDBG Card's configuration by Go to File -> Project Properties -> EDBG -> Reset 

* pmsm_foc_encoder_sam_e70_mchv3 application running on dsPICDEM MCHV-3 requires increasing bandwidth of the quadrature encoder signal filter to maintain signal integrity of quadrature sensor signals at higher motor speeds. Without these modifications, motor operation may fail at higher speeds.
  * Reduce the capacitance value of C25, C26 and C27 from 100pF to 10pF 50V NPO 0805


### Development Tools

* [MPLAB X IDE v5.45](https://www.microchip.com/mplab/mplab-x-ide)
* [MPLAB XC32 C/C++ Compiler v2.50](https://www.microchip.com/mplab/compilers)
* MPLAB X IDE plug-ins:
  * MPLAB Harmony Configurator (MHC) v3.6.2
  * X2CScope v1.3.0.
