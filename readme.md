﻿---
title: Microchip MPLAB® Harmony 3 Motor Control Application Examples for SAM E7x/S7x/V7x family
nav_order: 1
has_children: true
has_toc: false
---

# Microchip MPLAB® Harmony 3 Motor Control Application Examples for SAM E7x/S7x/V7x family

MPLAB Harmony 3 is an extension of the MPLAB® ecosystem for creating
embedded firmware solutions for Microchip 32-bit SAM and PIC32 microcontroller
and microprocessor devices.  Refer to the following links for more information.
 - [Microchip 32-bit MCUs for Motor Control Applications](https://www.microchip.com/design-centers/motor-control-and-drive/control-products/32-bit-solutions)
 - [Microchip 32-bit MCUs](https://www.microchip.com/design-centers/32-bit)
 - [Microchip 32-bit MPUs](https://www.microchip.com/design-centers/32-bit-mpus)
 - [Microchip MPLAB X IDE](https://www.microchip.com/mplab/mplab-x-ide)
 - [Microchip MPLAB Harmony](https://www.microchip.com/mplab/mplab-harmony)
 - [Microchip MPLAB Harmony Pages](https://microchip-mplab-harmony.github.io/)

This repository contains the MPLAB® Harmony 3 Motor Control application exmaples for SAME7x/S7x/V7x family. Users can use these examples as a reference for
developing their own motor control applications. Refer to the following links for release
notes and licensing information.

 - [Release Notes](./release_notes.md)
 - [MPLAB Harmony License](mplab_harmony_license.md)

## Contents Summary

| Folder     | Description                                               |
|------------|-----------------------------------------------------------|
| apps       | Demonstration applications for Motor Control              |
| docs       | Contains documentation in html format for offline viewing (to be used only after cloning this repository onto a local machine). Use [github pages](https://microchip-mplab-harmony.github.io/mc_apps_sam_e7x_s7x_v7x/) of this repository for viewing it online. |


## Configurable Motor Control Examples (MCC with Harmony QSpin Motor Control)

The following applications are provided to demonstrate the Harmony QSpin to generate the motor control firmware.

| Name | Description|Control Board|Inverter Board|
|:-----|:-----------|:------------|:-------------|
| [PMSM FOC using PLL Estimator ](apps/mcp_pmsm_foc_pll_estimator_sam_e70/readme.md) | Sensorless Field Oriented Control of PMSM using PLL Estimator | [ATSAME70 Motor Control Plugin Module](https://www.microchip.com/Developmenttools/ProductDetails/MA320203) | [dsPICDEM™ MCLV-2 Support](https://www.microchip.com/DevelopmentTools/ProductDetails/DM330021-2) <br  />[dsPICDEM™ MCHV-3 Support](https://www.microchip.com/developmenttools/ProductDetails/dm330023-3) |
| [PMSM FOC using Quadrature Encoder](apps/mcp_pmsm_foc_encoder_sam_e70/readme.md) | Sensored Field Oriented Control of PMSM using Quadrature Encoder |[ATSAME70 Motor Control Plugin Module](https://www.microchip.com/Developmenttools/ProductDetails/MA320203)| [dsPICDEM™ MCLV-2 Support](https://www.microchip.com/DevelopmentTools/ProductDetails/DM330021-2) <br  />[dsPICDEM™ MCHV-3 Support](https://www.microchip.com/developmenttools/ProductDetails/dm330023-3) |


## Static Motor Control Examples

These applications contain static algorithm code and peripherals are configured using MCC. Configurations can be changed in userparam.h file. 

| Name | Description|Control Board|Inverter Board|
|:-----|:-----------|:------------|:-------------|
| [PFC and PMSM FOC using PLL Estimator](apps/pmsm_pfc_foc_pll_estimator_sam_e70/readme.md) | Power Factor Correction and Sensorless Field Oriented Control of PMSM using PLL Estimator |[ATSAME70 Motor Control Plugin Module](https://www.microchip.com/Developmenttools/ProductDetails/MA320203) |[dsPICDEM™ MCHV-3 Support](https://www.microchip.com/developmenttools/ProductDetails/dm330023-3) |
| [PMSM FOC using Sliding Mode Observer](apps/pmsm_foc_smo_sam_e70/readme.md) | Sensorless Field Oriented Control of PMSM using Sliding Mode Observer |[ATSAME70 Motor Control Plugin Module](https://www.microchip.com/Developmenttools/ProductDetails/MA320203) | [dsPICDEM™ MCLV-2 Support](https://www.microchip.com/DevelopmentTools/ProductDetails/DM330021-2) |
|||||



[![License](https://img.shields.io/badge/license-Harmony%20license-orange.svg)](https://github.com/Microchip-MPLAB-Harmony/mc/blob/master/mplab_harmony_license.md)
[![Latest release](https://img.shields.io/github/release/Microchip-MPLAB-Harmony/mc.svg)](https://github.com/Microchip-MPLAB-Harmony/mc/releases/latest)
[![Latest release date](https://img.shields.io/github/release-date/Microchip-MPLAB-Harmony/mc.svg)](https://github.com/Microchip-MPLAB-Harmony/mc/releases/latest)
[![Commit activity](https://img.shields.io/github/commit-activity/y/Microchip-MPLAB-Harmony/mc.svg)](https://github.com/Microchip-MPLAB-Harmony/mc/graphs/commit-activity)
[![Contributors](https://img.shields.io/github/contributors-anon/Microchip-MPLAB-Harmony/mc.svg)]()
____

[![Follow us on Youtube](https://img.shields.io/badge/Youtube-Follow%20us%20on%20Youtube-red.svg)](https://www.youtube.com/user/MicrochipTechnology)
[![Follow us on LinkedIn](https://img.shields.io/badge/LinkedIn-Follow%20us%20on%20LinkedIn-blue.svg)](https://www.linkedin.com/company/microchip-technology)
[![Follow us on Facebook](https://img.shields.io/badge/Facebook-Follow%20us%20on%20Facebook-blue.svg)](https://www.facebook.com/microchiptechnology/)
[![Follow us on Twitter](https://img.shields.io/twitter/follow/MicrochipTech.svg?style=social)](https://twitter.com/MicrochipTech)

[![](https://img.shields.io/github/stars/Microchip-MPLAB-Harmony/mc.svg?style=social)]()
[![](https://img.shields.io/github/watchers/Microchip-MPLAB-Harmony/mc.svg?style=social)]()
