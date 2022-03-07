var myVariable = `
{"0": {
    "doc": "Hardware Setup",
    "title": "Hardware Setup",
    "content": "dsPICDEM™ MCHV-3 Development Board (High Voltage) . | Board | Description | . | MCHV3 Setup for Sensorless Mode | Hardware setup of MCHV3 development board for sensorless mode | . | MCHV3 Setup for Quadrature Encoder Mode | Hardware setup of MCHV3 development board for encoder mode | . |   |   | . dsPICDEM™ MCLV-2 Development Board (Motor Control Low-Voltage) . | Board | Description | . | MCLV2 Setup for Sensorless Mode | Hardware setup of MCLV2 development board for sensorless mode | . | MCLV2 Setup for Quadrature Encoder Mode | Hardware setup of MCLV2 development board for encoder mode | . |   |   | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/docs/hardware_setup.html",
    "relUrl": "/apps/docs/hardware_setup.html"
  },"1": {
    "doc": "MCHV3 Development Board Setup for Quadrature Encoder",
    "title": "MCHV3 Development Board",
    "content": " ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/docs/mchv3_atsame70_pim_encoder.html#mchv3-development-board",
    "relUrl": "/apps/docs/mchv3_atsame70_pim_encoder.html#mchv3-development-board"
  },"2": {
    "doc": "MCHV3 Development Board Setup for Quadrature Encoder",
    "title": "Setting up the hardware",
    "content": "The following table shows the target hardware for the application projects. | Project Name | Hardware | . | mchv3_sam_e70_pim.X | MCHV3 Development BoardATSAME70 Plug-in moduleLeadshine EL5-M0400-1-24 Motor Isolated Embedded Debugger Interface | . |   |   | . Setting up MCHV3 Development Board . | Mount the ATSAME70 Motor Control Plug In Module on U9 header. | Place the “PFC - External Opamp Configuration” Matrix board at J4. | Motor Connections: . | Phase U - M1 | Phase V - M2 | Phase W - M3 | . | Encoder Connections: . | A+ - HA | B+ - HB | 5V - +5V | 0V - GND | . | Jumper Settings: . | J11 - VAC ( Short Pin 3 - 4) | J12 - IA ( Short Pin 1 - 2) | J13 - IB ( Short Pin 1 - 2) | J14 - Fault_IP/IBUS ( Short Pin 1 - 2) | . | Power the board with (110V/220V) AC mains. For additional safety, it is recommended to use a current limited power supply while testing this software demonstration on a non-default hardware and motor. | Installing Isolated Embedded Debugger Default programmer or debugger daughter card shipped with the MCHV3 board cannot program or debug SAM series MCU and therefore, it needs to be replaced with an Isolated Embedded Debugger Interface for MCHV. | Complete Setup . | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/docs/mchv3_atsame70_pim_encoder.html#setting-up-the-hardware",
    "relUrl": "/apps/docs/mchv3_atsame70_pim_encoder.html#setting-up-the-hardware"
  },"3": {
    "doc": "MCHV3 Development Board Setup for Quadrature Encoder",
    "title": "Running the Application",
    "content": ". | Build and Program the application using its IDE | Press switch PUSHBUTTON to start the motor | Vary potentiometer to change the speed of the motor | Press switch to stop the motor | Monitor graphs on X2C Scope | . Refer to the following tables for switch and LED details: . | Switch | Description | . | PUSHBUTTON | To start or stop the motor | . |   |   | . | LED D2 Status | Description | . | OFF | No fault | . | ON | Fault is detected | . |   |   | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/docs/mchv3_atsame70_pim_encoder.html#running-the-application",
    "relUrl": "/apps/docs/mchv3_atsame70_pim_encoder.html#running-the-application"
  },"4": {
    "doc": "MCHV3 Development Board Setup for Quadrature Encoder",
    "title": "MCHV3 Development Board Setup for Quadrature Encoder",
    "content": " ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/docs/mchv3_atsame70_pim_encoder.html",
    "relUrl": "/apps/docs/mchv3_atsame70_pim_encoder.html"
  },"5": {
    "doc": "MCHV3 Development Board Setup for Sensorless Mode",
    "title": "MCHV3 Development Board",
    "content": " ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/docs/mchv3_atsame70_pim_sensorless.html#mchv3-development-board",
    "relUrl": "/apps/docs/mchv3_atsame70_pim_sensorless.html#mchv3-development-board"
  },"6": {
    "doc": "MCHV3 Development Board Setup for Sensorless Mode",
    "title": "Setting up the hardware",
    "content": "The following table shows the target hardware for the application projects. | Project Name | Hardware | . | mchv3_sam_e70_pim.X | MCHV3 Development BoardATSAME70 Plug-in moduleLeadshine EL5-M0400-1-24 Motor Isolated Embedded Debugger Interface | . |   |   | . Setting up MCHV3 Development Board . | Mount the ATSAME70 Motor Control Plug In Module on U9 header. | Place the “PFC - External Opamp Configuration” Matrix board at J4. | Motor Connections: . | Phase U - M1 | Phase V - M2 | Phase W - M3 | . | Jumper Settings: . | J11 - VAC ( Short Pin 3 - 4) | J12 - IA ( Short Pin 1 - 2) | J13 - IB ( Short Pin 1 - 2) | J14 - Fault_IP/IBUS ( Short Pin 1 - 2) | . | Power the board with (110V/220V) AC mains. For additional safety, it is recommended to use a current limited power supply while testing this software demonstration on a non-default hardware and motor. | Installing Isolated Embedded Debugger Default programmer or debugger daughter card shipped with the MCHV3 board cannot program or debug SAM series MCU and therefore, it needs to be replaced with an Isolated Embedded Debugger Interface for MCHV. | Complete Setup . | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/docs/mchv3_atsame70_pim_sensorless.html#setting-up-the-hardware",
    "relUrl": "/apps/docs/mchv3_atsame70_pim_sensorless.html#setting-up-the-hardware"
  },"7": {
    "doc": "MCHV3 Development Board Setup for Sensorless Mode",
    "title": "Running the Application",
    "content": ". | Build and Program the application using its IDE | Press switch PUSHBUTTON to start the motor | Vary potentiometer to change the speed of the motor | Press switch to stop the motor | Monitor graphs on X2C Scope | . Refer to the following tables for switch and LED details: . | Switch | Description | . | PUSHBUTTON | To start or stop the motor | . |   |   | . | LED D2 Status | Description | . | OFF | No fault | . | ON | Fault is detected | . |   |   | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/docs/mchv3_atsame70_pim_sensorless.html#running-the-application",
    "relUrl": "/apps/docs/mchv3_atsame70_pim_sensorless.html#running-the-application"
  },"8": {
    "doc": "MCHV3 Development Board Setup for Sensorless Mode",
    "title": "MCHV3 Development Board Setup for Sensorless Mode",
    "content": " ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/docs/mchv3_atsame70_pim_sensorless.html",
    "relUrl": "/apps/docs/mchv3_atsame70_pim_sensorless.html"
  },"9": {
    "doc": "MCLV2 Development Board Setup for Quadrature Encoder",
    "title": "MCLV2 Development Board",
    "content": " ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/docs/mclv2_atsame70_pim_encoder.html#mclv2-development-board",
    "relUrl": "/apps/docs/mclv2_atsame70_pim_encoder.html#mclv2-development-board"
  },"10": {
    "doc": "MCLV2 Development Board Setup for Quadrature Encoder",
    "title": "Setting up the hardware",
    "content": "The following table shows the target hardware for the application projects. | Project Name | Hardware | . | mclv2_sam_e70_pim.X | MCLV2 Development BoardATSAME70 Plug-in moduleHurst Motor with encoder | . |   |   | . Setting up MCLV2 Development Board . | Mount the ATSAME70 Motor Control Plug In Module on U9 header. | Place the “External Opamp Configuration” Matrix board at J14. | Motor Connections: . | White (Phase U) - M1 | Black (Phase V) - M2 | Red (Phase W) - M3 | . | Encoder Connections: Connect encoder wires as shown below . | (Red) +5V : +5V | (Black) -5V : GND | (White) A : HA | (Blue) B : HB | . | Jumper Settings: . | JP1 - Curr, JP2 - Curr, JP3 - Curr | In order to use RS232 port for X2CScope Communication JP4 - UART, JP5 - UART | In order to use USB port for X2CScope Communication JP4 - USB, JP5 - USB | . | Power the board with a 24V DC supply using J2 or BP1-BP2. For additional safety, it is recommended to use a current limited power supply while testing this software demonstration on a non-default hardware and motor. | Complete Setup . | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/docs/mclv2_atsame70_pim_encoder.html#setting-up-the-hardware",
    "relUrl": "/apps/docs/mclv2_atsame70_pim_encoder.html#setting-up-the-hardware"
  },"11": {
    "doc": "MCLV2 Development Board Setup for Quadrature Encoder",
    "title": "Running the Application",
    "content": ". | Build and Program the application using its IDE | Press switch S2 to start the motor | Vary potentiometer to change the speed of the motor | Press switch S2 to stop the motor | Press switch S3 to change the direction of the motor | Press switch S2 again to start the motor | Monitor graphs on X2C Scope | . Refer to the following tables for switch and LED details: . | Switch | Description | . | Switch S2 | To start or stop the motor | . | Switch S3 | To change the direction of rotation. Direction toggle command is accepted only when motor is stationary. | . |   |   | . | LED D2 Status | Description | . | OFF | Motor spin direction is “positive” | . | ON | Motor spin direction is “negative” | . |   |   | . | LED D17 Status | Description | . | OFF | No fault | . | ON | Fault is detected | . |   |   | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/docs/mclv2_atsame70_pim_encoder.html#running-the-application",
    "relUrl": "/apps/docs/mclv2_atsame70_pim_encoder.html#running-the-application"
  },"12": {
    "doc": "MCLV2 Development Board Setup for Quadrature Encoder",
    "title": "MCLV2 Development Board Setup for Quadrature Encoder",
    "content": " ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/docs/mclv2_atsame70_pim_encoder.html",
    "relUrl": "/apps/docs/mclv2_atsame70_pim_encoder.html"
  },"13": {
    "doc": "MCLV2 Development Board Setup for Sensorless Mode",
    "title": "MCLV2 Development Board",
    "content": " ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/docs/mclv2_atsame70_pim_sensorless.html#mclv2-development-board",
    "relUrl": "/apps/docs/mclv2_atsame70_pim_sensorless.html#mclv2-development-board"
  },"14": {
    "doc": "MCLV2 Development Board Setup for Sensorless Mode",
    "title": "Setting up the hardware",
    "content": "The following table shows the target hardware for the application projects. | Project Name | Hardware | . | mclv2_sam_e70_pim.X | MCLV2 Development BoardATSAME70 Plug-in moduleHurst Motor with encoder | . |   |   | . Setting up MCLV2 Development Board . | Mount the ATSAME70 Motor Control Plug In Module on U9 header. | Place the “External Opamp Configuration” Matrix board at J14. | Motor Connections: . | White (Phase U) - M1 | Black (Phase V) - M2 | Red (Phase W) - M3 | . | Jumper Settings: . | JP1 - Curr, JP2 - Curr, JP3 - Curr | In order to use RS232 port for X2CScope Communication JP4 - UART, JP5 - UART | In order to use USB port for X2CScope Communication JP4 - USB, JP5 - USB | . | Power the board with a 24V DC supply using J2 or BP1-BP2. For additional safety, it is recommended to use a current limited power supply while testing this software demonstration on a non-default hardware and motor. | Complete Setup . | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/docs/mclv2_atsame70_pim_sensorless.html#setting-up-the-hardware",
    "relUrl": "/apps/docs/mclv2_atsame70_pim_sensorless.html#setting-up-the-hardware"
  },"15": {
    "doc": "MCLV2 Development Board Setup for Sensorless Mode",
    "title": "Running the Application",
    "content": ". | Build and Program the application using its IDE | Press switch S2 to start the motor | Vary potentiometer to change the speed of the motor | Press switch S2 to stop the motor | Press switch S3 to change the direction of the motor | Press switch S2 again to start the motor | Monitor graphs on X2C Scope | . Refer to the following tables for switch and LED details: . | Switch | Description | . | Switch S2 | To start or stop the motor | . | Switch S3 | To change the direction of rotation. Direction toggle command is accepted only when motor is stationary. | . |   |   | . | LED D2 Status | Description | . | OFF | Motor spin direction is “positive” | . | ON | Motor spin direction is “negative” | . |   |   | . | LED D17 Status | Description | . | OFF | No fault | . | ON | Fault is detected | . |   |   | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/docs/mclv2_atsame70_pim_sensorless.html#running-the-application",
    "relUrl": "/apps/docs/mclv2_atsame70_pim_sensorless.html#running-the-application"
  },"16": {
    "doc": "MCLV2 Development Board Setup for Sensorless Mode",
    "title": "MCLV2 Development Board Setup for Sensorless Mode",
    "content": " ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/docs/mclv2_atsame70_pim_sensorless.html",
    "relUrl": "/apps/docs/mclv2_atsame70_pim_sensorless.html"
  },"17": {
    "doc": "License",
    "title": "License",
    "content": "MICROCHIP SOFTWARE IS PROVIDED SOLELY TO ASSIST YOU IN DEVELOPING PRODUCTS AND SYSTEMS THAT USE MICROCHIP PRODUCTS. DOWNLOAD AND/OR USE OF THE SOFTWARE REQUIRES THAT YOU ACCEPT THIS SOFTWARE LICENSE AGREEMENT. IF YOU DO NOT WISH TO ACCEPT THESE TERMS, DO NOT DOWNLOAD OR USE ANY OF THE SOFTWARE. DOWNLOADING OR USING THE SOFTWARE CONTITUTES YOUR ACCEPTANCE OF THIS SOFTWARE LICENSE AGREEMENT. SOFTWARE LICENSE AGREEMENT . This Software License Agreement (\\\"Agreement\\\") is an agreement between you (if licensing as an individual) or the entity you represent (if licensing as a business) (\\\"you\\\" or \\\"Licensee\\\") and Microchip Technology Incorporated, a Delaware corporation, with a place of business at 2355 W. Chandler Blvd., Chandler, AZ 85224-6199, and its affiliates including Microchip Technology Ireland Limited, a company organized under the laws of Ireland, with a principal address at Ground Floor, Block W., East Point Business Park, Dublin, Ireland 3 (collectively, \\\"Microchip\\\") for the Microchip MPLAB Harmony Integrated Software Framework and documentation included in the download or otherwise provided by Microchip to Licensee (collectively, the \\\"Software\\\"). 1. Use. Subject to the terms of this Agreement, Microchip hereby grants Licensee a limited, revocable, non-exclusive, non-transferable, worldwide license to (a) use the Software, and (b) modify the Software provided in source code form, if any (and use and copy modifications of such Software made by Licensee), provided that in each case (with respect to clauses (a) and (b)) Licensee solely uses the Software with Microchip Products, Licensee Products, or other products agreed to by Microchip in writing. Licensee has no right to (i) substitute third party products for Microchip Products, or (ii) except as expressly provided in Section 2 below, sublicense its rights under this Agreement or otherwise disclose or distribute the Software to any third party. Licensee may make a reasonable number of copies of the Software solely as necessary to exercise its license rights in this Section 1. Licensee will not remove or alter any copyright, trademark, or other proprietary notices contained on or in the Software or any copies. “Microchip Products” means those Microchip devices purchased from Microchip or one of its authorized distributors that are identified in the Software, or if not identified in the Software, then such Microchip devices that are consistent with the purpose of the Software, including but not limited to Microchip 32-bit microcontroller and microprocessor devices and the like. “Licensee Products” means products manufactured by or for Licensee that use or incorporate Microchip Products. 2. Subcontractors. If Licensee wishes for its subcontractor to obtain and use the Software in order to provide design, manufacturing, or other services to Licensee: (a) such subcontractor may (i) download and agree to the terms of this Agreement or (ii) contact Microchip directly for a copy of this Agreement and agree to its terms; or (b) Licensee may sublicense the rights described in Section 1 directly to its subcontractor, provided that (i) such subcontractor agrees in writing to the terms of this Agreement � a copy of which will be provided to Microchip upon request, and (ii) Licensee is liable for such subcontractor's acts and omissions. 3. Third Party Software. (a) Third Party Materials. Licensee agrees to comply with third party license terms applicable to Third Party Materials, if any. Microchip will not be held responsible for Licensee's failure to comply with such terms. Microchip has no obligation to provide support or maintenance for Third Party Materials. \\\"Third Party Materials\\\" means the third party software, systems, tools, or specifications (including those of a standards setting organization) referenced in, bundled with, or included in the Software. (b) Open Source Components. Notwithstanding the license grant in Section 1 above, Licensee acknowledges that the Software may include Open Source Components. To the extent required by the licenses covering Open Source Components, the terms of such license apply in lieu of the terms of this Agreement. To the extent the terms of the licenses applicable to Open Source Components prohibit any of the restrictions in this Agreement with respect to such Open Source Components, those restrictions will not apply to the Open Source Component. \\\"Open Source Components\\\" means components of the Software that are subject to the terms of an Open Source License. \\\"Open Source License\\\" means any software license approved as an open source license by the Open Source Initiative or any substantially similar license, including without limitation any license that, as a condition of distribution of the software licensed under such license, requires that the distributor make the software available in source code format. 4. Licensee Obligations. (a) Restrictions. Except as expressly permitted by this Agreement, Licensee agrees that it will not (i) modify or alter the Software or a Microchip Product; (ii) adapt, translate, decompile, reverse engineer, disassemble the Software provided in object code form, any Microchip Product, or any samples or prototypes provided by Microchip, or create derivative works thereof; or (iii) use the Software with any software or other materials that are subject to licenses or restrictions (e.g., Open Source Licenses) that, when combined with the Software, would require Microchip to disclose, license, distribute, or otherwise make all or any part of such Software available to anyone. (b) Indemnity. Licensee will indemnify (and, at Microchip's election, defend) Microchip from and against any and all claims, costs, damages, expenses (including reasonable attorneys' fees), liabilities, and losses, arising out of or related to: (i) Licensee's modification, disclosure, or distribution of the Software or Third Party Materials; (ii) the use, sale, or distribution of Licensee Products; and (iii) an allegation that Licensee Products or Licensee's modification of the Software infringe third party intellectual property rights. (c) Licensee Products. Licensee understands and agrees that Licensee remains responsible for using its independent analysis, evaluation, and judgment in designing Licensee Products and systems and has full and exclusive responsibility to assure the safety of its products and compliance of its products (and of all Microchip Products used in or for such Licensee Products) with applicable laws and requirements. 5. Confidentiality. (a) Licensee agrees that the Software, underlying inventions, algorithms, know-how, and ideas relating to the Software, and any other non-public business or technical information disclosed by Microchip to Licensee are confidential and proprietary information, including information derived therefrom, belonging to Microchip and its licensors (collectively, \\\"Confidential Information\\\"). Licensee will use Confidential Information only to exercise its rights and perform its obligations under this Agreement and will take all reasonable measures to protect the secrecy of and avoid unauthorized access, disclosure, and use of Confidential Information. Such measures include, but are not limited to, the highest degree of care that it uses to protect its own information of a similar nature, but not less than reasonable care. Licensee will only disclose Confidential Information to its employees, subcontractors, consultants, auditors and representatives (collectively \\\"Representatives\\\") who have a need to know such information and who have use and confidentiality obligations to Licensee at least as restrictive as those set forth in this Agreement. Licensee is responsible for disclosure or misuse of Confidential Information by its Representatives. Use of Confidential Information for personal gain, for the benefit of a third party or to compete with Microchip, whether directly or indirectly, is a breach of this Agreement. Licensee will notify Microchip in writing of any actual or suspected misuse, misappropriation, or unauthorized disclosure of Confidential Information that comes to Licensee's attention. Confidential Information will not include information that: (i) is or becomes publicly available without breach of this Agreement; (ii) is known or becomes known to Licensee from a source other than Microchip without restriction and without breach of this Agreement or violation of Microchip's rights, as demonstrated by credible evidence in existence at the time of disclosure; (iii) is independently developed by Licensee without use of or reference to the Confidential Information, as demonstrated by credible evidence in existence at the time of independent development; or (iv) is disclosed generally to third parties by Microchip without restrictions similar to those contained in this Agreement. Licensee may disclose Confidential Information to the extent required under law, rule, or regulation (including those of any national securities exchange), by subpoena, civil investigative demand, or similar process, or by a court or administrative agency (each a \\\"Requirement\\\"'), provided, that to the extent permitted by applicable law, Licensee will provide prompt notice of such Requirement to Microchip to enable Microchip to seek a protective order or otherwise prevent or restrict such disclosure. (b) Return of Materials. Upon Microchip's request and direction, Licensee will promptly return or destroy the Confidential Information, including any physical information or materials provided to Licensee (together with any copies, excerpts, syntheses, CD ROMS, diskettes, etc.), and, in the case of information derived therefrom, provide written certification that all the Confidential Information has been expunged from any such materials or that all such materials have been destroyed. Further, if Licensee or its affiliates become competitors of Microchip, and Microchip notifies Licensee in writing of its status as a competitor in a given market, then Licensee will promptly engage in the return and certification process described above in this Section 5(b). 6. Ownership and Retention of Rights. All rights, title, and interest (including all intellectual property rights) in and to the Software, including any derivative works of the Software and any incremental modifications to the Software whether made by or for Licensee or Microchip (collectively, \\\"Microchip Property\\\"), are and will remain the sole and exclusive property of Microchip, whether such Microchip Property is separate or combined with any other products. Licensee, on behalf of itself and its affiliates, agrees to, and does hereby, assign to Microchip or its designee all right, title and interest (including all intellectual property rights) in and to derivative works of and any incremental modifications to the Software. Licensee will take (and will cause its affiliates, their subcontractors, and all related individuals to take) all action as may be reasonably necessary, proper or advisable to perfect and secure the ownership, licenses, intellectual property and other rights of or to Microchip as set forth in this Agreement. All rights not expressly granted under this Agreement are reserved to Microchip and its licensors and suppliers, and there are no implied rights. Licensee retains all right, title, and interest in and to any technology independently developed by Licensee not derived, directly or indirectly, from the Microchip Property or any other item of tangible property provided to Licensee by Microchip hereunder. 7. Termination. This Agreement will start once accepted by Licensee and continue unless and until terminated as provided in this Agreement. This Agreement automatically terminates immediately if Licensee violates the restrictions set forth in Sections 1, 2 or 4(a). Microchip may terminate this Agreement immediately upon notice if (a) Licensee or its affiliates become competitors of Microchip, or (b) Licensee breaches any other term of this Agreement and does not cure such breach within 30 days after receipt of written notice of such breach from Microchip. Upon termination of this Agreement, (i) the license grants in Sections 1 and 2(b) terminate, and (ii) Licensee will return to Microchip or destroy (and certify the destruction of) all Microchip Property and Confidential Information in its possession or under its control, and all copies thereof. The following sections survive termination of this Agreement: 3, 4, 5, 6, 7, 8, 9, 10, 11 and 12. 8. Dangerous Applications. The Software is not fault-tolerant and is not designed, manufactured, or intended for use in hazardous environments requiring failsafe performance (\\\"Dangerous Applications\\\"). Dangerous Applications include the operation of nuclear facilities, aircraft navigation, aircraft communication systems, air traffic control, direct life support machines, weapons systems, or any environment or system in which the failure of the Software could lead directly or indirectly to death, personal injury, or severe physical or environmental damage. Microchip specifically disclaims (a) any express or implied warranty of fitness for use of the Software in Dangerous Applications; and (b) any and all liability for loss, damages and claims resulting from the use of the Software in Dangerous Applications. 9. EU Consumers � Applicable Terms. WHERE LICENSEE IS A CONSUMER LOCATED IN EUROPE, THE FOLLOWING PROVISIONS APPLY INSTEAD OF SECTIONS 9 AND 10 BELOW: Microchip and its licensors will not be liable (a) for any loss suffered by Licensee in connection with the Software where such loss was not reasonably foreseeable when the Software was first downloaded by Licensee, even if such loss was the result of negligence or the failure of Microchip and its licensors to comply with this Agreement; or (b) irrespective of the basis of claim, for any loss of revenue, profit or other business or economic loss suffered. Some Software is made available to Licensee free of charge, and Licensee may at any time download further copies without charge to replace the Software initially downloaded and others may require a fee to be downloaded, or to download any further copies. In all circumstances, to the extent liability may lawfully be limited or excluded, the cumulative liability of Microchip and its licensors will not exceed USD$1,000 (or equivalent sum in the currency of the country in which Licensee resides). However, none of the foregoing limits or excludes any liability for death or personal injury arising from negligence, or for fraud, fraudulent misrepresentation or any other cause that by law cannot be excluded and limited. 10. Warranty Disclaimers. EXCEPT FOR CONSUMERS TO WHOM SECTION 8 APPLIES, THE SOFTWARE IS LICENSED ON AN \\\"AS-IS\\\" BASIS. MICROCHIP MAKES NO WARRANTIES OF ANY KIND WITH RESPECT TO THE SOFTWARE, WHETHER EXPRESS, IMPLIED, STAUTORY OR OTHERWISE, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, TITLE OR NON-INFRINGEMENT AND ANY WARRANTIES THAT MAY ARISE FROM USAGE OF TRADE OR COURSE OF DEALING. MICROCHIP AND ITS LICENSORS HAVE NO OBLIGATION TO CORRECT ANY DEFECTS IN THE SOFTWARE. TECHNICAL ASSISTANCE, IF PROVIDED, WILL NOT EXPAND THESE WARRANTIES. IF CUSTOMER IS A CONSUMER, THE ABOVE WILL NOT ACT TO EXCLUDE YOUR STATUTORY RIGHTS. 11. Limited Liability. EXCEPT FOR CONSUMERS TO WHOM SECTION 8 APPLIES, IN NO EVENT WILL MICROCHIP BE LIABLE, WHETHER IN CONTRACT, WARRANTY, REPRESENTATION, TORT, STRICT LIABILITY, INDEMNITY, CONTRIBUTION OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER, HOWEVER CAUSED, OR ANY LOSS OF PRODUCTION, COST OF PROCUREMENT OF SUBSTITUTE PRODUCTS OR SERVICES, ANY LOSS OF PROFITS, LOSS OF BUSINESS, LOSS OF USE OR LOSS OF DATA, OR INTERRUPTION OF BUSINESS ARISING OUT OF THIS AGREEMENT, HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH LOSS, AND NOTWITHSTANDING ANY FAILURE OF ESSENTIAL PURPOSE OF ANY LIMITED REMEDY. MICROCHIP'S TOTAL AGGREGATE LIABILITY UNDER THIS AGREEMENT WILL NOT EXCEED USD$1,000. 12. General. (a) This Agreement will be governed by and construed in accordance with the laws of the State of Arizona and the United States, without regard to conflicts of law provisions. The parties hereby irrevocably consent to the exclusive personal jurisdiction and venue of the state and federal courts in Maricopa County, Arizona for any dispute relating to this Agreement. WHERE LICENSEE IS A CONSUMER LOCATED IN EUROPE, this Agreement is subject to the laws of the country in which the Software is downloaded, and, to the extent so mandated by such laws, subject to the jurisdiction of the courts of that country. The parties expressly disclaim the applicability of the United Nations Convention on Contracts for the International Sale of Goods in connection with this Agreement. (b) Unless the parties have a mutually executed agreement relating to the licensing of this Software by Microchip to Licensee (\\\"Signed Agreement\\\"), this Agreement constitutes the entire agreement between the parties with respect to the Software, and supersedes and replaces prior or contemporaneous written or verbal agreements or communications between the parties regarding the Software, including any purchase orders. If the parties have a Signed Agreement, this Agreement does not supersede or replace that Signed Agreement. This Agreement will not be modified except by a written agreement signed by an authorized representative of Microchip. If any provision of this Agreement is held by a court of competent jurisdiction to be illegal, invalid, or unenforceable, that provision will be limited or eliminated to the minimum extent necessary so that this Agreement will otherwise remain in full force and effect and enforceable. No waiver of any breach of any provision of this Agreement constitutes a waiver of any prior, concurrent, or subsequent breach of the same or any other provisions of this Agreement, and no waiver will be effective unless made in writing and signed by an authorized representative of the waiving party. (c) Licensee agrees to comply with all import and export laws and restrictions and regulations of the Department of Commerce or other United States or foreign agency or authority. (d) This Agreement will bind and inure to the benefit of each party's permitted successors and assigns. Licensee may not assign this Agreement in whole or in part, whether by law or otherwise, without Microchip's prior written consent. Any merger, consolidation, amalgamation, reorganization, transfer of all or substantially all assets or other change in control or majority ownership (\\\"Change of Control\\\") is considered an assignment for the purpose of this Section. Any attempt to assign this Agreement without such consent will be null and void. However, Microchip may assign this Agreement to an affiliate, or to another entity in the event of a Change of Control. (e) Licensee acknowledges its breach of any confidentiality or proprietary rights provision of this Agreement would cause Microchip irreparable damage, for which the award of damages would not be an adequate remedy. Licensee, therefore, agrees if Microchip alleges that Licensee has breached or violated any such provisions then Microchip may seek equitable relief, in addition to all other remedies at law or in equity. (f) Authorized representatives of Microchip shall have the right to reasonably inspect Licensee's premises and to audit Licensee's records and inventory of Licensee Products, whether located on Licensee's premises or elsewhere at any time, announced or unannounced, and in its sole and absolute discretion, in order to ensure Licensee's adherence to the terms of this Agreement. (g) Consistent with 48 C.F.R. �12.212 or 48 C.F.R. �227.7202-1 through 227.7202-4, as applicable, the Software is being licensed to U.S. Government end users (i) only as Commercial Items, and (ii) with only those rights as are granted to all other end users pursuant to the terms and conditions of the applicable Microchip licenses. To the extent the Software (or a portion thereof) qualifies as �technical data' as such term is defined in 48 C.F.R. �252.227-7015(a)(5), then its use, duplication, or disclosure by the U.S. Government is subject to the restrictions set forth in subparagraphs (a) through (e) of the Rights in Technical Data clause at 48 C.F.R. �252.227-7015. Contractor/manufacturer is Microchip Technology Inc., 2355 W. Chandler Blvd., Chandler, AZ 85224-6199. Questions about this Agreement should be sent to: Microchip Technology Inc., 2355 W. Chandler Blvd., Chandler, AZ 85224-6199 USA. ATTN: Marketing. v.3.3.2021 . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/mplab_harmony_license.html",
    "relUrl": "/mplab_harmony_license.html"
  },"18": {
    "doc": "PMSM FOC using Quadrature Encoder",
    "title": "PMSM FOC using Quadrature Encoder",
    "content": "This example application shows how to control the Permanent Magnet Synchronous Motor (PMSM) with Quadrature Encoder based Field Oriented Control (FOC) on a SAME70 Micro-controller. ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_encoder_sam_e70/readme.html#pmsm-foc-using-quadrature-encoder",
    "relUrl": "/apps/pmsm_foc_encoder_sam_e70/readme.html#pmsm-foc-using-quadrature-encoder"
  },"19": {
    "doc": "PMSM FOC using Quadrature Encoder",
    "title": "Description",
    "content": "Permanent Magnet Synchronous Motor (PMSM) is controlled using Field Oriented Control (FOC). Rotor position and speed is determined using quadrature encoder sensor. Motor start/stop operation is controlled by the switch and motor speed can be changed by the on-board potentiometer. Waveforms and variables can be monitored runtime using X2CScope. Key features enabled in this project are: . | Dual shunt current measurement | Speed control loop | Field weakening | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_encoder_sam_e70/readme.html#description",
    "relUrl": "/apps/pmsm_foc_encoder_sam_e70/readme.html#description"
  },"20": {
    "doc": "PMSM FOC using Quadrature Encoder",
    "title": "MHC Project Configurations",
    "content": ". | AFEC Peripheral: . The AFEC is used to measure analog quantities. Four channels are used to measure the Phase Current U, the Phase Current V, the DC Bus Voltage and the Potentiometer. Conversion is triggered at the PWM (zero match + offset of the switch delay) . | PWM Peripheral: . This peripheral is used to generated three phase synchronous PWM waveforms. Fault functionality is also enabled to switch off the output waveforms asynchronously. | TC QDEC Peripheral: . TC peripheral is configured in QDEC mode. It is used to decode the rotor position and speed from quadrature encoder signals. | X2CScope: . This component adds X2C scope protocol code. This uses UART to communicate to the host PC. X2CScope allows user to monitor variables runtime. | UART Peripheral: . The UART is used for X2CScope communication to observe graphs and variable values in run time . | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_encoder_sam_e70/readme.html#mhc-project-configurations",
    "relUrl": "/apps/pmsm_foc_encoder_sam_e70/readme.html#mhc-project-configurations"
  },"21": {
    "doc": "PMSM FOC using Quadrature Encoder",
    "title": "Control Algorithm",
    "content": "This section briefly explains the FOC control algorithm, software design and implementation. Refer to Application note AN2520 for the PLL estimator based sensor-less FOC technique in detail. Field Oriented Control is the technique used to achieve the decoupled control of torque and flux. This is done by transforming the stator current quantities (phase currents) from stationary reference frame to torque and flux producing currents components in rotating reference frame using mathematical transformations. The Field Oriented Control is done as follows: . | Measure the motor phase currents. | Transform them into the two phase system (a, b) using the Clarke transformation. | Calculate the rotor position angle. | Transform stator currents into the d,q-coordinate system using the Park transformation. | The stator current torque (iq) and flux (id) producing components are controlled separately by the controllers. | The output stator voltage space vector is transformed back from the d,q-coordinate system into the two phase system fixed with the stator by the Inverse Park transformation. | Using the space vector modulation, the three-phase output voltage is generated. | . Quadrature Encoder based FOC : . Rotor position and speed are determined using quadrature encoder sensor. TC peripheral is configured in QDEC position mode. It counts the decoded quadrature pulses which is the position of the rotor. In this example, counter is a free running counter and software logic is implemented to get the exact angular position from the count. Speed is calculated by measuring the number of quadrature pulses in a fixed time interval. Rotor is first aligned to a known position by exiciting either d-axis or q-axis. And then motor is controlled in a closed loop there after. The following block diagram shows the software realization of the FOC algorithm. ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_encoder_sam_e70/readme.html#control-algorithm",
    "relUrl": "/apps/pmsm_foc_encoder_sam_e70/readme.html#control-algorithm"
  },"22": {
    "doc": "PMSM FOC using Quadrature Encoder",
    "title": "Software Design",
    "content": "1. SAME54, SAME70, PIC32MK MCF and PIC32MK MCM . | PMSM FOC Control loop is implemented in the ADC result ready interrupt. Refer to the flow chart given below. | ADC channel conversion is triggered by the PWM overflow/zero match event. This trigger point could vary based on the current measurement techniques and MCU PWM IP implementation. | ADC interrupt frequency depends on the PWM frequency. | Slow loop task is executed 10 times slower than ADC interrupt. Polling of switches for user inputs and reference speed calculation is handled in the slow loop task. | . Timing Diagram . Flow Chart . State Machine . | Idle: | . In this state, control waits for the switch press. | Flying Start: | . In this state, the control algorithm detects if the motor is freewheeling. If the motor’s freewheeling speed is above the minimum Flying Start Detect speed and in the same direction as the command direction, the state machine directly enters in to “Closed Loop” state. However, if the freewheeling speed is less than minimum Flying Start Detect speed, the control algorithm attempts to passive brake the motor by applying pulses of “NUll Vectors”. If the freewheeling speed is more than the minimum Flying Start Detect speed but in the opposite direction of the command direction, the control algorithm attempts to active brake the motor by regenerative braking (if enabled). Once the motor speed falls below the minimum Flying Start Detect speed, the control algorithm attempts to passive brake the motor by applying pulses of “NUll Vectors”. | Field Alignment: | . Rotor is aligned to known position at D-axis or Q-axis by applying a pre-defined value of the current for a pre-defined length of time. The magnitude of the current and the length of the time for which it is applied depends upon the electrical and mechanical time constant of the PMSM motor drive. Electrical time constant of the motor is a function of R and L values of the motor windings, whereas the mechanical time constant of the motor drive is primarily a function of the static load on the motor shaft. | Open Loop: | . This state is applicable to sensorless position feedback methods. In this state, the speed of the PMSM motor is gradually ramped up using an open loop control. During this mode, the rotor angle is derived from the asserted open loop speed reference. This derived rotor angle would be lagging from the actual rotor angle. The speed is ramped up linearly to a minimum value required for the PLL estimator to estimate the electrical speed of the PMSM motor with sufficient accuracy. Rotor angle information is obtained by integrating the estimated electrical speed of the motor. | Closing Loop: | . In this state, control waits for stabilization time. | Closed Loop: | . Control switched to closed loop and rotor angle is obtained from the configured position feedback method. Code Structure . Configurations: . | mc_userparameters.h contains the user configurations. | mc_derivedparameters.h contains the calculated values used in the code. | mc_pmsm_foc_common.h - common data structures and defines | . PMSM_FOC: . | mc_pmsm_foc.c/h - PMSM FOC algorithm interface file | . Interrupts: . | mc_control_loop.c - Control loop is implemented in the ADC result ready ISR. | mc_errorhandler.c - PWM fault ISR to take corrective action on over-current | . Control library: . | mc_lib.c/h - FOC library | mc_picontroller.c/h - PI controller implementation | mc_pwm.c/h - Space Vector modulation (SVM) and updating PWM duty cycles | lib_mc_flyingstart.a - Flying Start Control Library | . Control Middleware: . | mc_speed.c/h - Calculate the reference speed | mc_rotorposition.c/.h - Calculate the position and speed of the rotor | mc_voltagemeasurement.c/h - Get the DC Bus voltage | mc_currentmeasurement.c/h - Get the motor phase currents | . HAL: . | mc_hal.h - Hardware Abstraction Layer to interact with PLIBs | . 2. SAMC21 and PIC32CM MC . | PMSM FOC Control loop is implemented in the ADC result ready interrupt. Refer to the flow chart given below. | ADC channel conversion is triggered by the PWM overflow/zero match event. This trigger point could vary based on the current measurement techniques and MCU PWM IP implementation. | For bandwidth constraints, the FOC is executed every alternate PWM cycle | Slow loop task is executed in every 10ms in the task process. Polling of switches for user inputs is handled in the slow loop task. | . Timing Diagram . Flow Chart . State Machine . | Idle: | . In this state, control waits for the switch press. | Field Alignment: | . Rotor is aligned to known position at D-axis or Q-axis by applying a pre-defined value of the current for a pre-defined length of time. The magnitude of the current and the length of the time for which it is applied depends upon the electrical and mechanical time constant of the PMSM motor drive. Electrical time constant of the motor is a function of R and L values of the motor windings, whereas the mechanical time constant of the motor drive is primarily a function of the static load on the motor shaft. | Closed Loop: | . Control switched to closed loop and rotor angle is obtained from the configured position feedback method. Code Structure . Configurations: . | mc_userparameters.h contains the user configurations. | . PMSM FOC Application: . | mc_pmsm_foc.c/h - PMSM FOC algorithm interface file | . Interrupts: . | mc_function_coordinator.c - Initializes and coordinates motor control ISR and slow task processes | mc_error_handler.c - PWM fault ISR to take corrective action on over-current | . Control Middleware: . | mc_motor_control.c/h - Implements the motor control state machines. | . Control library: . | mc_start_up.c/h - Implements the initial field alignment and open loop start-up profile. | mc_speed_control.c/h - Calculate and regulates the reference speed | mc_current_control.c/h - Controls the direct and quadrature axis currents | mc_rotor_position.c/.h - Calculate the position and speed of the rotor | mc_voltage_measurement.c/h - Get the DC Bus voltage | mc_current_measurement.c/h - Get the motor phase currents | mc_interface_handling.c/h - Manages global variables and data-types | mc_generic_library.c/h - FOC library | mc_pwm.c/h - Space Vector modulation (SVM) and updating PWM duty cycles | mc_ramp_profiler.c/h - Speed reference profiles for user inputs | . HAL: . | mc_hardware_abstraction.c/h - Hardware Abstraction Layer to interact with PLIBs | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_encoder_sam_e70/readme.html#software-design",
    "relUrl": "/apps/pmsm_foc_encoder_sam_e70/readme.html#software-design"
  },"23": {
    "doc": "PMSM FOC using Quadrature Encoder",
    "title": "Development Kits",
    "content": "MCLV2 with ATSAME70 PIM . Downloading and building the application . To clone or download this application from Github, go to the main page of this repository and then click Clone button to clone this repository or download as zip file. This content can also be downloaded using content manager by following these instructions. Path of the application within the repository is apps/pmsm_foc_encoder_sam_e70 . To build the application, refer to the following table and open the project using its IDE. | Project Name | Description | Hardware Setup Guide | . | mclv2_sam_e70_pim.X | MPLABX project for MCLV2 board with ATSAME70 PIM | Hardware Setup and Running The Application on MCLV2 with ATSAME70 PIM | . |   |   |   | . MCHV3 with ATSAME70 PIM . Downloading and building the application . To clone or download this application from Github, go to the main page of this repository and then click Clone button to clone this repository or download as zip file. This content can also be downloaded using content manager by following these instructions. Path of the application within the repository is apps/pmsm_foc_encoder_sam_e70 . To build the application, refer to the following table and open the project using its IDE. | Project Name | Description | Hardware Setup Guide | . | mchv3_sam_e70_pim.X | MPLABX project for MCHV3 board with ATSAME70 PIM | Hardware Setup and Running The Application on MCHV3 with ATSAME70 PIM | . |   |   |   | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_encoder_sam_e70/readme.html#development-kits",
    "relUrl": "/apps/pmsm_foc_encoder_sam_e70/readme.html#development-kits"
  },"24": {
    "doc": "PMSM FOC using Quadrature Encoder",
    "title": "PMSM FOC using Quadrature Encoder",
    "content": ". ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_encoder_sam_e70/readme.html",
    "relUrl": "/apps/pmsm_foc_encoder_sam_e70/readme.html"
  },"25": {
    "doc": "PMSM FOC using PLL Estimator",
    "title": "PMSM FOC using PLL Estimator",
    "content": "This example application shows how to control the Permanent Magnet Synchronous Motor (PMSM) with PLL Estimator based Field Oriented Control (FOC) on a SAME70 Micro-controller. ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_pll_estimator_sam_e70/readme.html#pmsm-foc-using-pll-estimator",
    "relUrl": "/apps/pmsm_foc_pll_estimator_sam_e70/readme.html#pmsm-foc-using-pll-estimator"
  },"26": {
    "doc": "PMSM FOC using PLL Estimator",
    "title": "Description",
    "content": "Permanent Magnet Synchronous Motor (PMSM) is controlled using Field Oriented Control (FOC). Rotor position and speed is determined using PLL estimator technique. Motor start/stop operation is controlled by the switch and motor speed can be changed by the on-board potentiometer. Waveforms and variables can be monitored runtime using X2CScope. Key features enabled in this project are: . | Dual shunt current measurement | Speed control loop | Field weakening | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_pll_estimator_sam_e70/readme.html#description",
    "relUrl": "/apps/pmsm_foc_pll_estimator_sam_e70/readme.html#description"
  },"27": {
    "doc": "PMSM FOC using PLL Estimator",
    "title": "MHC Project Configurations",
    "content": ". | AFEC Peripheral: . The AFEC is used to measure analog quantities. Four channels are used to measure the Phase Current U, the Phase Current V, the DC Bus Voltage and the Potentiometer. Conversion is triggered at the PWM (zero match + offset of the switch delay) . | PWM Peripheral: . This peripheral is used to generated three phase synchronous PWM waveforms. Fault functionality is also enabled to switch off the output waveforms asynchronously. | X2CScope: . This component adds X2C scope protocol code. This uses UART to communicate to the host PC. X2CScope allows user to monitor variables runtime. | UART Peripheral: . The UART is used for X2CScope communication to observe graphs and variable values in run time . | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_pll_estimator_sam_e70/readme.html#mhc-project-configurations",
    "relUrl": "/apps/pmsm_foc_pll_estimator_sam_e70/readme.html#mhc-project-configurations"
  },"28": {
    "doc": "PMSM FOC using PLL Estimator",
    "title": "Control Algorithm",
    "content": "This section briefly explains the FOC control algorithm, software design and implementation. Refer to Application note AN2520 for the PLL estimator based sensor-less FOC technique in detail. Field Oriented Control is the technique used to achieve the decoupled control of torque and flux. This is done by transforming the stator current quantities (phase currents) from stationary reference frame to torque and flux producing currents components in rotating reference frame using mathematical transformations. The Field Oriented Control is done as follows: . | Measure the motor phase currents. | Transform them into the two phase system (a, b) using the Clarke transformation. | Calculate the rotor position angle. | Transform stator currents into the d,q-coordinate system using the Park transformation. | The stator current torque (iq) and flux (id) producing components are controlled separately by the controllers. | The output stator voltage space vector is transformed back from the d,q-coordinate system into the two phase system fixed with the stator by the Inverse Park transformation. | Using the space vector modulation, the three-phase output voltage is generated. | . PLL estimator : . This estimator uses PLL structure to estimate the rotor position and thus speed. Its operating principle is based on the fact that the d-component of the Back Electromotive Force (BEMF) must be equal to zero at a steady state functioning mode. It can not estimate the rotor angle at lower rotor speeds because of very low back EMF. So, open loop startup is used till required minimum speed is achieved. The following block diagram shows the software realization of the FOC algorithm. ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_pll_estimator_sam_e70/readme.html#control-algorithm",
    "relUrl": "/apps/pmsm_foc_pll_estimator_sam_e70/readme.html#control-algorithm"
  },"29": {
    "doc": "PMSM FOC using PLL Estimator",
    "title": "Software Design",
    "content": "1. SAME54, SAME70, PIC32MK MCF and PIC32MK MCM . | PMSM FOC Control loop is implemented in the ADC result ready interrupt. Refer to the flow chart given below. | ADC channel conversion is triggered by the PWM overflow/zero match event. This trigger point could vary based on the current measurement techniques and MCU PWM IP implementation. | ADC interrupt frequency depends on the PWM frequency. | Slow loop task is executed 10 times slower than ADC interrupt. Polling of switches for user inputs and reference speed calculation is handled in the slow loop task. | . Timing Diagram . Flow Chart . State Machine . | Idle: | . In this state, control waits for the switch press. | Flying Start: | . In this state, the control algorithm detects if the motor is freewheeling. If the motor’s freewheeling speed is above the minimum Flying Start Detect speed and in the same direction as the command direction, the state machine directly enters in to “Closed Loop” state. However, if the freewheeling speed is less than minimum Flying Start Detect speed, the control algorithm attempts to passive brake the motor by applying pulses of “NUll Vectors”. If the freewheeling speed is more than the minimum Flying Start Detect speed but in the opposite direction of the command direction, the control algorithm attempts to active brake the motor by regenerative braking (if enabled). Once the motor speed falls below the minimum Flying Start Detect speed, the control algorithm attempts to passive brake the motor by applying pulses of “NUll Vectors”. | Field Alignment: | . Rotor is aligned to known position at D-axis or Q-axis by applying a pre-defined value of the current for a pre-defined length of time. The magnitude of the current and the length of the time for which it is applied depends upon the electrical and mechanical time constant of the PMSM motor drive. Electrical time constant of the motor is a function of R and L values of the motor windings, whereas the mechanical time constant of the motor drive is primarily a function of the static load on the motor shaft. | Open Loop: | . This state is applicable to sensorless position feedback methods. In this state, the speed of the PMSM motor is gradually ramped up using an open loop control. During this mode, the rotor angle is derived from the asserted open loop speed reference. This derived rotor angle would be lagging from the actual rotor angle. The speed is ramped up linearly to a minimum value required for the PLL estimator to estimate the electrical speed of the PMSM motor with sufficient accuracy. Rotor angle information is obtained by integrating the estimated electrical speed of the motor. | Closing Loop: | . In this state, control waits for stabilization time. | Closed Loop: | . Control switched to closed loop and rotor angle is obtained from the configured position feedback method. Code Structure . Configurations: . | mc_userparameters.h contains the user configurations. | mc_derivedparameters.h contains the calculated values used in the code. | mc_pmsm_foc_common.h - common data structures and defines | . PMSM_FOC: . | mc_pmsm_foc.c/h - PMSM FOC algorithm interface file | . Interrupts: . | mc_control_loop.c - Control loop is implemented in the ADC result ready ISR. | mc_errorhandler.c - PWM fault ISR to take corrective action on over-current | . Control library: . | mc_lib.c/h - FOC library | mc_picontroller.c/h - PI controller implementation | mc_pwm.c/h - Space Vector modulation (SVM) and updating PWM duty cycles | lib_mc_flyingstart.a - Flying Start Control Library | . Control Middleware: . | mc_speed.c/h - Calculate the reference speed | mc_rotorposition.c/.h - Calculate the position and speed of the rotor | mc_voltagemeasurement.c/h - Get the DC Bus voltage | mc_currentmeasurement.c/h - Get the motor phase currents | . HAL: . | mc_hal.h - Hardware Abstraction Layer to interact with PLIBs | . 2. SAMC21 and PIC32CM MC . | PMSM FOC Control loop is implemented in the ADC result ready interrupt. Refer to the flow chart given below. | ADC channel conversion is triggered by the PWM overflow/zero match event. This trigger point could vary based on the current measurement techniques and MCU PWM IP implementation. | For bandwidth constraints, the FOC is executed every alternate PWM cycle | Slow loop task is executed in every 10ms in the task process. Polling of switches for user inputs is handled in the slow loop task. | . Timing Diagram . Flow Chart . State Machine . | Idle: | . In this state, control waits for the switch press. | Field Alignment: | . Rotor is aligned to known position at D-axis or Q-axis by applying a pre-defined value of the current for a pre-defined length of time. The magnitude of the current and the length of the time for which it is applied depends upon the electrical and mechanical time constant of the PMSM motor drive. Electrical time constant of the motor is a function of R and L values of the motor windings, whereas the mechanical time constant of the motor drive is primarily a function of the static load on the motor shaft. | Open Loop: | . This state is applicable to sensorless position feedback methods. In this state, the speed of the PMSM motor is gradually ramped up using an open loop control. During this mode, the rotor angle is derived from the asserted open loop speed reference. This derived rotor angle would be lagging from the actual rotor angle. The speed is ramped up linearly to a minimum value required for the PLL estimator to estimate the electrical speed of the PMSM motor with sufficient accuracy. Rotor angle information is obtained by integrating the estimated electrical speed of the motor. | Closed Loop: | . Control switched to closed loop and rotor angle is obtained from the configured position feedback method. Code Structure . Configurations: . | mc_userparameters.h contains the user configurations. | . PMSM FOC Application: . | mc_pmsm_foc.c/h - PMSM FOC algorithm interface file | . Interrupts: . | mc_function_coordinator.c - Initializes and coordinates motor control ISR and slow task processes | mc_error_handler.c - PWM fault ISR to take corrective action on over-current | . Control Middleware: . | mc_motor_control.c/h - Implements the motor control state machines. | . Control library: . | mc_start_up.c/h - Implements the initial field alignment and open loop start-up profile. | mc_speed_control.c/h - Calculate and regulates the reference speed | mc_current_control.c/h - Controls the direct and quadrature axis currents | mc_rotor_position.c/.h - Calculate the position and speed of the rotor | mc_voltage_measurement.c/h - Get the DC Bus voltage | mc_current_measurement.c/h - Get the motor phase currents | mc_interface_handling.c/h - Manages global variables and data-types | mc_generic_library.c/h - FOC library | mc_pwm.c/h - Space Vector modulation (SVM) and updating PWM duty cycles | mc_ramp_profiler.c/h - Speed reference profiles for user inputs | . HAL: . | mc_hardware_abstraction.c/h - Hardware Abstraction Layer to interact with PLIBs | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_pll_estimator_sam_e70/readme.html#software-design",
    "relUrl": "/apps/pmsm_foc_pll_estimator_sam_e70/readme.html#software-design"
  },"30": {
    "doc": "PMSM FOC using PLL Estimator",
    "title": "Development Kits",
    "content": "MCLV2 with ATSAME70 PIM . Downloading and building the application . To clone or download this application from Github, go to the main page of this repository and then click Clone button to clone this repository or download as zip file. This content can also be downloaded using content manager by following these instructions. Path of the application within the repository is apps/pmsm_foc_pll_estimator_sam_e70 . To build the application, refer to the following table and open the project using its IDE. | Project Name | Description | Hardware Setup Guide | . | mclv2_sam_e70_pim.X | MPLABX project for MCLV2 board with ATSAME70 PIM | Hardware Setup and Running The Application on MCLV2 with ATSAME70 PIM | . |   |   |   | . MCHV3 with ATSAME70 PIM . Downloading and building the application . To clone or download this application from Github, go to the main page of this repository and then click Clone button to clone this repository or download as zip file. This content can also be downloaded using content manager by following these instructions. Path of the application within the repository is apps/pmsm_foc_pll_estimator_sam_e70 . To build the application, refer to the following table and open the project using its IDE. | Project Name | Description | Hardware Setup Guide | . | mchv3_sam_e70_pim.X | MPLABX project for MCHV3 board with ATSAME70 PIM | Hardware Setup and Running The Application on MCHV3 with ATSAME70 PIM | . |   |   |   | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_pll_estimator_sam_e70/readme.html#development-kits",
    "relUrl": "/apps/pmsm_foc_pll_estimator_sam_e70/readme.html#development-kits"
  },"31": {
    "doc": "PMSM FOC using PLL Estimator",
    "title": "PMSM FOC using PLL Estimator",
    "content": ". ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_pll_estimator_sam_e70/readme.html",
    "relUrl": "/apps/pmsm_foc_pll_estimator_sam_e70/readme.html"
  },"32": {
    "doc": "PMSM FOC using Sliding Mode Observer",
    "title": "PMSM FOC using Sliding Mode Observer",
    "content": "This example application shows how to control the Permanent Magnet Synchronous Motor (PMSM) with Sliding Mode Observer based Field Oriented Control (FOC) on a SAME70 Micro-controller. ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_smo_sam_e70/readme.html#pmsm-foc-using-sliding-mode-observer",
    "relUrl": "/apps/pmsm_foc_smo_sam_e70/readme.html#pmsm-foc-using-sliding-mode-observer"
  },"33": {
    "doc": "PMSM FOC using Sliding Mode Observer",
    "title": "Description",
    "content": "Permanent Magnet Synchronous Motor (PMSM) is controlled using Field Oriented Control (FOC). Rotor position and speed is determined using Sliding Mode Observer (SMO) technique. Motor start/stop operation is controlled by the switch and motor speed can be changed by the on-board potentiometer. Waveforms and variables can be monitored runtime using X2CScope. Key features enabled in this project are: . | Dual shunt current measurement | Speed control loop | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_smo_sam_e70/readme.html#description",
    "relUrl": "/apps/pmsm_foc_smo_sam_e70/readme.html#description"
  },"34": {
    "doc": "PMSM FOC using Sliding Mode Observer",
    "title": "MHC Project Configurations",
    "content": ". | AFEC Peripheral: | . The AFEC (ADC) is used to measure analog quantities. Four channels are used to measure the Phase Current A, the Phase Current B, the DC Bus Voltage and the Potentiometer. Conversion is triggered at the PWM (zero match + offset of the switch delay) . | PWM Peripheral: | . This peripheral is used to generated three phase synchronous PWM waveforms. Fault functionality is also enabled to switch off the output waveforms asynchronously . | UART Peripheral: | . The UART is used for X2CScope communication to observe graphs and variable values in run time . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_smo_sam_e70/readme.html#mhc-project-configurations",
    "relUrl": "/apps/pmsm_foc_smo_sam_e70/readme.html#mhc-project-configurations"
  },"35": {
    "doc": "PMSM FOC using Sliding Mode Observer",
    "title": "Control Algorithm",
    "content": "This section briefly explains the FOC control algorithm, software design and implementation. Field Oriented Control is the technique used to achieve the decoupled control of torque and flux. This is done by transforming the stator current quantities (phase currents) from stationary reference frame to torque and flux producing currents components in rotating reference frame using mathematical transformations. The Field Oriented Control is done as follows: . | Measure the motor phase currents. | Transform them into the two phase system (a, b) using the Clarke transformation. | Calculate the rotor position angle. | Transform stator currents into the d,q-coordinate system using the Park transformation. | The stator current torque (iq) and flux (id) producing components are controlled separately by the controllers. | The output stator voltage space vector is transformed back from the d,q-coordinate system into the two phase system fixed with the stator by the Inverse Park transformation. | Using the space vector modulation, the three-phase output voltage is generated. | . Sliding Mode Observer (SMO) : . Sliding Mode Observer is used to estimate the rotor position and thus speed. The electrical rotor position and speed is calculated using the Sliding Mode Observer. The main disadvantage of SMO Observer is its inability to estimate the rotor angle at lower rotor speeds because of very low value of back EMF. Therefore, the FOC algorithm also integrates the ramp-up profile for motor start. The reference speed is incremented linearly using a open loop phase voltage control until the required minimum reference speed for the SMO observer is reached. The Sliding Mode Observer ( SMO ) is based on the principle of sliding mode control. It generates a sliding motion on the error between the plant output and the output of the observer such it produces a set of states that are precisely drives the estimated output to the actual output of the plant. The sliding mode observer consists of a model based current observer followed by a back EMF Observer. The observed back EMF is filtered and then used to extract the position information from it. The following figure shows a typical Sliding Mode Observer: . The current and back EMF observer is modeled in discrete time by following equations: . Current Observer: . Back EMF Observer: . The following block diagram shows the software realization of the FOC algorithm. ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_smo_sam_e70/readme.html#control-algorithm",
    "relUrl": "/apps/pmsm_foc_smo_sam_e70/readme.html#control-algorithm"
  },"36": {
    "doc": "PMSM FOC using Sliding Mode Observer",
    "title": "Software Design",
    "content": "The following figure shows the various state machines of the the motor control software. In the software, the PMSM speed control task is realized by a state machine as shown in the previous figure. The following sections briefly describes the various states in the PMSM speed control task: . | Initialize: | . In this state, following tasks are performed: Initialization and configuration of NVIC, AFEC, PWM motor control peripherals for generation of periodic ADC triggers and ADC conversion interrupt Current Offset measurement and calibration Initialize PI controller parameters for speed and current control loops . | Start: | . In this state, the motor control state variables are reset and periodic ADC conversion interrupt is enabled. Control waits for the switch press. | Run: | . In this state, the motor starts spinning. The following flow chart and the timing diagram shows the tasks performed in run state: . In run state, two threads are executed- Main task thread and ADC Interrupt task thread. The current control and speed control is carried out in the ADC interrupt task thread. The main task thread monitors the external switches and maintains the state machine. Therefore, the ADC interrupt cycle indicates the current and speed control frequency. As pointed out earlier, the SMO is unable to estimate the rotor position information at the start-up phase when the speed is very low. This necessitates the motor control algorithm to integrate an open-loop control mechanism to linearly ramp the motor speed to the required minimum speed before switching over to closed loop control. Therefore, the motor control state can further be classified into three sub-states - Field Alignment, Open-loop Control and Close-loop Control. Field Alignment: In this mode, a limited value of DC current is applied to the U phase of PMSM motor in order to align the rotor magnetic field with the U-phase of the motor. The time for which DC current has to be applied to achieve the field alignment depends on the time constant of the PMSM motor drive. Open Loop Control: In this mode, the speed of the PMSM motor is gradually ramped up using an open loop control. The speed is ramped up linearly to a minimum value required for the sensor-less SMO observer to estimate the speed of the PMSM motor with required accuracy. Close Loop Control: In this mode, the speed can be regulated using closed loop SMO based sensor-less FOC algorithm. | Stop: In this state, the PWM channels are disabled thereby stopping the motor. The periodic ADC trigger and conversion interrupt is disabled. | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_smo_sam_e70/readme.html#software-design",
    "relUrl": "/apps/pmsm_foc_smo_sam_e70/readme.html#software-design"
  },"37": {
    "doc": "PMSM FOC using Sliding Mode Observer",
    "title": "Software Configurations",
    "content": ". | The PWM controller frequency ( in Hz ) can be configured by setting the following macro in userparams.h file. This frequency should be same as the frequency configured in the PWM peripheral in the MHC. | . | Macro | Description | . | PWM_FREQUENCY | Current controller and PWM frequency in Hz | . |   |   | . | Setting motor specific -parameter: Set the motor following motor parameters in userparams.h file. | . | Macro | Description | Unit | . | MOTOR_PER_PHASE_RESISTANCE | Motor per phase resistance | ohm | . | MOTOR_PER_PHASE_INDUCTANCE | Motor per phase inductance | H | . | MOTOR_BACK_EMF_CONSTANT_Vpeak_Line_Line_KRPM_MECH | Back EMF constant | Vpk(L-L)/KRPM | . | NOPOLESPAIRS | Number of pole pairs |   | . | NOMINAL_SPEED_RPM | Rated mechanical speed of the motor | RPM | . |   |   |   | . | Setting PI Controller parameters: Depending on the type of motor used, and the corresponding application PI controller parameters should to be updated in userparams.h file. | . Parameters for speed Control loop: . | Macro | Description | . | SPEEDCNTR_PTERM | Proportional gain of speed control loop | . | SPEEDCNTR_ITERM | Integral gain of speed control loop | . | SPEEDCNTR_CTERM | Anti-windup term of speed control loop | . | SPEEDCNTR_OUTMAX | Maximum controller output of speed control loop | . |   |   | . Parameters for Id current loop: . | Macro | Description | . | D_CURRCNTR_PTERM | Proportional gain of Id current control loop | . | D_CURRCNTR_ITERM | Integral gain of Id current control loop | . | D_CURRCNTR_CTERM | Anti-windup term of Id current control loop | . | D_CURRCNTR_OUTMAX | Maximum controller output of Id current control loop | . |   |   | . Parameters for Iq current loop: . | Macro | Description | . | Q_CURRCNTR_PTERM | Proportional gain of Iq current control loop | . | Q_CURRCNTR_ITERM | Integral gain of Iq current control loop | . | Q_CURRCNTR_CTERM | Anti-windup term of Iq current control loop | . | Q_CURRCNTR_OUTMAX | Maximum controller output of Iq current control loop | . |   |   | . | Debugging Features: Open loop functioning is useful for debugging when using new motor. This feature can be enabled by setting the following macros in userparams.h file. | . Parameters for Iq current loop: . | Macro | Description | . | OPEN_LOOP_FUNCTIONING | Open loop enable switch (0 - Disable, 1 - Enable ) | . | Q_CURRENT_REF_OPENLOOP | Startup current in open loop | . |   |   | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_smo_sam_e70/readme.html#software-configurations",
    "relUrl": "/apps/pmsm_foc_smo_sam_e70/readme.html#software-configurations"
  },"38": {
    "doc": "PMSM FOC using Sliding Mode Observer",
    "title": "Development Kits",
    "content": "MCLV2 with ATSAME70 PIM . Downloading and building the application . To clone or download this application from Github, go to the main page of this repository and then click Clone button to clone this repository or download as zip file. This content can also be downloaded using content manager by following these instructions. Path of the application within the repository is apps/pmsm_foc_smo_sam_e70 . To build the application, refer to the following table and open the project using its IDE. | Project Name | Description | Hardware Setup Guide | . | mclv2_sam_e70_pim.X | MPLABX project for MCLV2 board with ATSAME70 PIM | Hardware Setup and Running The Application on MCLV2 with ATSAME70 PIM | . |   |   |   | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_smo_sam_e70/readme.html#development-kits",
    "relUrl": "/apps/pmsm_foc_smo_sam_e70/readme.html#development-kits"
  },"39": {
    "doc": "PMSM FOC using Sliding Mode Observer",
    "title": "PMSM FOC using Sliding Mode Observer",
    "content": ". ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_smo_sam_e70/readme.html",
    "relUrl": "/apps/pmsm_foc_smo_sam_e70/readme.html"
  },"40": {
    "doc": "PFC with PMSM FOC using PLL Estimator",
    "title": "Power Factor Correction with PMSM FOC using PLL Estimator",
    "content": "This example application shows how to control the Permanent Magnet Synchronous Motor (PMSM) with PLL Estimator based Field Oriented Control (FOC) along with Power Factor Correction (PFC)on a SAME70 Micro-controller. ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_pfc_foc_pll_estimator_sam_e70/readme.html#power-factor-correction-with-pmsm-foc-using-pll-estimator",
    "relUrl": "/apps/pmsm_pfc_foc_pll_estimator_sam_e70/readme.html#power-factor-correction-with-pmsm-foc-using-pll-estimator"
  },"41": {
    "doc": "PFC with PMSM FOC using PLL Estimator",
    "title": "Description",
    "content": "Power Factor Correction ( PFC ) shapes the input current drawn from the AC mains to be in synchronization with the input AC voltage. This project uses Boost-Converter topology to carry out the Power Factor Correction. Permanent Magnet Synchronous Motor (PMSM) is controlled using Field Oriented Control (FOC). Rotor position and speed is determined using PLL estimator technique. Motor start/stop operation is controlled by the switch and motor speed can be changed by the on-board potentiometer. Waveforms and variables can be monitored runtime using X2CScope. Key features enabled in this project are: . | Power factor correction | Dual shunt current measurement | Speed control loop | Field weakening | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_pfc_foc_pll_estimator_sam_e70/readme.html#description",
    "relUrl": "/apps/pmsm_pfc_foc_pll_estimator_sam_e70/readme.html#description"
  },"42": {
    "doc": "PFC with PMSM FOC using PLL Estimator",
    "title": "MHC Project Configurations",
    "content": ". | AFEC Peripheral: | . The AFEC0 peripheral is used to measure analog quantities for motor control operation. Four channels are used to measure the Phase Current A, the Phase Current B, the DC Bus Voltage and the Potentiometer. Conversion is triggered at the PWM (zero match + offset of the switch delay) . The AFEC1 peripheral is used to measure analog quantities for power factor correction. Three channels are used to measure the input rectified voltage, input rectified current, and the output DC link inverter voltage. Conversion is triggered at the PWM period match . | PWM Peripheral: | . The PWM0 peripheral is used to generated three phase synchronous PWM waveforms. Fault functionality is also enabled to switch off the output waveforms asynchronously . The PWM1 peripheral is used to generate the PWM waveform for the boost converter switch for current waveform shaping . | UART Peripheral: | . The UART is used for X2CScope communication to observe graphs and variable values in run time . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_pfc_foc_pll_estimator_sam_e70/readme.html#mhc-project-configurations",
    "relUrl": "/apps/pmsm_pfc_foc_pll_estimator_sam_e70/readme.html#mhc-project-configurations"
  },"43": {
    "doc": "PFC with PMSM FOC using PLL Estimator",
    "title": "Control Algorithm",
    "content": "The Power Factor Correction is a technique of increasing the power factor of the input AC power supply, thereby providing following advantages: . | Low line harmonics. | Reduced input current. | Reduced kVA requirement. | Improved line efficiency, and improved electromagnetic compatibility. | . The Power Factor Correction techniques can be classified as - Passive and Active. The Passive Power Factor Correction uses passive components, while the active Power Factor Correction Technique uses power electronic switches to carry out Power Factor Correction. Following figure shows the various Power Factor Correction techniques classification: . This project uses Boost topology for Power Factor Correction. A boost converter is placed between the rectifier and output inverter stage to synchronize input AC current to the input AC voltage. For details, refer to application note AN1106. The block diagram of a typical boost converter topology based Power Factor Correction is shown below. Block Diagram: . The power factor correction is implemented in following steps: . | Measure the rectifier stage rectified AC voltage and rectified AC current, and output inverter stage DC bus voltage via high speed ADC channels. | Regulate the DC bus voltage using the Voltage Error Compensator. | Determine the reference input for Current Error Compensator based on the output of Voltage Error Compensator and Voltage Feed-Forward Compensator. | Track the rectified reference AC current using a Current Error Compensator. | . Current Error Compensator: . The inner loop in the control block forms the current loop. The input to the current loop is the reference current signal IACREF and the actual inductor current IAC. The current error compensator is designed to produce a control output such that the inductor current IAC follows the reference current IACREF. The current loop should run at a much faster rate when compared to the voltage loop. The bandwidth of the current compensator should be higher for correctly tracking the semi-sinusoidal waveform at twice the input frequency. The current controller GI produces a duty cycle value after appropriate scaling to drive the gate of the boost converter MOSFET. Voltage Error Compensator: . The outer loop in the control block forms the voltage loop. The input to the voltage loop is the reference DC voltage VDCREF and the actual sensed output DC voltage VDC. The voltage error compensator is designed to produce a control output such that the DC bus voltage VDC remains constant at the reference value VDCREF regardless of variations in the load current IO and the supply voltage VAC. The voltage controller GV produces a control signal, which determines the reference current IACREF for the inner current loop. The output voltage is controlled by the voltage error compensator. When the input voltage increases, the product of VAC and VPI increases, and thereby increasing the programming signal. When this signal is divided by the square of the average voltage signal, it results in the current reference signal being reduced proportionally. The outcome is that the current is reduced proportionally to the increase in voltage, thereby keeping the input power constant. This ensures that the reference control output IACREF from the voltage compensator is maximum such that the rated output power is delivered at minimum input voltage. Voltage Feed-Forward Compensator: . If the voltage decreases, the product (VAC · VPI), which determines IACREF, also proportionally decreases. However, to maintain a constant output power at reduced input voltage, the term IACREF should proportionally increase. The purpose of having an input voltage feed-forward, is to maintain the output power constant as determined by the load regardless of variations in the input line voltage. This compensator implemented digitally by calculating the average value of the input line voltage, squaring this average value and using the result as a divider for the input reference current, which is fed to the current error compensator. FOC with PLL estimator : . This estimator uses PLL structure to estimate the rotor position and thus speed. Its operating principle is based on the fact that the d-component of the Back Electromotive Force (BEMF) must be equal to zero at a steady state functioning mode. It can not estimate the rotor angle at lower rotor speeds because of very low back EMF. So, open loop startup is used till required minimum speed is achieved. The following block diagram shows the software realization of the FOC algorithm. ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_pfc_foc_pll_estimator_sam_e70/readme.html#control-algorithm",
    "relUrl": "/apps/pmsm_pfc_foc_pll_estimator_sam_e70/readme.html#control-algorithm"
  },"44": {
    "doc": "PFC with PMSM FOC using PLL Estimator",
    "title": "PFC Software Design",
    "content": "The following section describes the software flow diagram for the power factor correction application project. The power factor correction algorithm in this example project is realized by following state machines. | PFC Init State: In this state following tasks are performed: Initialization and configuration of NVIC, AFEC, PWM peripherals for generation of periodic ADC triggers and ADC conversion interrupt. Calibration of input rectified AC current sense amplifiers. Initialization of reference DC voltage and reference rectified AC current PI controllers used for Power Factor Correction. | PFC Start state: In this state following tasks are performed: Start PWM peripherals timer to trigger ADC conversion interrupt. | PFC Running State: In this state, the power factor correction ISR is executed. | . | PFC Stop State: In this state, the power factor correction is disabled. | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_pfc_foc_pll_estimator_sam_e70/readme.html#pfc-software-design",
    "relUrl": "/apps/pmsm_pfc_foc_pll_estimator_sam_e70/readme.html#pfc-software-design"
  },"45": {
    "doc": "PFC with PMSM FOC using PLL Estimator",
    "title": "Software Configuration:",
    "content": "The following section describes user specific parameters used the the project. | Setting the PFC boost converter stage frequency: The boost converter stage PWM frequency ( in Hz ) can be configured by setting the following macro in userparams.h file. This period timer ticks should be configured properly for PWM peripheral in the MHC to get the desired frequency as shown below: | . | Macro | Description | . | PFC_PWM_FREQUENCY | Boost converter stage PWM frequency | . |   |   | . | Setting PI Controller parameters of Boost Converter Controller: | . The voltage and current controllers used in the PFC algorithm has to be tuned to get the optimal control. Parameters for reference DC bus voltage PI Controller: . | Macro | Description | . | PFC_VOLTAGE_PTERM | Proportional gain of voltage control loop | . | PFC_VOLTAGE_ITERM | Integral gain of voltage control loop | . | PFC_VOLTAGE_CTERM | Anti-windup term of voltage control loop | . | PFC_VOLTAGE_OUTMAX | Maximum controller output of voltage control loop | . |   |   | . Parameters for reference rectified AC current PI Controller: . | Macro | Description | . | PFC_CURRCNTR_PTERM | Proportional gain of input current control loop | . | PFC_CURRCNTR_ITERM | Integral gain of input current control loop | . | PFC_CURRCNTR_CTERM | Anti-windup term of input current control loop | . | PFC_CURRCNTR_OUTMAX | Maximum controller output of input current control loop | . |   |   | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_pfc_foc_pll_estimator_sam_e70/readme.html#software-configuration",
    "relUrl": "/apps/pmsm_pfc_foc_pll_estimator_sam_e70/readme.html#software-configuration"
  },"46": {
    "doc": "PFC with PMSM FOC using PLL Estimator",
    "title": "Development Kits",
    "content": "MCHV3 with ATSAME70 PIM . Downloading and building the application . To clone or download this application from Github, go to the main page of this repository and then click Clone button to clone this repository or download as zip file. This content can also be downloaded using content manager by following these instructions. Path of the application within the repository is apps/pmsm_pfc_foc_pll_estimator_sam_e70 . To build the application, refer to the following table and open the project using its IDE. | Project Name | Description | Hardware Setup Guide | . | mchv3_sam_e70_pim.X | MPLABX project for MCHV3 board with ATSAME70 PIM | Hardware Setup and Running The Application on MCHV3 with ATSAME70 PIM | . |   |   |   | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_pfc_foc_pll_estimator_sam_e70/readme.html#development-kits",
    "relUrl": "/apps/pmsm_pfc_foc_pll_estimator_sam_e70/readme.html#development-kits"
  },"47": {
    "doc": "PFC with PMSM FOC using PLL Estimator",
    "title": "PFC with PMSM FOC using PLL Estimator",
    "content": ". ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_pfc_foc_pll_estimator_sam_e70/readme.html",
    "relUrl": "/apps/pmsm_pfc_foc_pll_estimator_sam_e70/readme.html"
  },"48": {
    "doc": "PMSM FOC using PLL Estimator",
    "title": "PMSM FOC using PLL Estimator",
    "content": "This example application shows how to control the Permanent Magnet Synchronous Motor (PMSM) with PLL Estimator based Field Oriented Control (FOC) on a SAME70 Micro-controller. ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/mcp_pmsm_foc_pll_estimator_sam_e70/readme.html#pmsm-foc-using-pll-estimator",
    "relUrl": "/apps/mcp_pmsm_foc_pll_estimator_sam_e70/readme.html#pmsm-foc-using-pll-estimator"
  },"49": {
    "doc": "PMSM FOC using PLL Estimator",
    "title": "Description",
    "content": "Permanent Magnet Synchronous Motor (PMSM) is controlled using Field Oriented Control (FOC). Rotor position and speed is determined using PLL estimator technique. Motor start/stop operation is controlled by the switch and motor speed can be changed by the on-board potentiometer. Waveforms and variables can be monitored runtime using X2CScope. Key features enabled in this project are: . | Dual shunt current measurement | Speed control loop | Field weakening | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/mcp_pmsm_foc_pll_estimator_sam_e70/readme.html#description",
    "relUrl": "/apps/mcp_pmsm_foc_pll_estimator_sam_e70/readme.html#description"
  },"50": {
    "doc": "PMSM FOC using PLL Estimator",
    "title": "MHC Project Configurations",
    "content": ". | AFEC Peripheral: . The AFEC is used to measure analog quantities. Four channels are used to measure the Phase Current U, the Phase Current V, the DC Bus Voltage and the Potentiometer. Conversion is triggered at the PWM (zero match + offset of the switch delay) . | PWM Peripheral: . This peripheral is used to generated three phase synchronous PWM waveforms. Fault functionality is also enabled to switch off the output waveforms asynchronously. | X2CScope: . This component adds X2C scope protocol code. This uses UART to communicate to the host PC. X2CScope allows user to monitor variables runtime. | UART Peripheral: . The UART is used for X2CScope communication to observe graphs and variable values in run time . | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/mcp_pmsm_foc_pll_estimator_sam_e70/readme.html#mhc-project-configurations",
    "relUrl": "/apps/mcp_pmsm_foc_pll_estimator_sam_e70/readme.html#mhc-project-configurations"
  },"51": {
    "doc": "PMSM FOC using PLL Estimator",
    "title": "Project Details",
    "content": "For details refer Motor Control Plant . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/mcp_pmsm_foc_pll_estimator_sam_e70/readme.html#project-details",
    "relUrl": "/apps/mcp_pmsm_foc_pll_estimator_sam_e70/readme.html#project-details"
  },"52": {
    "doc": "PMSM FOC using PLL Estimator",
    "title": "Development Kits",
    "content": "MCLV2 with ATSAME70 PIM . Downloading and building the application . To clone or download this application from Github, go to the main page of this repository and then click Clone button to clone this repository or download as zip file. This content can also be downloaded using content manager by following these instructions. Path of the application within the repository is apps/mcp_pmsm_foc_pll_estimator_sam_e70 . To build the application, refer to the following table and open the project using its IDE. | Project Name | Description | Hardware Setup Guide | . | mclv2_sam_e70_pim.X | MPLABX project for MCLV2 board with ATSAME70 PIM | Hardware Setup and Running The Application on MCLV2 with ATSAME70 PIM | . |   |   |   | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/mcp_pmsm_foc_pll_estimator_sam_e70/readme.html#development-kits",
    "relUrl": "/apps/mcp_pmsm_foc_pll_estimator_sam_e70/readme.html#development-kits"
  },"53": {
    "doc": "PMSM FOC using PLL Estimator",
    "title": "PMSM FOC using PLL Estimator",
    "content": ". ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/mcp_pmsm_foc_pll_estimator_sam_e70/readme.html",
    "relUrl": "/apps/mcp_pmsm_foc_pll_estimator_sam_e70/readme.html"
  },"54": {
    "doc": "PMSM FOC using Quadrature Encoder",
    "title": "PMSM FOC using Quadrature Encoder",
    "content": "This example application shows how to control the Permanent Magnet Synchronous Motor (PMSM) with Quadrature Encoder based Field Oriented Control (FOC) on a SAME70 Micro-controller. ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/mcp_pmsm_foc_encoder_sam_e70/readme.html#pmsm-foc-using-quadrature-encoder",
    "relUrl": "/apps/mcp_pmsm_foc_encoder_sam_e70/readme.html#pmsm-foc-using-quadrature-encoder"
  },"55": {
    "doc": "PMSM FOC using Quadrature Encoder",
    "title": "Description",
    "content": "Permanent Magnet Synchronous Motor (PMSM) is controlled using Field Oriented Control (FOC). Rotor position and speed is determined using quadrature encoder sensor. Motor start/stop operation is controlled by the switch and motor speed can be changed by the on-board potentiometer. Waveforms and variables can be monitored runtime using X2CScope. Key features enabled in this project are: . | Dual shunt current measurement | Speed control loop | Field weakening | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/mcp_pmsm_foc_encoder_sam_e70/readme.html#description",
    "relUrl": "/apps/mcp_pmsm_foc_encoder_sam_e70/readme.html#description"
  },"56": {
    "doc": "PMSM FOC using Quadrature Encoder",
    "title": "MHC Project Configurations",
    "content": ". | PMSM_FOC: . This component configures FOC algorithm parameters, motor parameters and motor control board parameters. It connects to underlying peripheral libraries AFEC and PWM. This components auto configures ADC channels and PWM channels as per PMSM_FOC component configurations. | AFEC Peripheral: . The AFEC is used to measure analog quantities. Four channels are used to measure the Phase Current U, the Phase Current V, the DC Bus Voltage and the Potentiometer. Conversion is triggered at the PWM (zero match + offset of the switch delay) . | PWM Peripheral: . This peripheral is used to generated three phase synchronous PWM waveforms. Fault functionality is also enabled to switch off the output waveforms asynchronously. | TC QDEC Peripheral: . TC peripheral is configured in QDEC mode. It is used to decode the rotor position and speed from quadrature encoder signals. | X2CScope: . This component adds X2C scope protocol code. This uses UART to communicate to the host PC. X2CScope allows user to monitor variables runtime. | UART Peripheral: . The UART is used for X2CScope communication to observe graphs and variable values in run time . | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/mcp_pmsm_foc_encoder_sam_e70/readme.html#mhc-project-configurations",
    "relUrl": "/apps/mcp_pmsm_foc_encoder_sam_e70/readme.html#mhc-project-configurations"
  },"57": {
    "doc": "PMSM FOC using Quadrature Encoder",
    "title": "Project Details",
    "content": "For details refer Motor Control Plant . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/mcp_pmsm_foc_encoder_sam_e70/readme.html#project-details",
    "relUrl": "/apps/mcp_pmsm_foc_encoder_sam_e70/readme.html#project-details"
  },"58": {
    "doc": "PMSM FOC using Quadrature Encoder",
    "title": "Development Kits",
    "content": "MCLV2 with ATSAME70 PIM . Downloading and building the application . To clone or download this application from Github, go to the main page of this repository and then click Clone button to clone this repository or download as zip file. This content can also be downloaded using content manager by following these instructions. Path of the application within the repository is apps/mcp_pmsm_foc_encoder_sam_e70 . To build the application, refer to the following table and open the project using its IDE. | Project Name | Description | Hardware Setup Guide | . | mclv2_sam_e70_pim.X | MPLABX project for MCLV2 board with ATSAME70 PIM | Hardware Setup and Running The Application on MCLV2 with ATSAME70 PIM | . |   |   |   | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/mcp_pmsm_foc_encoder_sam_e70/readme.html#development-kits",
    "relUrl": "/apps/mcp_pmsm_foc_encoder_sam_e70/readme.html#development-kits"
  },"59": {
    "doc": "PMSM FOC using Quadrature Encoder",
    "title": "PMSM FOC using Quadrature Encoder",
    "content": ". ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/apps/mcp_pmsm_foc_encoder_sam_e70/readme.html",
    "relUrl": "/apps/mcp_pmsm_foc_encoder_sam_e70/readme.html"
  },"60": {
    "doc": "Release notes",
    "title": "Microchip MPLAB® Harmony 3 Release Notes",
    "content": " ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/release_notes.html#microchip-mplab-harmony-3-release-notes",
    "relUrl": "/release_notes.html#microchip-mplab-harmony-3-release-notes"
  },"61": {
    "doc": "Release notes",
    "title": "Microchip MPLAB® Harmony 3 Motor Control Application Examples for SAM E7x/S7x/V7x family v3.2.0",
    "content": "Applications . | Development Board | Number of Applications | . | dsPICDEM™ MCLV-2 Low Voltage Development Board | 5 | . | dsPICDEM™ MCHV-3 High Voltage Development Board | 3 | . New Features . | Added Motor Control Plant generated motor control firmware for PLL based sensorless FOC for MCLV2 | Added Motor Control Plant generated motor control firmware for Encoder based sensored FOC for MCLV2 | Migrated MHC generated code to MCC generated code | . Known Issues . | Isolated EDBG Card . | The Isolated EDBG Card may appear “grayed out” (disabled) under the list of tools in MPLABX v5.50. In order to resolve this issue, please go to Tools -&gt; Options -&gt;Embedded-&gt;Generic Settings and enable “Exclude device checks for kits” by selecting the check box. | . | If programming failure occurs with message “java.lang.RuntimeException:RDDI_DAP_OPERATION_FAILED”, then reset the Isolated EDBG Card’s configuration by Go to File -&gt; Project Properties -&gt; EDBG -&gt; Reset | . | pmsm_foc_encoder_sam_e70_mchv3 application running on dsPICDEM MCHV-3 requires increasing bandwidth of the quadrature encoder signal filter to maintain signal integrity of quadrature sensor signals at higher motor speeds. Without these modifications, motor operation may fail at higher speeds. | Reduce the capacitance value of C25, C26 and C27 from 100pF to 10pF 50V NPO 0805 | . | . Required MPLAB Harmony v3 Modules . | csp v3.11.0 | x2c v1.4.0 | bsp v3.11.0 | motor_control v3.8.0 | dev_packs v3.11.0 | mcc v1.0.9 | . Development Tools . | MPLAB X IDE v6.00 | MPLAB XC32 C/C++ Compiler v4.00 | MPLAB X IDE plug-ins: . | MPLAB Code Configurator (MCC) v5.1.1 | X2CScope v1.3.0. | . | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/release_notes.html#microchip-mplab-harmony-3-motor-control-application-examples-for-sam-e7xs7xv7x-family-v320",
    "relUrl": "/release_notes.html#microchip-mplab-harmony-3-motor-control-application-examples-for-sam-e7xs7xv7x-family-v320"
  },"62": {
    "doc": "Release notes",
    "title": "Microchip MPLAB® Harmony 3 Motor Control Application Examples for SAM E7x/S7x/V7x family v3.1.0",
    "content": "Applications . | Development Board | Number of Applications | . | dsPICDEM™ MCLV-2 Low Voltage Development Board | 3 | . | dsPICDEM™ MCHV-3 High Voltage Development Board | 3 | . New Features . None . Known Issues . | Isolated EDBG Card . | The Isolated EDBG Card may appear “grayed out” (disabled) under the list of tools in MPLABX v5.50. In order to resolve this issue, please go to Tools -&gt; Options -&gt;Embedded-&gt;Generic Settings and enable “Exclude device checks for kits” by selecting the check box. | . | If programming failure occurs with message “java.lang.RuntimeException:RDDI_DAP_OPERATION_FAILED”, then reset the Isolated EDBG Card’s configuration by Go to File -&gt; Project Properties -&gt; EDBG -&gt; Reset | . | pmsm_foc_encoder_sam_e70_mchv3 application running on dsPICDEM MCHV-3 requires increasing bandwidth of the quadrature encoder signal filter to maintain signal integrity of quadrature sensor signals at higher motor speeds. Without these modifications, motor operation may fail at higher speeds. | Reduce the capacitance value of C25, C26 and C27 from 100pF to 10pF 50V NPO 0805 | . | . Required MPLAB Harmony v3 Modules . | csp v3.9.1 | x2c v1.1.4 | motor_control v3.7.0 | dev_packs v3.9.0 | mhc v3.8.0 | . Development Tools . | MPLAB X IDE v5.50 | MPLAB XC32 C/C++ Compiler v3.01 | MPLAB X IDE plug-ins: . | MPLAB Harmony Configurator (MHC) v3.6.4 | X2CScope v1.3.0. | . | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/release_notes.html#microchip-mplab-harmony-3-motor-control-application-examples-for-sam-e7xs7xv7x-family-v310",
    "relUrl": "/release_notes.html#microchip-mplab-harmony-3-motor-control-application-examples-for-sam-e7xs7xv7x-family-v310"
  },"63": {
    "doc": "Release notes",
    "title": "Microchip MPLAB® Harmony 3 Motor Control Application Examples for SAM E7x/S7x/V7x family v3.0.0",
    "content": "Applications . Applications migrated from motor_control repository to this application repository for SAME7x/S7x/V7x family. | Development Board | Number of Applications | . | dsPICDEM™ MCLV-2 Low Voltage Development Board | 3 | . | dsPICDEM™ MCHV-3 High Voltage Development Board | 3 | . Required MPLAB Harmony v3 Modules . | csp v3.8.3 | x2c v1.1.3 | motor_control v3.6.0 | dev_packs v3.8.0 | mhc v3.6.5 | . Known Issues . | Isolated EDBG Card . | The Isolated EDBG Card may appear “grayed out” (disabled) under the list of tools in MPLABX v5.45. In order to resolve this issue, please go to Tools -&gt; Options -&gt;Embedded-&gt;Generic Settings and enable “Exclude device checks for kits” by selecting the check box. | . | If programming failure occurs with message “java.lang.RuntimeException:RDDI_DAP_OPERATION_FAILED”, then reset the Isolated EDBG Card’s configuration by Go to File -&gt; Project Properties -&gt; EDBG -&gt; Reset | . | pmsm_foc_encoder_sam_e70_mchv3 application running on dsPICDEM MCHV-3 requires increasing bandwidth of the quadrature encoder signal filter to maintain signal integrity of quadrature sensor signals at higher motor speeds. Without these modifications, motor operation may fail at higher speeds. | Reduce the capacitance value of C25, C26 and C27 from 100pF to 10pF 50V NPO 0805 | . | . Development Tools . | MPLAB X IDE v5.45 | MPLAB XC32 C/C++ Compiler v2.50 | MPLAB X IDE plug-ins: . | MPLAB Harmony Configurator (MHC) v3.6.2 | X2CScope v1.3.0. | . | . ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/release_notes.html#microchip-mplab-harmony-3-motor-control-application-examples-for-sam-e7xs7xv7x-family-v300",
    "relUrl": "/release_notes.html#microchip-mplab-harmony-3-motor-control-application-examples-for-sam-e7xs7xv7x-family-v300"
  },"64": {
    "doc": "Release notes",
    "title": "Release notes",
    "content": ". ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/release_notes.html",
    "relUrl": "/release_notes.html"
  },"65": {
    "doc": "Microchip MPLAB® Harmony 3 Motor Control Application Examples for SAM E7x/S7x/V7x family",
    "title": "Microchip MPLAB® Harmony 3 Motor Control Application Examples for SAM E7x/S7x/V7x family",
    "content": "# Microchip MPLAB® Harmony 3 Motor Control Application Examples for SAM E7x/S7x/V7x family MPLAB Harmony 3 is an extension of the MPLAB® ecosystem for creating embedded firmware solutions for Microchip 32-bit SAM and PIC32 microcontroller and microprocessor devices. Refer to the following links for more information. - [Microchip 32-bit MCUs for Motor Control Applications](https://www.microchip.com/design-centers/motor-control-and-drive/control-products/32-bit-solutions) - [Microchip 32-bit MCUs](https://www.microchip.com/design-centers/32-bit) - [Microchip 32-bit MPUs](https://www.microchip.com/design-centers/32-bit-mpus) - [Microchip MPLAB X IDE](https://www.microchip.com/mplab/mplab-x-ide) - [Microchip MPLAB Harmony](https://www.microchip.com/mplab/mplab-harmony) - [Microchip MPLAB Harmony Pages](https://microchip-mplab-harmony.github.io/) This repository contains the MPLAB® Harmony 3 Motor Control application exmaples for SAME7x/S7x/V7x family. Users can use these examples as a reference for developing their own motor control applications. Refer to the following links for release notes and licensing information. - [Release Notes](/mc_apps_sam_e7x_s7x_v7x/release_notes.html) - [MPLAB Harmony License](/mc_apps_sam_e7x_s7x_v7x/mplab_harmony_license.html) ## Contents Summary | Folder | Description |------------|-----------------------------------------------------------| apps | Demonstration applications for Motor Control | docs | Contains documentation in html format for offline viewing (to be used only after cloning this repository onto a local machine). Use [github pages](https://microchip-mplab-harmony.github.io/mc_apps_sam_e7x_s7x_v7x/) of this repository for viewing it online. ||| ## Configurable Motor Control Examples (MCC with Motor Control Plant) The following applications are provided to demonstrate the Motor Control Plant to generate the motor control firmware. | Name | Description|Control Board|Inverter Board|:-----|:-----------|:------------|:-------------| [PMSM FOC using PLL Estimator ](/mc_apps_sam_e7x_s7x_v7x/apps/mcp_pmsm_foc_pll_estimator_sam_e70/readme.html) | Sensorless Field Oriented Control of PMSM using PLL Estimator | [ATSAME70 Motor Control Plugin Module](https://www.microchip.com/Developmenttools/ProductDetails/MA320203) | [dsPICDEM™ MCLV-2 Support](https://www.microchip.com/DevelopmentTools/ProductDetails/DM330021-2) [dsPICDEM™ MCHV-3 Support](https://www.microchip.com/developmenttools/ProductDetails/dm330023-3) | [PMSM FOC using Quadrature Encoder](/mc_apps_sam_e7x_s7x_v7x/apps/mcp_pmsm_foc_encoder_sam_e70/readme.html) | Sensored Field Oriented Control of PMSM using Quadrature Encoder |[ATSAME70 Motor Control Plugin Module](https://www.microchip.com/Developmenttools/ProductDetails/MA320203)| [dsPICDEM™ MCLV-2 Support](https://www.microchip.com/DevelopmentTools/ProductDetails/DM330021-2) [dsPICDEM™ MCHV-3 Support](https://www.microchip.com/developmenttools/ProductDetails/dm330023-3) ||||| ## Static Motor Control Examples These applications contain static algorithm code and peripherals are configured using MHC. Configurations can be changed in userparam.h file. | Name | Description|Control Board|Inverter Board|:-----|:-----------|:------------|:-------------| [PMSM FOC using PLL Estimator](/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_pll_estimator_sam_e70/readme.html) | Sensorless Field Oriented Control of PMSM using PLL Estimator | [ATSAME70 Motor Control Plugin Module](https://www.microchip.com/Developmenttools/ProductDetails/MA320203) | [dsPICDEM™ MCLV-2 Support](https://www.microchip.com/DevelopmentTools/ProductDetails/DM330021-2) [dsPICDEM™ MCHV-3 Support](https://www.microchip.com/developmenttools/ProductDetails/dm330023-3) | [PMSM FOC using Quadrature Encoder](/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_encoder_sam_e70/readme.html) | Sensored Field Oriented Control of PMSM using Quadrature Encoder |[ATSAME70 Motor Control Plugin Module](https://www.microchip.com/Developmenttools/ProductDetails/MA320203)| [dsPICDEM™ MCLV-2 Support](https://www.microchip.com/DevelopmentTools/ProductDetails/DM330021-2) [dsPICDEM™ MCHV-3 Support](https://www.microchip.com/developmenttools/ProductDetails/dm330023-3) ||||| [PFC and PMSM FOC using PLL Estimator](/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_pfc_foc_pll_estimator_sam_e70/readme.html) | Power Factor Correction and Sensorless Field Oriented Control of PMSM using PLL Estimator |[ATSAME70 Motor Control Plugin Module](https://www.microchip.com/Developmenttools/ProductDetails/MA320203) |[dsPICDEM™ MCHV-3 Support](https://www.microchip.com/developmenttools/ProductDetails/dm330023-3) | [PMSM FOC using Sliding Mode Observer](/mc_apps_sam_e7x_s7x_v7x/apps/pmsm_foc_smo_sam_e70/readme.html) | Sensorless Field Oriented Control of PMSM using Sliding Mode Observer |[ATSAME70 Motor Control Plugin Module](https://www.microchip.com/Developmenttools/ProductDetails/MA320203) | [dsPICDEM™ MCLV-2 Support](https://www.microchip.com/DevelopmentTools/ProductDetails/DM330021-2) ||||| [![License](https://img.shields.io/badge/license-Harmony%20license-orange.svg)](https://github.com/Microchip-MPLAB-Harmony/mc/blob/master/mplab_harmony_license.md) [![Latest release](https://img.shields.io/github/release/Microchip-MPLAB-Harmony/mc.svg)](https://github.com/Microchip-MPLAB-Harmony/mc/releases/latest) [![Latest release date](https://img.shields.io/github/release-date/Microchip-MPLAB-Harmony/mc.svg)](https://github.com/Microchip-MPLAB-Harmony/mc/releases/latest) [![Commit activity](https://img.shields.io/github/commit-activity/y/Microchip-MPLAB-Harmony/mc.svg)](https://github.com/Microchip-MPLAB-Harmony/mc/graphs/commit-activity) [![Contributors](https://img.shields.io/github/contributors-anon/Microchip-MPLAB-Harmony/mc.svg)]() ____ [![Follow us on Youtube](https://img.shields.io/badge/Youtube-Follow%20us%20on%20Youtube-red.svg)](https://www.youtube.com/user/MicrochipTechnology) [![Follow us on LinkedIn](https://img.shields.io/badge/LinkedIn-Follow%20us%20on%20LinkedIn-blue.svg)](https://www.linkedin.com/company/microchip-technology) [![Follow us on Facebook](https://img.shields.io/badge/Facebook-Follow%20us%20on%20Facebook-blue.svg)](https://www.facebook.com/microchiptechnology/) [![Follow us on Twitter](https://img.shields.io/twitter/follow/MicrochipTech.svg?style=social)](https://twitter.com/MicrochipTech) [![](https://img.shields.io/github/stars/Microchip-MPLAB-Harmony/mc.svg?style=social)]() [![](https://img.shields.io/github/watchers/Microchip-MPLAB-Harmony/mc.svg?style=social)]() ",
    "url": "http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/",
    "relUrl": "/"
  }
}
`;
var data_for_search

var repo_name = "mc_apps_sam_e7x_s7x_v7x";
var doc_folder_name = "docs";
var localhost_path = "http://localhost:4000/";
var home_index_string = "Microchip MPLAB® Harmony 3 Motor Control Application Examples for SAM E7x/S7x/V7x family";

(function (jtd, undefined) {

// Event handling

jtd.addEvent = function(el, type, handler) {
  if (el.attachEvent) el.attachEvent('on'+type, handler); else el.addEventListener(type, handler);
}
jtd.removeEvent = function(el, type, handler) {
  if (el.detachEvent) el.detachEvent('on'+type, handler); else el.removeEventListener(type, handler);
}
jtd.onReady = function(ready) {
  // in case the document is already rendered
  if (document.readyState!='loading') ready();
  // modern browsers
  else if (document.addEventListener) document.addEventListener('DOMContentLoaded', ready);
  // IE <= 8
  else document.attachEvent('onreadystatechange', function(){
      if (document.readyState=='complete') ready();
  });
}

// Show/hide mobile menu

function initNav() {
  jtd.addEvent(document, 'click', function(e){
    var target = e.target;
    while (target && !(target.classList && target.classList.contains('nav-list-expander'))) {
      target = target.parentNode;
    }
    if (target) {
      e.preventDefault();
      target.parentNode.classList.toggle('active');
    }
  });

  const siteNav = document.getElementById('site-nav');
  const mainHeader = document.getElementById('main-header');
  const menuButton = document.getElementById('menu-button');

  jtd.addEvent(menuButton, 'click', function(e){
    e.preventDefault();

    if (menuButton.classList.toggle('nav-open')) {
      siteNav.classList.add('nav-open');
      mainHeader.classList.add('nav-open');
    } else {
      siteNav.classList.remove('nav-open');
      mainHeader.classList.remove('nav-open');
    }
  });
}
// Site search

function initSearch() {

    data_for_search = JSON.parse(myVariable);
    lunr.tokenizer.separator = /[\s/]+/

    var index = lunr(function () {
        this.ref('id');
        this.field('title', { boost: 200 });
        this.field('content', { boost: 2 });
        this.field('url');
        this.metadataWhitelist = ['position']

        var location = document.location.pathname;
        var path = location.substring(0, location.lastIndexOf("/"));
        var directoryName = path.substring(path.lastIndexOf("/")+1);

        var cur_path_from_repo = path.substring(path.lastIndexOf(repo_name));

        // Decrement depth by 2 as HTML files are placed in repo_name/doc_folder_name
        var cur_depth_from_doc_folder = (cur_path_from_repo.split("/").length - 2);

        var rel_path_to_doc_folder = "";

        if (cur_depth_from_doc_folder == 0) {
            rel_path_to_doc_folder = "./"
        }
        else {
            for (var i = 0; i < cur_depth_from_doc_folder; i++)
            {
                rel_path_to_doc_folder = rel_path_to_doc_folder + "../"
            }
        }

        for (var i in data_for_search) {

            data_for_search[i].url = data_for_search[i].url.replace(localhost_path + repo_name, rel_path_to_doc_folder);

            if (data_for_search[i].title == home_index_string)
            {
                data_for_search[i].url = data_for_search[i].url + "index.html"
            }

            this.add({
                id: i,
                title: data_for_search[i].title,
                content: data_for_search[i].content,
                url: data_for_search[i].url
            });
        }
    });

    searchLoaded(index, data_for_search);
}function searchLoaded(index, docs) {
  var index = index;
  var docs = docs;
  var searchInput = document.getElementById('search-input');
  var searchResults = document.getElementById('search-results');
  var mainHeader = document.getElementById('main-header');
  var currentInput;
  var currentSearchIndex = 0;

  function showSearch() {
    document.documentElement.classList.add('search-active');
  }

  function hideSearch() {
    document.documentElement.classList.remove('search-active');
  }

  function update() {
    currentSearchIndex++;

    var input = searchInput.value;
    if (input === '') {
      hideSearch();
    } else {
      showSearch();
      // scroll search input into view, workaround for iOS Safari
      window.scroll(0, -1);
      setTimeout(function(){ window.scroll(0, 0); }, 0);
    }
    if (input === currentInput) {
      return;
    }
    currentInput = input;
    searchResults.innerHTML = '';
    if (input === '') {
      return;
    }

    var results = index.query(function (query) {
      var tokens = lunr.tokenizer(input)
      query.term(tokens, {
        boost: 10
      });
      query.term(tokens, {
        wildcard: lunr.Query.wildcard.TRAILING
      });
    });

    if ((results.length == 0) && (input.length > 2)) {
      var tokens = lunr.tokenizer(input).filter(function(token, i) {
        return token.str.length < 20;
      })
      if (tokens.length > 0) {
        results = index.query(function (query) {
          query.term(tokens, {
            editDistance: Math.round(Math.sqrt(input.length / 2 - 1))
          });
        });
      }
    }

    if (results.length == 0) {
      var noResultsDiv = document.createElement('div');
      noResultsDiv.classList.add('search-no-result');
      noResultsDiv.innerText = 'No results found';
      searchResults.appendChild(noResultsDiv);

    } else {
      var resultsList = document.createElement('ul');
      resultsList.classList.add('search-results-list');
      searchResults.appendChild(resultsList);

      addResults(resultsList, results, 0, 10, 100, currentSearchIndex);
    }

    function addResults(resultsList, results, start, batchSize, batchMillis, searchIndex) {
      if (searchIndex != currentSearchIndex) {
        return;
      }
      for (var i = start; i < (start + batchSize); i++) {
        if (i == results.length) {
          return;
        }
        addResult(resultsList, results[i]);
      }
      setTimeout(function() {
        addResults(resultsList, results, start + batchSize, batchSize, batchMillis, searchIndex);
      }, batchMillis);
    }

    function addResult(resultsList, result) {
      var doc = docs[result.ref];

      var resultsListItem = document.createElement('li');
      resultsListItem.classList.add('search-results-list-item');
      resultsList.appendChild(resultsListItem);

      var resultLink = document.createElement('a');
      resultLink.classList.add('search-result');
      resultLink.setAttribute('href', doc.url);
      resultsListItem.appendChild(resultLink);

      var resultTitle = document.createElement('div');
      resultTitle.classList.add('search-result-title');
      resultLink.appendChild(resultTitle);

      var resultDoc = document.createElement('div');
      resultDoc.classList.add('search-result-doc');
      resultDoc.innerHTML = '<svg viewBox="0 0 24 24" class="search-result-icon"><use xlink:href="#svg-doc"></use></svg>';
      resultTitle.appendChild(resultDoc);

      var resultDocTitle = document.createElement('div');
      resultDocTitle.classList.add('search-result-doc-title');
      resultDocTitle.innerHTML = doc.doc;
      resultDoc.appendChild(resultDocTitle);
      var resultDocOrSection = resultDocTitle;

      if (doc.doc != doc.title) {
        resultDoc.classList.add('search-result-doc-parent');
        var resultSection = document.createElement('div');
        resultSection.classList.add('search-result-section');
        resultSection.innerHTML = doc.title;
        resultTitle.appendChild(resultSection);
        resultDocOrSection = resultSection;
      }

      var metadata = result.matchData.metadata;
      var titlePositions = [];
      var contentPositions = [];
      for (var j in metadata) {
        var meta = metadata[j];
        if (meta.title) {
          var positions = meta.title.position;
          for (var k in positions) {
            titlePositions.push(positions[k]);
          }
        }
        if (meta.content) {
          var positions = meta.content.position;
          for (var k in positions) {
            var position = positions[k];
            var previewStart = position[0];
            var previewEnd = position[0] + position[1];
            var ellipsesBefore = true;
            var ellipsesAfter = true;
            for (var k = 0; k < 5; k++) {
              var nextSpace = doc.content.lastIndexOf(' ', previewStart - 2);
              var nextDot = doc.content.lastIndexOf('. ', previewStart - 2);
              if ((nextDot >= 0) && (nextDot > nextSpace)) {
                previewStart = nextDot + 1;
                ellipsesBefore = false;
                break;
              }
              if (nextSpace < 0) {
                previewStart = 0;
                ellipsesBefore = false;
                break;
              }
              previewStart = nextSpace + 1;
            }
            for (var k = 0; k < 10; k++) {
              var nextSpace = doc.content.indexOf(' ', previewEnd + 1);
              var nextDot = doc.content.indexOf('. ', previewEnd + 1);
              if ((nextDot >= 0) && (nextDot < nextSpace)) {
                previewEnd = nextDot;
                ellipsesAfter = false;
                break;
              }
              if (nextSpace < 0) {
                previewEnd = doc.content.length;
                ellipsesAfter = false;
                break;
              }
              previewEnd = nextSpace;
            }
            contentPositions.push({
              highlight: position,
              previewStart: previewStart, previewEnd: previewEnd,
              ellipsesBefore: ellipsesBefore, ellipsesAfter: ellipsesAfter
            });
          }
        }
      }

      if (titlePositions.length > 0) {
        titlePositions.sort(function(p1, p2){ return p1[0] - p2[0] });
        resultDocOrSection.innerHTML = '';
        addHighlightedText(resultDocOrSection, doc.title, 0, doc.title.length, titlePositions);
      }

      if (contentPositions.length > 0) {
        contentPositions.sort(function(p1, p2){ return p1.highlight[0] - p2.highlight[0] });
        var contentPosition = contentPositions[0];
        var previewPosition = {
          highlight: [contentPosition.highlight],
          previewStart: contentPosition.previewStart, previewEnd: contentPosition.previewEnd,
          ellipsesBefore: contentPosition.ellipsesBefore, ellipsesAfter: contentPosition.ellipsesAfter
        };
        var previewPositions = [previewPosition];
        for (var j = 1; j < contentPositions.length; j++) {
          contentPosition = contentPositions[j];
          if (previewPosition.previewEnd < contentPosition.previewStart) {
            previewPosition = {
              highlight: [contentPosition.highlight],
              previewStart: contentPosition.previewStart, previewEnd: contentPosition.previewEnd,
              ellipsesBefore: contentPosition.ellipsesBefore, ellipsesAfter: contentPosition.ellipsesAfter
            }
            previewPositions.push(previewPosition);
          } else {
            previewPosition.highlight.push(contentPosition.highlight);
            previewPosition.previewEnd = contentPosition.previewEnd;
            previewPosition.ellipsesAfter = contentPosition.ellipsesAfter;
          }
        }

        var resultPreviews = document.createElement('div');
        resultPreviews.classList.add('search-result-previews');
        resultLink.appendChild(resultPreviews);

        var content = doc.content;
        for (var j = 0; j < Math.min(previewPositions.length, 3); j++) {
          var position = previewPositions[j];

          var resultPreview = document.createElement('div');
          resultPreview.classList.add('search-result-preview');
          resultPreviews.appendChild(resultPreview);

          if (position.ellipsesBefore) {
            resultPreview.appendChild(document.createTextNode('... '));
          }
          addHighlightedText(resultPreview, content, position.previewStart, position.previewEnd, position.highlight);
          if (position.ellipsesAfter) {
            resultPreview.appendChild(document.createTextNode(' ...'));
          }
        }
      }
      var resultRelUrl = document.createElement('span');
      resultRelUrl.classList.add('search-result-rel-url');
      resultRelUrl.innerText = doc.relUrl;
      resultTitle.appendChild(resultRelUrl);
    }

    function addHighlightedText(parent, text, start, end, positions) {
      var index = start;
      for (var i in positions) {
        var position = positions[i];
        var span = document.createElement('span');
        span.innerHTML = text.substring(index, position[0]);
        parent.appendChild(span);
        index = position[0] + position[1];
        var highlight = document.createElement('span');
        highlight.classList.add('search-result-highlight');
        highlight.innerHTML = text.substring(position[0], index);
        parent.appendChild(highlight);
      }
      var span = document.createElement('span');
      span.innerHTML = text.substring(index, end);
      parent.appendChild(span);
    }
  }

  jtd.addEvent(searchInput, 'focus', function(){
    setTimeout(update, 0);
  });

  jtd.addEvent(searchInput, 'keyup', function(e){
    switch (e.keyCode) {
      case 27: // When esc key is pressed, hide the results and clear the field
        searchInput.value = '';
        break;
      case 38: // arrow up
      case 40: // arrow down
      case 13: // enter
        e.preventDefault();
        return;
    }
    update();
  });

  jtd.addEvent(searchInput, 'keydown', function(e){
    switch (e.keyCode) {
      case 38: // arrow up
        e.preventDefault();
        var active = document.querySelector('.search-result.active');
        if (active) {
          active.classList.remove('active');
          if (active.parentElement.previousSibling) {
            var previous = active.parentElement.previousSibling.querySelector('.search-result');
            previous.classList.add('active');
          }
        }
        return;
      case 40: // arrow down
        e.preventDefault();
        var active = document.querySelector('.search-result.active');
        if (active) {
          if (active.parentElement.nextSibling) {
            var next = active.parentElement.nextSibling.querySelector('.search-result');
            active.classList.remove('active');
            next.classList.add('active');
          }
        } else {
          var next = document.querySelector('.search-result');
          if (next) {
            next.classList.add('active');
          }
        }
        return;
      case 13: // enter
        e.preventDefault();
        var active = document.querySelector('.search-result.active');
        if (active) {
          active.click();
        } else {
          var first = document.querySelector('.search-result');
          if (first) {
            first.click();
          }
        }
        return;
    }
  });

  jtd.addEvent(document, 'click', function(e){
    if (e.target != searchInput) {
      hideSearch();
    }
  });
}

// Switch theme

jtd.getTheme = function() {
  var cssFileHref = document.querySelector('[rel="stylesheet"]').getAttribute('href');
  return cssFileHref.substring(cssFileHref.lastIndexOf('-') + 1, cssFileHref.length - 4);
}

jtd.setTheme = function(theme) {
  var cssFile = document.querySelector('[rel="stylesheet"]');
  cssFile.setAttribute('href', 'http://localhost:4000/mc_apps_sam_e7x_s7x_v7x/assets/css/just-the-docs-' + theme + '.css');
}

// Document ready

jtd.onReady(function(){
  initNav();
  initSearch();
});

})(window.jtd = window.jtd || {});


