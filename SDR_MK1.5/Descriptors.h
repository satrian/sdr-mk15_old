/*
             LUFA Library
     Copyright (C) Dean Camera, 2010.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2010  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Header file for Descriptors.c.
 */

#ifndef _DESCRIPTORS_H_
#define _DESCRIPTORS_H_

	/* Includes: */
		#include <LUFA/Common/Common.h>
		#include <LUFA/Drivers/USB/USB.h>

	/* Macros: */

	/* VirtualSerial */
		/** Endpoint number of the first CDC interface's device-to-host data IN endpoint. */
		#define CDC1_TX_EPNUM					1

		/** Endpoint number of the first CDC interface's host-to-device data OUT endpoint. */
		#define CDC1_RX_EPNUM					2

		/** Endpoint number of the first CDC interface's device-to-host notification IN endpoint. */
		#define CDC1_NOTIFICATION_EPNUM			3

		/** Endpoint number of the first CDC interface's device-to-host data IN endpoint. */
		#define CDC2_TX_EPNUM					4

		/** Endpoint number of the first CDC interface's host-to-device data OUT endpoint. */
		#define CDC2_RX_EPNUM					7		// this is a fake endpoint, as we do not use it and do not have any endpoints to spare either!

		/** Endpoint number of the first CDC interface's device-to-host notification IN endpoint. */
		#define CDC2_NOTIFICATION_EPNUM			7		// this is a fake endpoint, as we do not use it and do not have any endpoints to spare either!

/* AudioInput */
		/** Endpoint number of the Audio isochronous streaming data endpoint. */
		#define AUDIO_STREAM_A_EPNUM           	5
		#define AUDIO_STREAM_B_EPNUM          	6



		/** Endpoint number of the second CDC interface's device-to-host notification IN endpoint. */
		//#define CDC2_NOTIFICATION_EPNUM        (4+AUDIO_STREAM_A_EPNUM)

		/** Endpoint number of the second CDC interface's device-to-host data IN endpoint. */
		//#define CDC2_TX_EPNUM                  (5+AUDIO_STREAM_A_EPNUM)

		/** Endpoint number of the second CDC interface's host-to-device data OUT endpoint. */
		//#define CDC2_RX_EPNUM                  (6+AUDIO_STREAM_A_EPNUM)

		/** Size in bytes of the CDC device-to-host notification IN endpoints. */
		#define CDC1_NOTIFICATION_EPSIZE        8

		/** Size in bytes of the CDC data IN and OUT endpoints. */
		#define CDC1_TX_EPSIZE                16	//16
		#define CDC1_RX_EPSIZE                16	//64

		/** Size in bytes of the CDC device-to-host notification IN endpoints. */
		#define CDC2_NOTIFICATION_EPSIZE        8

		/** Size in bytes of the CDC data IN and OUT endpoints. */
		#define CDC2_TX_EPSIZE                64	//16
		#define CDC2_RX_EPSIZE                16	//64


		// ****** these are not an endpoint numbers but, but interface numbers inside composite device ******

		#define SER_FIRST						0

		#define AUDIO_STREAM_A_INTERFACE		(SER_FIRST+4)
		#define AUDIO_STREAM_B_INTERFACE		(SER_FIRST+2)

		#define SER_SECOND					SER_FIRST+6

		// ******

		/** Endpoint size in bytes of the Audio isochronous streaming data endpoint. The Windows audio stack requires
		 *  at least 192 bytes for correct output, thus the smaller 128 byte maximum endpoint size on some of the smaller
		 *  USB AVR models will result in unavoidable distorted output.
		 */
		#define AUDIO_STREAM_A_EPSIZE          ENDPOINT_MAX_SIZE(AUDIO_STREAM_A_EPNUM)
		#define AUDIO_STREAM_B_EPSIZE          ENDPOINT_MAX_SIZE(AUDIO_STREAM_B_EPNUM)

		// This is the rate IRQ will be called and new samples will be ready
		#define SDR_SAMPLE_FREQUENCY	       48000L	//48000	//88200 96000 176400 192000 48000

		// The dividers will be calculated based on this setting. NOTE!, that channel B endpoint size is only 64bytes, what
		// allows only 12kHz sample rate in stereo. Now, if we have SDR working with 48kHz for example, and channel B working with
		// 12kHz, downconversion will happen during sampling, what means that data has to be filtered to 1/4 bandwidth before that.

		/** Sample frequency of the data being transmitted through the streaming endpoint. */
		#define AUDIO_SAMPLE_FREQUENCY_A       	SDR_SAMPLE_FREQUENCY	//88200 96000 176400 192000 48000
		#define AUDIO_SAMPLE_FREQUENCY_B       	SDR_SAMPLE_FREQUENCY	//96000	//12000L

		#define AUDIO_SUPPORTEDRATES_A			1
		#define AUDIO_SUPPORTEDRATES_B			1

		//#define CH_A_DIVIDER					(SDR_SAMPLE_FREQUENCY/AUDIO_SAMPLE_FREQUENCY_A)
		//#define CH_B_DIVIDER					(SDR_SAMPLE_FREQUENCY/AUDIO_SAMPLE_FREQUENCY_B)

		//
		// channel A is normally the low-speed channel in UAC mode (48kHz 16-bit stereo) while channel B is the bulk mode
		// high-speed channel (96kHz 16-bit stereo). However, in case the SDR is running on 48kHz, we are transforming the channel B
		// to a low-speed 24kHz 16-bit mono channel!
		//

		#define AUDIOCHANNELS_A					2
		#define AUDIOCHANNELCFG_A				(AUDIO_CHANNEL_LEFT_FRONT | AUDIO_CHANNEL_RIGHT_FRONT)
		#define AUDIOSUBFRAMESIZE_A				2
		#define AUDIOATTRIBUTES_A				(EP_TYPE_ISOCHRONOUS | ENDPOINT_ATTR_ASYNC | ENDPOINT_USAGE_DATA)		//=0x5
		//#define AUDIOATTRIBUTES_B				(EP_TYPE_ISOCHRONOUS | ENDPOINT_ATTR_SYNC | ENDPOINT_USAGE_DATA)		//=0xD
		//#define AUDIOATTRIBUTES_A				(EP_TYPE_ISOCHRONOUS | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA)		//=0x1

		#define AUDIOCHANNELS_B					2
		#define AUDIOCHANNELCFG_B				(AUDIO_CHANNEL_LEFT_FRONT | AUDIO_CHANNEL_RIGHT_FRONT)
		#define AUDIOSUBFRAMESIZE_B				2
		#define AUDIOATTRIBUTES_B				(EP_TYPE_ISOCHRONOUS | ENDPOINT_ATTR_ASYNC | ENDPOINT_USAGE_DATA)		//=0x5
		//#define AUDIOATTRIBUTES_B				(EP_TYPE_ISOCHRONOUS | ENDPOINT_ATTR_SYNC | ENDPOINT_USAGE_DATA)		//=0xD
		//#define AUDIOATTRIBUTES_B				(EP_TYPE_ISOCHRONOUS | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA)		//=0x1

		#define AUDIOBITS 						16

		// 0x1007 radio receiver
		// 0x603 Line-in at std levels
		#define TERMINALTYPE_A					0x603	// was: AUDIO_TERMINAL_IN_MIC
		#define TERMINALTYPE_B					0x603 //0x1007	// was: AUDIO_TERMINAL_IN_MIC


	/* Type Defines: */
		/** Type define for the device configuration descriptor structure. This must be defined in the
		 *  application code, as the configuration descriptor contains several sub-descriptors which
		 *  vary between devices, and which describe the device's usage to the host.
		 */
		typedef struct
		{
			USB_Descriptor_Configuration_Header_t     Config;

			USB_Descriptor_Interface_Association_t   CDC1_IAD;
			USB_Descriptor_Interface_t               CDC1_CCI_Interface;
			USB_CDC_Descriptor_FunctionalHeader_t    CDC1_Functional_Header;
			USB_CDC_Descriptor_FunctionalACM_t       CDC1_Functional_ACM;
			USB_CDC_Descriptor_FunctionalUnion_t     CDC1_Functional_Union;
			USB_Descriptor_Endpoint_t                CDC1_ManagementEndpoint;
			USB_Descriptor_Interface_t               CDC1_DCI_Interface;
			USB_Descriptor_Endpoint_t                CDC1_DataOutEndpoint;
			USB_Descriptor_Endpoint_t                CDC1_DataInEndpoint;

			USB_Descriptor_Interface_Association_t   CDC2_IAD;
			USB_Descriptor_Interface_t               CDC2_CCI_Interface;
			USB_CDC_Descriptor_FunctionalHeader_t    CDC2_Functional_Header;
			USB_CDC_Descriptor_FunctionalACM_t       CDC2_Functional_ACM;
			USB_CDC_Descriptor_FunctionalUnion_t     CDC2_Functional_Union;
			USB_Descriptor_Endpoint_t                CDC2_ManagementEndpoint;
			USB_Descriptor_Interface_t               CDC2_DCI_Interface;
			USB_Descriptor_Endpoint_t                CDC2_DataOutEndpoint;
			USB_Descriptor_Endpoint_t                CDC2_DataInEndpoint;

			USB_Descriptor_Interface_Association_t    Audio_IAD;
			USB_Descriptor_Interface_t                Audio_ControlInterface;
			USB_Audio_Descriptor_Interface_AC_t       Audio_ControlInterface_SPC;
			USB_Audio_Descriptor_InputTerminal_t      Audio_InputTerminal;
			USB_Audio_Descriptor_OutputTerminal_t     Audio_OutputTerminal;
			USB_Descriptor_Interface_t                Audio_StreamInterface_Alt0;
			USB_Descriptor_Interface_t                Audio_StreamInterface_Alt1;
			USB_Audio_Descriptor_Interface_AS_t       Audio_StreamInterface_SPC;
			USB_Audio_Descriptor_Format_t             Audio_AudioFormat;
			USB_Audio_SampleFreq_t                    Audio_AudioFormatSampleRates[AUDIO_SUPPORTEDRATES_A];
			USB_Audio_Descriptor_StreamEndpoint_Std_t Audio_StreamEndpoint;
			USB_Audio_Descriptor_StreamEndpoint_Spc_t Audio_StreamEndpoint_SPC;

			USB_Descriptor_Interface_Association_t    Audio2_IAD;
			USB_Descriptor_Interface_t                Audio2_ControlInterface;
			USB_Audio_Descriptor_Interface_AC_t       Audio2_ControlInterface_SPC;
			USB_Audio_Descriptor_InputTerminal_t      Audio2_InputTerminal;
			USB_Audio_Descriptor_OutputTerminal_t     Audio2_OutputTerminal;
			USB_Descriptor_Interface_t                Audio2_StreamInterface_Alt0;
			USB_Descriptor_Interface_t                Audio2_StreamInterface_Alt1;
			USB_Audio_Descriptor_Interface_AS_t       Audio2_StreamInterface_SPC;
			USB_Audio_Descriptor_Format_t             Audio2_AudioFormat;
			USB_Audio_SampleFreq_t                    Audio2_AudioFormatSampleRates[AUDIO_SUPPORTEDRATES_B];
			USB_Audio_Descriptor_StreamEndpoint_Std_t Audio2_StreamEndpoint;
			USB_Audio_Descriptor_StreamEndpoint_Spc_t Audio2_StreamEndpoint_SPC;

		} USB_Descriptor_Configuration_t;

	/* Function Prototypes: */
		uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
		                                    const uint8_t wIndex,
		                                    const void** const DescriptorAddress)
		                                    ATTR_WARN_UNUSED_RESULT ATTR_NON_NULL_PTR_ARG(3);

#endif

