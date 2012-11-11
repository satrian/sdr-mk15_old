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
 *  USB Device Descriptors, for library use when in USB device mode. Descriptors are special
 *  computer-readable structures which the host requests upon device enumeration, to determine
 *  the device's capabilities and functions.
 */

#include "Descriptors.h"

extern bool external_power;			// dependent on this flag different descriptor table is loaded

/** Device descriptor structure. This descriptor, located in FLASH memory, describes the overall
 *  device characteristics, including the supported USB version, control endpoint size and the
 *  number of device configurations. The descriptor is read out by the USB host when the enumeration
 *  process begins.
 */
USB_Descriptor_Device_t PROGMEM DeviceDescriptor =
{
	.Header                 = {.Size = sizeof(USB_Descriptor_Device_t), .Type = DTYPE_Device},

	//.USBSpecification       = VERSION_BCD(01.10),
	.USBSpecification       = VERSION_BCD(02.00),
	 /*
	.Class                  = USB_CSCP_NoDeviceClass,
	.SubClass               = USB_CSCP_NoDeviceSubclass,
	.Protocol               = USB_CSCP_NoDeviceProtocol,
	 */
	.Class                  = USB_CSCP_IADDeviceClass,
	.SubClass               = USB_CSCP_IADDeviceSubclass,
	.Protocol               = USB_CSCP_IADDeviceProtocol,

	.Endpoint0Size          = FIXED_CONTROL_ENDPOINT_SIZE,

	.VendorID               = CPU_TO_LE16(0x03EB),
	.ProductID              = CPU_TO_LE16(0x204E),	//serial
	//.ProductID              = 0x2047,	//audio
	.ReleaseNumber          = VERSION_BCD(00.01),
	//.ReleaseNumber          = VERSION_BCD(00.02),

	.ManufacturerStrIndex   = 0x01,
	.ProductStrIndex        = 0x02,
	//.SerialNumStrIndex      = NO_DESCRIPTOR,
	.SerialNumStrIndex      = USE_INTERNAL_SERIAL,		// give serial number to avoid device having new COM port number when connected to different USB ports on computer (unwanted "COM-port proliferation.")

	.NumberOfConfigurations = FIXED_NUM_CONFIGURATIONS
};


/** Configuration descriptor structure. This descriptor, located in FLASH memory, describes the usage
 *  of the device in one of its supported configurations, including information about any device interfaces
 *  and endpoints. The descriptor is read out by the USB host during the enumeration process when selecting
 *  a configuration so that the host may correctly communicate with the USB device.
 */
USB_Descriptor_Configuration_t ConfigurationDescriptor =		// not using PROGMEM (what evaluates as const for 32-bit platform), as we need to modify descriptor if external power is used!
{
	.Config =
		{
			.Header                   = {.Size = sizeof(USB_Descriptor_Configuration_Header_t), .Type = DTYPE_Configuration},

			.TotalConfigurationSize   = CPU_TO_LE16(sizeof(USB_Descriptor_Configuration_t)),
			.TotalInterfaces          = 8,

			.ConfigurationNumber      = 1,
			.ConfigurationStrIndex    = NO_DESCRIPTOR,

			.ConfigAttributes         = (USB_CONFIG_ATTR_RESERVED /*| USB_CONFIG_ATTR_SELFPOWERED*/),

			.MaxPowerConsumption      = USB_CONFIG_POWER_MA(500)
		},

	.CDC1_IAD =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_Association_t), .Type = DTYPE_InterfaceAssociation},

			.FirstInterfaceIndex    = SER_FIRST,
			.TotalInterfaces        = 2,

			.Class                  = CDC_CSCP_CDCClass,
			.SubClass               = CDC_CSCP_ACMSubclass,
			.Protocol               = CDC_CSCP_ATCommandProtocol,

			.IADStrIndex            = NO_DESCRIPTOR
		},

	.CDC1_CCI_Interface =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber        = SER_FIRST,
			.AlternateSetting       = 0,

			.TotalEndpoints         = 1,

			.Class                  = CDC_CSCP_CDCClass,
			.SubClass               = CDC_CSCP_ACMSubclass,
			.Protocol               = CDC_CSCP_ATCommandProtocol,		//?? should be 0x2 actually for V.25ter AT protocol ??

			.InterfaceStrIndex      = NO_DESCRIPTOR
		},

	.CDC1_Functional_Header =
		{
			.Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalHeader_t), .Type = DTYPE_CSInterface},
			.Subtype                = CDC_DSUBTYPE_CSInterface_Header,

			.CDCSpecification       = VERSION_BCD(01.10),
		},

	.CDC1_Functional_ACM =
		{
			.Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalACM_t), .Type = DTYPE_CSInterface},
			.Subtype                = CDC_DSUBTYPE_CSInterface_ACM,

			.Capabilities           = 0x06,
		},

	.CDC1_Functional_Union =
		{
			.Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalUnion_t), .Type = DTYPE_CSInterface},
			.Subtype                = CDC_DSUBTYPE_CSInterface_Union,

			.MasterInterfaceNumber  = SER_FIRST,
			.SlaveInterfaceNumber   = SER_FIRST+1,
		},

	.CDC1_ManagementEndpoint =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},

			.EndpointAddress        = (ENDPOINT_DIR_IN | CDC1_NOTIFICATION_EPNUM),
			.Attributes             = (EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = CPU_TO_LE16(CDC1_NOTIFICATION_EPSIZE),
			.PollingIntervalMS      = 0xFF
		},

	.CDC1_DCI_Interface =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber        = SER_FIRST+1,
			.AlternateSetting       = 0,

			.TotalEndpoints         = 2,

			.Class                  = CDC_CSCP_CDCDataClass,
			.SubClass               = CDC_CSCP_NoDataSubclass,
			.Protocol               = CDC_CSCP_NoDataProtocol,

			.InterfaceStrIndex      = NO_DESCRIPTOR
		},

	.CDC1_DataOutEndpoint =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},

			.EndpointAddress        = (ENDPOINT_DIR_OUT | CDC1_RX_EPNUM),
			.Attributes             = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = CPU_TO_LE16(CDC1_RX_EPSIZE),
			.PollingIntervalMS      = 0x01
		},

	.CDC1_DataInEndpoint =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},

			.EndpointAddress        = (ENDPOINT_DIR_IN | CDC1_TX_EPNUM),
			.Attributes             = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = CPU_TO_LE16(CDC1_TX_EPSIZE),
			.PollingIntervalMS      = 0x01
		},

///////////////////////////////

	.CDC2_IAD =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_Association_t), .Type = DTYPE_InterfaceAssociation},

			.FirstInterfaceIndex    = SER_SECOND,
			.TotalInterfaces        = 2,

			.Class                  = CDC_CSCP_CDCClass,
			.SubClass               = CDC_CSCP_ACMSubclass,
			.Protocol               = CDC_CSCP_ATCommandProtocol,

			.IADStrIndex            = NO_DESCRIPTOR
		},

	.CDC2_CCI_Interface =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber        = SER_SECOND,
			.AlternateSetting       = 0,

			.TotalEndpoints         = 1,

			.Class                  = CDC_CSCP_CDCClass,
			.SubClass               = CDC_CSCP_ACMSubclass,
			.Protocol               = CDC_CSCP_ATCommandProtocol,		//?? should be 0x2 actually for V.25ter AT protocol ??

			.InterfaceStrIndex      = NO_DESCRIPTOR
		},

	.CDC2_Functional_Header =
		{
			.Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalHeader_t), .Type = DTYPE_CSInterface},
			.Subtype                = CDC_DSUBTYPE_CSInterface_Header,

			.CDCSpecification       = VERSION_BCD(01.10),
		},

	.CDC2_Functional_ACM =
		{
			.Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalACM_t), .Type = DTYPE_CSInterface},
			.Subtype                = CDC_DSUBTYPE_CSInterface_ACM,

			.Capabilities           = 0x06,
		},

	.CDC2_Functional_Union =
		{
			.Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalUnion_t), .Type = DTYPE_CSInterface},
			.Subtype                = CDC_DSUBTYPE_CSInterface_Union,

			.MasterInterfaceNumber  = SER_SECOND,
			.SlaveInterfaceNumber   = SER_SECOND+1,
		},

	.CDC2_ManagementEndpoint =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},

			.EndpointAddress        = (ENDPOINT_DIR_IN | CDC2_NOTIFICATION_EPNUM),
			.Attributes             = (EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = CPU_TO_LE16(CDC2_NOTIFICATION_EPSIZE),
			.PollingIntervalMS      = 0xFF
		},

	.CDC2_DCI_Interface =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber        = SER_SECOND+1,
			.AlternateSetting       = 0,

			.TotalEndpoints         = 2,

			.Class                  = CDC_CSCP_CDCDataClass,
			.SubClass               = CDC_CSCP_NoDataSubclass,
			.Protocol               = CDC_CSCP_NoDataProtocol,

			.InterfaceStrIndex      = NO_DESCRIPTOR
		},

	.CDC2_DataOutEndpoint =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},

			.EndpointAddress        = (ENDPOINT_DIR_OUT | CDC2_RX_EPNUM),
			.Attributes             = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = CPU_TO_LE16(CDC2_RX_EPSIZE),
			.PollingIntervalMS      = 0x01
		},

	.CDC2_DataInEndpoint =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},

			.EndpointAddress        = (ENDPOINT_DIR_IN | CDC2_TX_EPNUM),
			.Attributes             = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = CPU_TO_LE16(CDC2_TX_EPSIZE),
			.PollingIntervalMS      = 0x01
		},

///////////////////////////////

	.Audio_IAD =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_Association_t), .Type = DTYPE_InterfaceAssociation},

			.FirstInterfaceIndex    = AUDIO_STREAM_A_INTERFACE,
			.TotalInterfaces        = 2,

			.Class                    = AUDIO_CSCP_AudioClass,
			.SubClass                 = AUDIO_CSCP_ControlSubclass,
			.Protocol                 = AUDIO_CSCP_ControlProtocol,

			.IADStrIndex            = NO_DESCRIPTOR
		},

	.Audio_ControlInterface =
		{
			.Header                   = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber          = AUDIO_STREAM_A_INTERFACE,
			.AlternateSetting         = 0,

			.TotalEndpoints           = 0,

			.Class                    = AUDIO_CSCP_AudioClass,
			.SubClass                 = AUDIO_CSCP_ControlSubclass,
			.Protocol                 = AUDIO_CSCP_ControlProtocol,

			.InterfaceStrIndex        = NO_DESCRIPTOR
		},

	.Audio_ControlInterface_SPC =
		{
			.Header                   = {.Size = sizeof(USB_Audio_Descriptor_Interface_AC_t), .Type = DTYPE_CSInterface},
			.Subtype                  = AUDIO_DSUBTYPE_CSInterface_Header,

			.ACSpecification          = VERSION_BCD(01.00),
			.TotalLength              = CPU_TO_LE16(sizeof(USB_Audio_Descriptor_Interface_AC_t) +
			                                  sizeof(USB_Audio_Descriptor_InputTerminal_t) +
			                                  sizeof(USB_Audio_Descriptor_OutputTerminal_t)),

			.InCollection             = 1,
			.InterfaceNumber          = AUDIO_STREAM_A_INTERFACE+1,
		},

	.Audio_InputTerminal =
		{
			.Header                   = {.Size = sizeof(USB_Audio_Descriptor_InputTerminal_t), .Type = DTYPE_CSInterface},
			.Subtype                  = AUDIO_DSUBTYPE_CSInterface_InputTerminal,

			.TerminalID               = 0x01,
			.TerminalType             = CPU_TO_LE16(TERMINALTYPE_A),
			.AssociatedOutputTerminal = 0x00,

			//.TotalChannels            = 1,
			//.ChannelConfig            = 0,
			.TotalChannels            = AUDIOCHANNELS_A,
			.ChannelConfig            = CPU_TO_LE16(AUDIOCHANNELCFG_A),

			.ChannelStrIndex          = NO_DESCRIPTOR,
			.TerminalStrIndex         = NO_DESCRIPTOR
		},

	.Audio_OutputTerminal =
		{
			.Header                   = {.Size = sizeof(USB_Audio_Descriptor_OutputTerminal_t), .Type = DTYPE_CSInterface},
			.Subtype                  = AUDIO_DSUBTYPE_CSInterface_OutputTerminal,

			.TerminalID               = 0x02,
			.TerminalType             = CPU_TO_LE16(AUDIO_TERMINAL_STREAMING),
			.AssociatedInputTerminal  = 0x00,

			.SourceID                 = 0x01,

			.TerminalStrIndex         = NO_DESCRIPTOR
		},

	.Audio_StreamInterface_Alt0 =
		{
			.Header                   = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber          = AUDIO_STREAM_A_INTERFACE+1,
			.AlternateSetting         = 0,

			.TotalEndpoints           = 0,

			.Class                    = AUDIO_CSCP_AudioClass,
			.SubClass                 = AUDIO_CSCP_AudioStreamingSubclass,
			.Protocol                 = AUDIO_CSCP_StreamingProtocol,

			.InterfaceStrIndex        = NO_DESCRIPTOR
		},

	.Audio_StreamInterface_Alt1 =
		{
			.Header                   = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber          = AUDIO_STREAM_A_INTERFACE+1,
			.AlternateSetting         = 1,

			.TotalEndpoints           = 1,

			.Class                    = AUDIO_CSCP_AudioClass,
			.SubClass                 = AUDIO_CSCP_AudioStreamingSubclass,
			.Protocol                 = AUDIO_CSCP_StreamingProtocol,

			.InterfaceStrIndex        = NO_DESCRIPTOR
		},

	.Audio_StreamInterface_SPC =
		{
			.Header                   = {.Size = sizeof(USB_Audio_Descriptor_Interface_AS_t), .Type = DTYPE_CSInterface},
			.Subtype                  = AUDIO_DSUBTYPE_CSInterface_General,

			.TerminalLink             = 0x02,

			.FrameDelay               = 1,
			.AudioFormat              = CPU_TO_LE16(0x0001)
		},

	.Audio_AudioFormat =
		{
			.Header                   = {.Size = sizeof(USB_Audio_Descriptor_Format_t) + (AUDIO_SUPPORTEDRATES_A*sizeof(USB_Audio_SampleFreq_t)), .Type = DTYPE_CSInterface},
			.Subtype                  = AUDIO_DSUBTYPE_CSInterface_FormatType,

			.FormatType               = 0x01,
			.Channels                 = AUDIOCHANNELS_A, /*****/

			.SubFrameSize             = AUDIOSUBFRAMESIZE_A,
			.BitResolution            = 16,	//AUDIOBITS,

			//.SampleFrequencyType      = AUDIO_TOTAL_SAMPLE_RATES,
			.TotalDiscreteSampleRates = AUDIO_SUPPORTEDRATES_A, //(sizeof(ConfigurationDescriptor.Audio_AudioFormatSampleRates) / sizeof(USB_Audio_SampleFreq_t)),

			//.SampleFrequencies        = {AUDIO_SAMPLE_FREQ(AUDIO_SAMPLE_FREQUENCY_A)}
		},

	.Audio_AudioFormatSampleRates =
		{
			AUDIO_SAMPLE_FREQ(AUDIO_SAMPLE_FREQUENCY_A)
			//AUDIO_SAMPLE_FREQ(8000),
			//AUDIO_SAMPLE_FREQ(11025),
			//AUDIO_SAMPLE_FREQ(22050),
			//AUDIO_SAMPLE_FREQ(44100),
			//AUDIO_SAMPLE_FREQ(48000),
		},

	.Audio_StreamEndpoint =
		{
			.Endpoint =
				{
					.Header              = {.Size = sizeof(USB_Audio_Descriptor_StreamEndpoint_Std_t), .Type = DTYPE_Endpoint},

					.EndpointAddress     = (ENDPOINT_DIR_IN | AUDIO_STREAM_A_EPNUM),
					.Attributes          = AUDIOATTRIBUTES_A,
					.EndpointSize        = CPU_TO_LE16(AUDIO_STREAM_A_EPSIZE),
					.PollingIntervalMS   = 0x01
				},

			.Refresh                  = 0,
			.SyncEndpointNumber       = 0
		},

	.Audio_StreamEndpoint_SPC =
		{
			.Header                   = {.Size = sizeof(USB_Audio_Descriptor_StreamEndpoint_Spc_t), .Type = DTYPE_CSEndpoint},
			.Subtype                  = AUDIO_DSUBTYPE_CSEndpoint_General,

			.Attributes               = (AUDIO_EP_ACCEPTS_SMALL_PACKETS /* | AUDIO_EP_SAMPLE_FREQ_CONTROL*/),

			.LockDelayUnits           = 0x0, //0x02,		// 2 for pcm
			.LockDelay                = CPU_TO_LE16(0x0000)
		},

//////////////////////////////

	/////////////////////////////////

    .Audio2_IAD =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_Association_t), .Type = DTYPE_InterfaceAssociation},

			.FirstInterfaceIndex    = AUDIO_STREAM_B_INTERFACE,
			.TotalInterfaces        = 2,

			.Class                    = AUDIO_CSCP_AudioClass,
			.SubClass                 = AUDIO_CSCP_ControlSubclass,
			.Protocol                 = AUDIO_CSCP_ControlProtocol,

			.IADStrIndex            = NO_DESCRIPTOR
		},

	.Audio2_ControlInterface =
		{
			.Header                   = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber          = AUDIO_STREAM_B_INTERFACE,
			.AlternateSetting         = 0,

			.TotalEndpoints           = 0,

			.Class                    = AUDIO_CSCP_AudioClass,
			.SubClass                 = AUDIO_CSCP_ControlSubclass,
			.Protocol                 = AUDIO_CSCP_ControlProtocol,

			.InterfaceStrIndex        = NO_DESCRIPTOR
		},

	.Audio2_ControlInterface_SPC =
		{
			.Header                   = {.Size = sizeof(USB_Audio_Descriptor_Interface_AC_t), .Type = DTYPE_CSInterface},
			.Subtype                  = AUDIO_DSUBTYPE_CSInterface_Header,

			.ACSpecification          = VERSION_BCD(01.00),
			.TotalLength              = CPU_TO_LE16(sizeof(USB_Audio_Descriptor_Interface_AC_t) +
			                             sizeof(USB_Audio_Descriptor_InputTerminal_t) +
			                             sizeof(USB_Audio_Descriptor_OutputTerminal_t)),

			.InCollection             = 1,
			.InterfaceNumber          = AUDIO_STREAM_B_INTERFACE+1,
		},

	.Audio2_InputTerminal =
		{
			.Header                   = {.Size = sizeof(USB_Audio_Descriptor_InputTerminal_t), .Type = DTYPE_CSInterface},
			.Subtype                  = AUDIO_DSUBTYPE_CSInterface_InputTerminal,

			.TerminalID               = 0x01,
			.TerminalType             = CPU_TO_LE16(TERMINALTYPE_B),
			.AssociatedOutputTerminal = 0x00,

			//.TotalChannels            = 1,
			//.ChannelConfig            = 0,
			.TotalChannels            = AUDIOCHANNELS_B,
			.ChannelConfig            = CPU_TO_LE16(AUDIOCHANNELCFG_B),

			.ChannelStrIndex          = NO_DESCRIPTOR,
			.TerminalStrIndex         = NO_DESCRIPTOR
		},

	.Audio2_OutputTerminal =
		{
			.Header                   = {.Size = sizeof(USB_Audio_Descriptor_OutputTerminal_t), .Type = DTYPE_CSInterface},
			.Subtype                  = AUDIO_DSUBTYPE_CSInterface_OutputTerminal,

			.TerminalID               = 0x02,
			.TerminalType             = CPU_TO_LE16(AUDIO_TERMINAL_STREAMING),
			.AssociatedInputTerminal  = 0x00,

			.SourceID                 = 0x01,

			.TerminalStrIndex         = NO_DESCRIPTOR
		},

	.Audio2_StreamInterface_Alt0 =
		{
			.Header                   = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber          = AUDIO_STREAM_B_INTERFACE+1,
			.AlternateSetting         = 0,

			.TotalEndpoints           = 0,

			.Class                    = AUDIO_CSCP_AudioClass,
			.SubClass                 = AUDIO_CSCP_AudioStreamingSubclass,
			.Protocol                 = AUDIO_CSCP_StreamingProtocol,

			.InterfaceStrIndex        = NO_DESCRIPTOR
		},

	.Audio2_StreamInterface_Alt1 =
		{
			.Header                   = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber          = AUDIO_STREAM_B_INTERFACE+1,
			.AlternateSetting         = 1,

			.TotalEndpoints           = 1,

			.Class                    = AUDIO_CSCP_AudioClass,
			.SubClass                 = AUDIO_CSCP_AudioStreamingSubclass,
			.Protocol                 = AUDIO_CSCP_StreamingProtocol,

			.InterfaceStrIndex        = NO_DESCRIPTOR
		},

	.Audio2_StreamInterface_SPC =
		{
			.Header                   = {.Size = sizeof(USB_Audio_Descriptor_Interface_AS_t), .Type = DTYPE_CSInterface},
			.Subtype                  = AUDIO_DSUBTYPE_CSInterface_General,

			.TerminalLink             = 0x02,

			.FrameDelay               = 1,
			.AudioFormat              = CPU_TO_LE16(0x0001)
		},

	.Audio2_AudioFormat =
		{
			.Header                   = {.Size = sizeof(USB_Audio_Descriptor_Format_t) + (AUDIO_SUPPORTEDRATES_B*sizeof(USB_Audio_SampleFreq_t)), .Type = DTYPE_CSInterface},
			.Subtype                  = AUDIO_DSUBTYPE_CSInterface_FormatType,

			.FormatType               = 0x01,
			.Channels                 = AUDIOCHANNELS_B, /*****/

			.SubFrameSize             = AUDIOSUBFRAMESIZE_B,
			.BitResolution            = 16, //AUDIOBITS,

			//.SampleFrequencyType      = AUDIO_TOTAL_SAMPLE_RATES,
			.TotalDiscreteSampleRates = AUDIO_SUPPORTEDRATES_B, //(sizeof(ConfigurationDescriptor.Audio_AudioFormatSampleRates) / sizeof(USB_Audio_SampleFreq_t)),

			//.SampleFrequencies        = {AUDIO_SAMPLE_FREQ(AUDIO_SAMPLE_FREQUENCY_B)}
		},

	.Audio2_AudioFormatSampleRates =
		{
			AUDIO_SAMPLE_FREQ(AUDIO_SAMPLE_FREQUENCY_B)
			//AUDIO_SAMPLE_FREQ(8000),
			//AUDIO_SAMPLE_FREQ(11025),
			//AUDIO_SAMPLE_FREQ(22050),
			//AUDIO_SAMPLE_FREQ(44100),
			//AUDIO_SAMPLE_FREQ(48000),
		},

	.Audio2_StreamEndpoint =
		{
			.Endpoint =
				{
					.Header              = {.Size = sizeof(USB_Audio_Descriptor_StreamEndpoint_Std_t), .Type = DTYPE_Endpoint},

					.EndpointAddress     = (ENDPOINT_DIR_IN | AUDIO_STREAM_B_EPNUM),
					.Attributes          = AUDIOATTRIBUTES_B,
					.EndpointSize        = CPU_TO_LE16(AUDIO_STREAM_B_EPSIZE),
					.PollingIntervalMS   = 0x01
				},

			.Refresh                  = 0,
			.SyncEndpointNumber       = 0
		},

	.Audio2_StreamEndpoint_SPC =
		{
			.Header                   = {.Size = sizeof(USB_Audio_Descriptor_StreamEndpoint_Spc_t), .Type = DTYPE_CSEndpoint},
			.Subtype                  = AUDIO_DSUBTYPE_CSEndpoint_General,

			.Attributes               = (AUDIO_EP_ACCEPTS_SMALL_PACKETS /* | AUDIO_EP_SAMPLE_FREQ_CONTROL */),

			.LockDelayUnits           = 0x0, //0x02,		// 2 for pcm
			.LockDelay                = CPU_TO_LE16(0x0000)
		}
};


/** Language descriptor structure. This descriptor, located in FLASH memory, is returned when the host requests
 *  the string descriptor with index 0 (the first index). It is actually an array of 16-bit integers, which indicate
 *  via the language ID table available at USB.org what languages the device supports for its string descriptors.
 */
USB_Descriptor_String_t PROGMEM LanguageString =
{
	.Header                 = {.Size = USB_STRING_LEN(1), .Type = DTYPE_String},

	.UnicodeString          = {CPU_TO_LE16(LANGUAGE_ID_ENG)}
};

/** Manufacturer descriptor string. This is a Unicode string containing the manufacturer's details in human readable
 *  form, and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.
 */
USB_Descriptor_String_t PROGMEM ManufacturerString =
{
	.Header                 = {.Size = USB_STRING_LEN(10), .Type = DTYPE_String},

	.UnicodeString          = {CPU_TO_LE16('U'),
	                           CPU_TO_LE16('V'),
	                           CPU_TO_LE16('B'),
	                           CPU_TO_LE16('-'),
	                           CPU_TO_LE16('7'),
	                           CPU_TO_LE16('6'),
	                           CPU_TO_LE16('.'),
	                           CPU_TO_LE16('n'),
	                           CPU_TO_LE16('e'),
	                           CPU_TO_LE16('t')}
};

/** Product descriptor string. This is a Unicode string containing the product's details in human readable form,
 *  and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.
 */
USB_Descriptor_String_t PROGMEM ProductString =
{
	.Header                 = {.Size = USB_STRING_LEN(15), .Type = DTYPE_String},

	.UnicodeString          = {CPU_TO_LE16('S'),
	                           CPU_TO_LE16('D'),
	                           CPU_TO_LE16('R'),
	                           CPU_TO_LE16(' '),
	                           CPU_TO_LE16('M'),
	                           CPU_TO_LE16('K'),
	                           CPU_TO_LE16('1'),
	                           CPU_TO_LE16('.'),
	                           CPU_TO_LE16('5'),
	                           CPU_TO_LE16(' '),
	                           CPU_TO_LE16('R'),
	                           CPU_TO_LE16('a'),
	                           CPU_TO_LE16('d'),
	                           CPU_TO_LE16('i'),
	                           CPU_TO_LE16('o')}
};
/*
USB_Descriptor_String_t PROGMEM AudioAString =
{
	.Header                 = {.Size = USB_STRING_LEN(20), .Type = DTYPE_String},

	.UnicodeString          = {CPU_TO_LE16('S'),
	                           CPU_TO_LE16('D'),
	                           CPU_TO_LE16('R'),
	                           CPU_TO_LE16(' '),
	                           CPU_TO_LE16('M'),
	                           CPU_TO_LE16('K'),
	                           CPU_TO_LE16('1'),
	                           CPU_TO_LE16('.'),
	                           CPU_TO_LE16('5'),
	                           CPU_TO_LE16(' '),
	                           CPU_TO_LE16('R'),
	                           CPU_TO_LE16('a'),
	                           CPU_TO_LE16('d'),
	                           CPU_TO_LE16('i'),
	                           CPU_TO_LE16('o'),
							   CPU_TO_LE16(' '),
	                           CPU_TO_LE16('C'),
	                           CPU_TO_LE16('H'),
	                           CPU_TO_LE16('_'),
	                           CPU_TO_LE16('A')}
};

USB_Descriptor_String_t PROGMEM AudioBString =
{
	.Header                 = {.Size = USB_STRING_LEN(20), .Type = DTYPE_String},

	.UnicodeString          = {CPU_TO_LE16('S'),
	                           CPU_TO_LE16('D'),
	                           CPU_TO_LE16('R'),
	                           CPU_TO_LE16(' '),
	                           CPU_TO_LE16('M'),
	                           CPU_TO_LE16('K'),
	                           CPU_TO_LE16('1'),
	                           CPU_TO_LE16('.'),
	                           CPU_TO_LE16('5'),
	                           CPU_TO_LE16(' '),
	                           CPU_TO_LE16('R'),
	                           CPU_TO_LE16('a'),
	                           CPU_TO_LE16('d'),
	                           CPU_TO_LE16('i'),
	                           CPU_TO_LE16('o'),
							   CPU_TO_LE16(' '),
	                           CPU_TO_LE16('C'),
	                           CPU_TO_LE16('H'),
	                           CPU_TO_LE16('_'),
	                           CPU_TO_LE16('B')}
};
*/

extern USB_ClassInfo_Audio_Device_t Main_Audio_Interface;
extern USB_ClassInfo_Audio_Device_t AudioB_Audio_Interface;

/** This function is called by the library when in device mode, and must be overridden (see library "USB Descriptors"
 *  documentation) by the application code so that the address and size of a requested descriptor can be given
 *  to the USB library. When the device receives a Get Descriptor request on the control endpoint, this function
 *  is called so that the descriptor details can be passed back and the appropriate descriptor sent back to the
 *  USB host.
 */


uint16_t windexes[64]={-1};
int __j=0;
extern USB_Request_Header_t USB_ControlRequest;
extern int16_t lastIndex;

uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint8_t wIndex,
                                    const void** const DescriptorAddress)
{
	const uint8_t  DescriptorType   = (wValue >> 8);
	const uint8_t  DescriptorNumber = (wValue & 0xFF);

	const void* Address = NULL;
	uint16_t    Size    = NO_DESCRIPTOR;

//	static uint16_t ifcorder=0;

	switch (DescriptorType)
	{
		case DTYPE_Device:
			Address = &DeviceDescriptor;
			Size    = sizeof(USB_Descriptor_Device_t);
			break;
		case DTYPE_Configuration:
			if (external_power)
				ConfigurationDescriptor.Config.ConfigAttributes=(USB_CONFIG_ATTR_SELFPOWERED);
			else
				ConfigurationDescriptor.Config.ConfigAttributes=(USB_CONFIG_ATTR_RESERVED);

			Address = &ConfigurationDescriptor;

			Size    = sizeof(USB_Descriptor_Configuration_t);
			break;
		case DTYPE_String:
			switch (DescriptorNumber)
			{
				case 0x00:
					Address = &LanguageString;
					Size    = pgm_read_byte(&LanguageString.Header.Size);
					break;
				case 0x01:
					Address = &ManufacturerString;
					Size    = pgm_read_byte(&ManufacturerString.Header.Size);
					break;
				case 0x02:
/*
					// hopeless!
					windexes[__j]=lastIndex;	//USB_ControlRequest.wIndex;	//Endpoint_GetCurrentEndpoint();
					if (__j < 64)
						__j++;
*/
/*
					if (!ifcorder)
					{
						Address = &AudioAString;
						Size    = pgm_read_byte(&AudioAString.Header.Size);
						ifcorder++;
					}
					else
					{
						Address = &AudioBString;
						Size    = pgm_read_byte(&AudioBString.Header.Size);
					}
					else
*/
					{
						Address = &ProductString;
						Size    = pgm_read_byte(&ProductString.Header.Size);
					}
					break;
			}

			break;
	}

	*DescriptorAddress = Address;
	return Size;
}


/** Audio class driver callback for the setting and retrieval of streaming endpoint properties. This callback must be implemented
 *  in the user application to handle property manipulations on streaming audio endpoints.
 *
 *  When the DataLength parameter is NULL, this callback should only indicate whether the specified operation is valid for
 *  the given endpoint index, and should return as fast as possible. When non-NULL, this value may be altered for GET operations
 *  to indicate the size of the retreived data.
 *
 *  \note The length of the retrieved data stored into the Data buffer on GET operations should not exceed the initial value
 *        of the \c DataLength parameter.
 *
 *  \param[in,out] AudioInterfaceInfo  Pointer to a structure containing an Audio Class configuration and state.
 *  \param[in]     EndpointProperty    Property of the endpoint to get or set, a value from Audio_ClassRequests_t.
 *  \param[in]     EndpointAddress     Address of the streaming endpoint whose property is being referenced.
 *  \param[in]     EndpointControl     Parameter of the endpoint to get or set, a value from Audio_EndpointControls_t.
 *  \param[in,out] DataLength          For SET operations, the length of the parameter data to set. For GET operations, the maximum
 *                                     length of the retrieved data. When NULL, the function should return whether the given property
 *                                     and parameter is valid for the requested endpoint without reading or modifying the Data buffer.
 *  \param[in,out] Data                Pointer to a location where the parameter data is stored for SET operations, or where
 *                                     the retrieved data is to be stored for GET operations.
 *
 *  \return Boolean true if the property get/set was successful, false otherwise
 */

static volatile uint32_t CurrentAudioSampleFrequencyA = AUDIO_SAMPLE_FREQUENCY_A;
static volatile uint32_t CurrentAudioSampleFrequencyB = AUDIO_SAMPLE_FREQUENCY_B;

bool CALLBACK_Audio_Device_GetSetEndpointProperty(USB_ClassInfo_Audio_Device_t* const AudioInterfaceInfo,
                                                  const uint8_t EndpointProperty,
                                                  const uint8_t EndpointAddress,
                                                  const uint8_t EndpointControl,
                                                  uint16_t* const DataLength,
                                                  uint8_t* Data)
{
/*
	// Check the requested endpoint to see if a supported endpoint is being manipulated
	if (EndpointAddress == (ENDPOINT_DIR_IN | Main_Audio_Interface.Config.DataINEndpointNumber))
	{
		// Check the requested control to see if a supported control is being manipulated
		if (EndpointControl == AUDIO_EPCONTROL_SamplingFreq)
		{
			// Check the requested property to see if a supported property is being manipulated
			switch (EndpointProperty)
			{
				case AUDIO_REQ_SetCurrent:
					// Check if we are just testing for a valid property, or actually adjusting it
					if (DataLength != NULL)
					{
						// Set the new sampling frequency to the value given by the host
						//CurrentAudioSampleFrequencyB = (((uint32_t)Data[2] << 16) | ((uint32_t)Data[1] << 8) | (uint32_t)Data[0]);

						// Adjust sample reload timer to the new frequency
						//OCR0A = ((F_CPU / 8 / CurrentAudioSampleFrequency) - 1);
					}

					return true;

				case AUDIO_REQ_GetCurrent:
					// Check if we are just testing for a valid property, or actually reading it
					if (DataLength != NULL)
					{
						*DataLength = 3;

						Data[2] = (CurrentAudioSampleFrequencyA >> 16);
						Data[1] = (CurrentAudioSampleFrequencyA >> 8);
						Data[0] = (CurrentAudioSampleFrequencyA &  0xFF);
					}

					return true;
			}
		}
	}
	else if (EndpointAddress == (ENDPOINT_DIR_IN | AudioB_Audio_Interface.Config.DataINEndpointNumber))
	{
		// Check the requested control to see if a supported control is being manipulated
		if (EndpointControl == AUDIO_EPCONTROL_SamplingFreq)
		{
			// Check the requested property to see if a supported property is being manipulated
			switch (EndpointProperty)
			{
				case AUDIO_REQ_SetCurrent:
					// Check if we are just testing for a valid property, or actually adjusting it
					if (DataLength != NULL)
					{
						// Set the new sampling frequency to the value given by the host
						//CurrentAudioSampleFrequencyB = (((uint32_t)Data[2] << 16) | ((uint32_t)Data[1] << 8) | (uint32_t)Data[0]);

						// Adjust sample reload timer to the new frequency
						//OCR0A = ((F_CPU / 8 / CurrentAudioSampleFrequency) - 1);
					}

					return true;

				case AUDIO_REQ_GetCurrent:
					// Check if we are just testing for a valid property, or actually reading it
					if (DataLength != NULL)
					{
						*DataLength = 3;

						Data[2] = (CurrentAudioSampleFrequencyB >> 16);
						Data[1] = (CurrentAudioSampleFrequencyB >> 8);
						Data[0] = (CurrentAudioSampleFrequencyB &  0xFF);
					}

					return true;
			}
		}
	}
*/
	return true;	//false;
}

/** Audio class driver callback for the setting and retrieval of streaming interface properties. This callback must be implemented
 *  in the user application to handle property manipulations on streaming audio interfaces.
 *
 *  When the DataLength parameter is NULL, this callback should only indicate whether the specified operation is valid for
 *  the given entity and should return as fast as possible. When non-NULL, this value may be altered for GET operations
 *  to indicate the size of the retreived data.
 *
 *  \note The length of the retrieved data stored into the Data buffer on GET operations should not exceed the initial value
 *        of the \c DataLength parameter.
 *
 *  \param[in,out] AudioInterfaceInfo  Pointer to a structure containing an Audio Class configuration and state.
 *  \param[in]     Property            Property of the interface to get or set, a value from \ref Audio_ClassRequests_t.
 *  \param[in]     EntityAddress       Address of the audio entity whose property is being referenced.
 *  \param[in]     Parameter           Parameter of the entity to get or set, specific to each type of entity (see USB Audio specification).
 *  \param[in,out] DataLength          For SET operations, the length of the parameter data to set. For GET operations, the maximum
 *                                     length of the retrieved data. When NULL, the function should return whether the given property
 *                                     and parameter is valid for the requested endpoint without reading or modifying the Data buffer.
 *  \param[in,out] Data                Pointer to a location where the parameter data is stored for SET operations, or where
 *                                     the retrieved data is to be stored for GET operations.
 *
 *  \return Boolean \c true if the property GET/SET was successful, \c false otherwise
 */
bool CALLBACK_Audio_Device_GetSetInterfaceProperty(USB_ClassInfo_Audio_Device_t* const AudioInterfaceInfo,
                                                   const uint8_t Property,
                                                   const uint8_t EntityAddress,
                                                   const uint16_t Parameter,
                                                   uint16_t* const DataLength,
                                                   uint8_t* Data)
{
	/* No audio interface entities in the device descriptor, thus no properties to get or set. */
	return true;	//false;
}

