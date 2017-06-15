#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/audio.h>
#include <libopencm3/usb/midi.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>

//static usbd_device *usbd_dev;


/*
 * All references in this file come from Universal Serial Bus Device Class
 * Definition for MIDI Devices, release 1.0.
 */

/*
 * Table B-1: MIDI Adapter Device Descriptor
 */
static const struct usb_device_descriptor dev = {
   .bLength = USB_DT_DEVICE_SIZE,
   .bDescriptorType = USB_DT_DEVICE,
   .bcdUSB = 0x0200,    /* was 0x0110 in Table B-1 example descriptor */
   .bDeviceClass = 0,   /* device defined at interface level */
   .bDeviceSubClass = 0,
   .bDeviceProtocol = 0,
   .bMaxPacketSize0 = 64,
   .idVendor = 0x6666,  /* Prototype product vendor ID */
   .idProduct = 0xB0B0, /* dd if=/dev/random bs=2 count=1 | hexdump */
   .bcdDevice = 0x0100,
   .iManufacturer = 1,  /* index to string desc */
   .iProduct = 2,       /* index to string desc */
   .iSerialNumber = 3,  /* index to string desc */
   .bNumConfigurations = 1,
};

/*
 * Midi specific endpoint descriptors.
 */
static const struct usb_midi_endpoint_descriptor midi_bulk_endp[] = {{
   /* Table B-12: MIDI Adapter Class-specific Bulk OUT Endpoint
    * Descriptor
    */
   .head = {
      .bLength = sizeof(struct usb_midi_endpoint_descriptor),
      .bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
      .bDescriptorSubType = USB_MIDI_SUBTYPE_MS_GENERAL,
      .bNumEmbMIDIJack = 1,
   },
   .jack[0] = {
      .baAssocJackID = 0x01,
   },
}, {
   /* Table B-14: MIDI Adapter Class-specific Bulk IN Endpoint
    * Descriptor
    */
   .head = {
      .bLength = sizeof(struct usb_midi_endpoint_descriptor),
      .bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
      .bDescriptorSubType = USB_MIDI_SUBTYPE_MS_GENERAL,
      .bNumEmbMIDIJack = 1,
   },
   .jack[0] = {
      .baAssocJackID = 0x03,
   },
} };

/*
 * Standard endpoint descriptors
 */
static const struct usb_endpoint_descriptor bulk_endp[] = {{
   /* Table B-11: MIDI Adapter Standard Bulk OUT Endpoint Descriptor */
   .bLength = USB_DT_ENDPOINT_SIZE,
   .bDescriptorType = USB_DT_ENDPOINT,
   .bEndpointAddress = 0x01,
   .bmAttributes = USB_ENDPOINT_ATTR_BULK,
   .wMaxPacketSize = 0x40,
   .bInterval = 0x00,

   .extra = &midi_bulk_endp[0],
   .extralen = sizeof(midi_bulk_endp[0])
}, {
   .bLength = USB_DT_ENDPOINT_SIZE,
   .bDescriptorType = USB_DT_ENDPOINT,
   .bEndpointAddress = 0x81,
   .bmAttributes = USB_ENDPOINT_ATTR_BULK,
   .wMaxPacketSize = 0x40,
   .bInterval = 0x00,

   .extra = &midi_bulk_endp[1],
   .extralen = sizeof(midi_bulk_endp[1])
} };

/*
 * Table B-4: MIDI Adapter Class-specific AC Interface Descriptor
 */
static const struct {
   struct usb_audio_header_descriptor_head header_head;
   struct usb_audio_header_descriptor_body header_body;
} __attribute__((packed)) audio_control_functional_descriptors = {
   .header_head = {
      .bLength = sizeof(struct usb_audio_header_descriptor_head) +
                 1 * sizeof(struct usb_audio_header_descriptor_body),
      .bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
      .bDescriptorSubtype = USB_AUDIO_TYPE_HEADER,
      .bcdADC = 0x0100,
      .wTotalLength =
            sizeof(struct usb_audio_header_descriptor_head) +
            1 * sizeof(struct usb_audio_header_descriptor_body),
      .binCollection = 1,
   },
   .header_body = {
      .baInterfaceNr = 0x01,
   },
};

/*
 *  * Table B-3: MIDI Adapter Standard AC Interface Descriptor
 *   */
static const struct usb_interface_descriptor audio_control_iface[] = {{
   .bLength = USB_DT_INTERFACE_SIZE,
   .bDescriptorType = USB_DT_INTERFACE,
   .bInterfaceNumber = 0,
   .bAlternateSetting = 0,
   .bNumEndpoints = 0,
   .bInterfaceClass = USB_CLASS_AUDIO,
   .bInterfaceSubClass = USB_AUDIO_SUBCLASS_CONTROL,
   .bInterfaceProtocol = 0,
   .iInterface = 0,

   .extra = &audio_control_functional_descriptors,
   .extralen = sizeof(audio_control_functional_descriptors)
} };

/*
 * Class-specific MIDI streaming interface descriptor
 */
static const struct {
   struct usb_midi_header_descriptor header;
   struct usb_midi_in_jack_descriptor in_embedded;
   struct usb_midi_in_jack_descriptor in_external;
   struct usb_midi_out_jack_descriptor out_embedded;
   struct usb_midi_out_jack_descriptor out_external;
} __attribute__((packed)) midi_streaming_functional_descriptors = {
   /* Table B-6: Midi Adapter Class-specific MS Interface Descriptor */
   .header = {
      .bLength = sizeof(struct usb_midi_header_descriptor),
      .bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
      .bDescriptorSubtype = USB_MIDI_SUBTYPE_MS_HEADER,
      .bcdMSC = 0x0100,
      .wTotalLength = sizeof(midi_streaming_functional_descriptors),
   },
   /* Table B-7: MIDI Adapter MIDI IN Jack Descriptor (Embedded) */
   .in_embedded = {
      .bLength = sizeof(struct usb_midi_in_jack_descriptor),
      .bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
      .bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_IN_JACK,
      .bJackType = USB_MIDI_JACK_TYPE_EMBEDDED,
      .bJackID = 0x01,
      .iJack = 0x00,
   },
   /* Table B-8: MIDI Adapter MIDI IN Jack Descriptor (External) */
   .in_external = {
      .bLength = sizeof(struct usb_midi_in_jack_descriptor),
      .bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
      .bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_IN_JACK,
      .bJackType = USB_MIDI_JACK_TYPE_EXTERNAL,
      .bJackID = 0x02,
      .iJack = 0x00,
   },
   /* Table B-9: MIDI Adapter MIDI OUT Jack Descriptor (Embedded) */
   .out_embedded = {
      .head = {
         .bLength = sizeof(struct usb_midi_out_jack_descriptor),
         .bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
         .bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_OUT_JACK,
         .bJackType = USB_MIDI_JACK_TYPE_EMBEDDED,
         .bJackID = 0x03,
         .bNrInputPins = 1,
      },
      .source[0] = {
         .baSourceID = 0x02,
         .baSourcePin = 0x01,
      },
      .tail = {
         .iJack = 0x00,
      }
   },
   /* Table B-10: MIDI Adapter MIDI OUT Jack Descriptor (External) */
   .out_external = {
      .head = {
         .bLength = sizeof(struct usb_midi_out_jack_descriptor),
         .bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
         .bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_OUT_JACK,
         .bJackType = USB_MIDI_JACK_TYPE_EXTERNAL,
         .bJackID = 0x04,
         .bNrInputPins = 1,
      },
      .source[0] = {
         .baSourceID = 0x01,
         .baSourcePin = 0x01,
      },
      .tail = {
         .iJack = 0x00,
      },
   },
};

/*
 * Table B-5: MIDI Adapter Standard MS Interface Descriptor
 */
static const struct usb_interface_descriptor midi_streaming_iface[] = {{
   .bLength = USB_DT_INTERFACE_SIZE,
   .bDescriptorType = USB_DT_INTERFACE,
   .bInterfaceNumber = 1,
   .bAlternateSetting = 0,
   .bNumEndpoints = 2,
   .bInterfaceClass = USB_CLASS_AUDIO,
   .bInterfaceSubClass = USB_AUDIO_SUBCLASS_MIDISTREAMING,
   .bInterfaceProtocol = 0,
   .iInterface = 0,

   .endpoint = bulk_endp,

   .extra = &midi_streaming_functional_descriptors,
   .extralen = sizeof(midi_streaming_functional_descriptors)
} };

static const struct usb_interface ifaces[] = {{
   .num_altsetting = 1,
   .altsetting = audio_control_iface,
}, {
   .num_altsetting = 1,
   .altsetting = midi_streaming_iface,
} };

/*
 * Table B-2: MIDI Adapter Configuration Descriptor
 */
static const struct usb_config_descriptor config = {
   .bLength = USB_DT_CONFIGURATION_SIZE,
   .bDescriptorType = USB_DT_CONFIGURATION,
   .wTotalLength = 0, /* can be anything, it is updated automatically
                         when the usb code prepares the descriptor */
   .bNumInterfaces = 2, /* control and data */
   .bConfigurationValue = 1,
   .iConfiguration = 0,
   .bmAttributes = 0x80, /* bus powered */
   .bMaxPower = 0x32,

   .interface = ifaces,
};

static const char* usb_strings[] = {
   "PabloGAD",
   "Teclao MIDI",
   "PGAD00000001\0"
};

// Buffer for control requests
uint8_t usbd_control_buffer[128];

/* SysEx identity message, preformatted with correct USB framing information */
const uint8_t sysex_identity[] = {
   0x04,   /* USB Framing (3 byte SysEx) */
   0xf0,   /* SysEx start */
   0x7e,   /* non-realtime */
   0x00,   /* Channel 0 */
   0x04,   /* USB Framing (3 byte SysEx) */
   0x7d,   /* Educational/prototype manufacturer ID */
   0x66,   /* Family code (byte 1) */
   0x66,   /* Family code (byte 2) */
   0x04,   /* USB Framing (3 byte SysEx) */
   0x51,   /* Model number (byte 1) */
   0x19,   /* Model number (byte 2) */
   0x00,   /* Version number (byte 1) */
   0x04,   /* USB Framing (3 byte SysEx) */
   0x00,   /* Version number (byte 2) */
   0x01,   /* Version number (byte 3) */
   0x00,   /* Version number (byte 4) */
   0x05,   /* USB Framing (1 byte SysEx) */
   0xf7,   /* SysEx end */
   0x00,   /* Padding */
   0x00,   /* Padding */
};


static void usbmidi_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
   (void)ep;

   char buf[64];
   int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

   /* This implementation treats any message from the host as a SysEx
    * identity request. This works well enough providing the host
    * packs the identify request in a single 8 byte USB message.
    */
   if (len) {
      while (usbd_ep_write_packet(usbd_dev, 0x81, sysex_identity,
                   sizeof(sysex_identity)) == 0);
   }

   gpio_toggle(GPIOC, GPIO5);
}

static void usbmidi_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
   (void)wValue;

   usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64,
         usbmidi_data_rx_cb);
   usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, 0);
}

static void button_send_event(usbd_device *usbd_dev, int pressed)
{
   char buf[4] = { 0x08, /* USB framing: virtual cable 0, note off */
                   0x80, /* MIDI command: note off, channel 1 */
                   60,   /* Note 60 (middle C) */
                   64,   /* "Normal" velocity */
   };

   buf[0] |= pressed;
   buf[1] |= pressed << 4;

   while (usbd_ep_write_packet(usbd_dev, 0x81, buf, sizeof(buf)) == 0);
}

static void button_poll(usbd_device *usbd_dev)
{
   static uint32_t button_state = 0;

   /* This is a simple shift based debounce. It's simplistic because
    * although this implements debounce adequately it does not have any
    * noise suppression. It is also very wide (32-bits) because it can
    * be polled in a very tight loop (no debounce timer).
    */
   uint32_t old_button_state = button_state;
   button_state = (button_state << 1) | (GPIOB_IDR & GPIO0);
   if ((0 == button_state) != (0 == old_button_state)) {
      button_send_event(usbd_dev, !!button_state);
   }
}

// Keyboard map: 16 rows x 8 columns
static uint8_t keyboard[ 16 ];
static uint8_t transpose = 2;   // MIDI scale of the first C on the keyboard (first key)

// Events map - fifo of wainting events
uint32_t events[ MAX_EVENTS ];
int event_start = 0;  // First event
int num_events = 0;   // Number of events queued < MAX_EVENTS

static int enqueue_event( uint32_t ev ) {
	int event_ptr = -1;
	if( num_events < MAX_EVENTS ) {
		event_ptr = event_start + num_events;
		if( event_ptr >= MAX_EVENTS ) {
			// Circular buffer
			event_ptr -= MAX_EVENTS;
		}
		events[ event_ptr ] = ev;
		num_events++;
	}
	return event_ptr;
}

static uint32_t recover_event() {
	uint32_t ret = -1;
	if( num_events > 0 ) {
		ret = events[ event_start ];
		event_start++;
		if( event_start == MAX_EVENTS ) event_start = 0;
		num_events--;
	}
	return ret;
}

// Timer irq
void tim2_isr(void)
{
    static uint8_t readwrite = 0;
    static uint32_t row = 1;
    static uint8_t nrow = 0;

    // Send row selector or read columns (alternated)
    if( readwrite == 0 ) {
        // Send row selector to GPIOA 0-3
        uint32_t prev_row = row >> 1;
        if( prev_row == 0 ) prev_row = 0x80000000;
        else prev_row = prev_row << 16;

        GPIOA_BSRR = row | prev_row;  // Set row bit, reset prev_row bit on output pin
    }
    else /* 1 */ {
        // Read columns for current row and get to the next row
        uint8_t columns = ( GPIOB_IDR >> 5 ) & 0xFF;
        // Check if any column changed and create MIDI event if so (note on/off)
        if( keyboard[ nrow ] ^ columns ) {

            // row: 0 - C
            //      1 - C#
            //      2 - D
            //      3 - D#
            //      4 - E
            //      5 - F
            //      6 - F#
            //      7 - G
            //      8 - G#
            //      9 - A
            //     10 - A#
            //     11 - B
            //     12-15 : other funcs
            //
            //  https://www.midi.org/specifications/item/table-1-summary-of-midi-message
            if( row < 12 ) {

                // It is a note: note-on or note-off message
				uint8_t scale = 1;
			    uint8_t offset_scale = 0;
				uint8_t base_note_midi_code = 12*transpose + row;   // Scale 0+transpose: midi code for note C = 12*(0+transpose)

				while( scale ) {

				    // Byte 0: ccccnnnn - midi jack of endpoint, nnnn = code, 8: note off / 9: note on
					// Byte 1/2/3: MIDI msg
				    uint32_t midi_event;
					
					if( columns & scale ) {
						// Note ON - 0 --> 1
						uint8_t note_code = base_note_midi_code + offset_scale;
						//XXX endianness!? 0x90 [note ON,chn 0] 0x3C [note C, 60] 0x40 [vel normal]
						midi_event = 0x09900000 + ( note_code << 8UL ) + 0x40;  // Velocity: 0x40, "normal"
						enqueue_event( midi_event );
					} 
					else {
						// Note OFF - 1 --> 0
						uint8_t note_code = base_note_midi_code + offset_scale;
						//XXX endianness!?
						midi_event = 0x08800000 + ( note_code << 8UL ) + 0x40;  // Velocity: 0x40, "normal"
						enqueue_event( midi_event );
					}
					scale *= 2;
					offset_scale += 12;
				}
				
            }
            else {
                // It is a control: row 12-15. (Control change - 0xBn)
            }
        }

        // Store new keyboard value
        keyboard[ nrow ] = columns;
        row *= 2;
        if( row & 0x10000 )
           row = 1;
        nrow++;
        nrow &= 0x0F;
    }

    // Switch function read -> write -> read -> ...
    readwrite ^= 1;

    // Clear interrupt flag
    TIM_SR(TIM2) &= ~TIM_SR_UIF;
}


// Send all queued events as MIDI messages through USB
void send_midi_events() {
	uint32_t ev = -1;
	while( ( ev = recover_event() ) != -1 ) {
		while (usbd_ep_write_packet(usbd_dev, 0x81, &ev, sizeof(buf)) == 0);
	}
}


int main(void)
{
   usbd_device *usbd_dev;

   rcc_clock_setup_in_hse_8mhz_out_72mhz();

   // NVIC - timer interrupt
   nvic_enable_irq( NVIC_TIM2_IRQ );
   nvic_set_priority( NVIC_TIM2_IRQ, 1 );

   rcc_periph_clock_enable( RCC_GPIOA );
   rcc_periph_clock_enable( RCC_GPIOB );
   rcc_periph_clock_enable( RCC_GPIOC );
   rcc_periph_clock_enable( RCC_TIM2 );

   // Timer: keyboard scan
   memset( keyboard, 0, 16 );

   // Time base unit: TIM_CNT : counter value register
   //                 TIM_PSC : prescaler register
   //                 TIM_ARR : auto reload register
   //                 TIM_CR1 : CEN: activate clock
   //                 TIM_EGR : config timer

   // Set timer start value
   TIM_CNT(TIM2) = 1;

   // Set timer prescaler. 72MHz/1440 => 50000 counts per second
   TIM_PSC(TIM2) = 1440;

   // End timer value. When reached an interrupt is generated
   TIM_ARR(TIM2) = 50000;

   // Update interrupt enable
   TIM_DIER(TIM2) |= TIM_DIER_UIE;

   // Start timer
   TIM_CR1(TIM2) |= TIM_CR1_CEN;

   // LED
   gpio_set_mode( GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO13 );

   gpio_set(GPIOC, GPIO13); // led off

   // Button pin: PA00
   gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);

   // Scan row: output to PA01 to PA04
   gpio_set_mode( GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO1 );
   gpio_set_mode( GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO2 );
   gpio_set_mode( GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO3 );
   gpio_set_mode( GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO4 );

   // Scan columns: input to PB05 to PB12
   gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO5);
   gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO6);
   gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO7);
   gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO8);
   gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO9);
   gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO10);
   gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO11);
   gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO12);

   // USB
   usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config,
         usb_strings, 3,
         usbd_control_buffer, sizeof(usbd_control_buffer));

   usbd_register_set_config_callback(usbd_dev, usbmidi_set_config);

   while (1) {
      usbd_poll(usbd_dev);
      button_poll(usbd_dev);
      send_midi_events();
   }

   return 0;
}
