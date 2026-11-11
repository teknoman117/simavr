/*
	simduino_leonardo.c

	An Arduino Leonardo: an ATmega32U4 at 16MHz running Arduino's Caterina
	bootloader, with its USB port exported over USB/IP. A USB/IP client
	attaches it to the host's USB stack, where avrdude or the Arduino IDE
	upload sketches to it as to a real board. See README.

	This file is part of simavr.

	simavr is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	simavr is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with simavr.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <libgen.h>

#include "sim_avr.h"
#include "sim_elf.h"
#include "sim_hex.h"
#include "sim_gdb.h"
#include "usbip_usb.h"

#define MMCU		"atmega32u4"
#define FREQUENCY	16000000
#define BOOTLOADER	"Caterina-Leonardo-bootloader.hex"
#define FLASH_FILE	"simduino_leonardo_flash.bin"

static volatile sig_atomic_t stop, reset_button;

static void
sig_stop(int sig)
{
	stop = 1;
}

#ifdef SIGQUIT
static void
sig_reset(int sig)
{
	reset_button = 1;
}
#endif

/*
 * The flash survives between runs, as on the board, so that the
 * bootloader finds the sketch it was given last time. Plain stdio, so
 * that it works on Windows too.
 */
static void
flash_load(avr_t * avr, const char * path)
{
	FILE * f = fopen(path, "rb");

	if (!f)
		return;	// first run, the flash is erased
	size_t n = fread(avr->flash, 1, avr->flashend + 1, f);
	fclose(f);
	printf("Flash: %zu bytes from %s\n", n, path);
}

static void
flash_save(avr_t * avr, const char * path)
{
	FILE * f = fopen(path, "wb");

	if (!f || fwrite(avr->flash, 1, avr->flashend + 1, f) != avr->flashend + 1)
		perror(path);
	if (f)
		fclose(f);
}

// .hex (gaps stay as they are) or .elf, into the flash
static void
load(avr_t * avr, const char * path, const char * progname)
{
	elf_firmware_t fw = { .mmcu = MMCU, .frequency = FREQUENCY };

	sim_setup_firmware(path, 0, &fw, progname);
	if (strcmp(fw.mmcu, MMCU))
		fprintf(stderr, "%s: warning, %s is built for %s\n", progname,
				path, fw.mmcu);
	fw.frequency = 0;	// the board's crystal, whatever the .elf says
	avr_load_firmware(avr, &fw);
}

static void
usage(const char * name)
{
	fprintf(stderr,
		"Usage: %s [-a addr] [-p port] [-F flash.bin] [-n] [-r] [-g] [-v]\n"
		"          [sketch]\n"
		"  sketch    .hex or .elf, programmed at 0 as an upload would\n"
		"  -a addr   USB/IP listening address, default 127.0.0.1,\n"
		"            0.0.0.0 lets other machines attach the board\n"
		"  -p port   USB/IP TCP port, default %d\n"
		"  -F file   flash contents kept between runs, default " FLASH_FILE "\n"
		"  -n        start with an erased flash and don't keep it\n"
		"  -r        start as after a press of the reset button: the\n"
		"            bootloader waits for an upload for 8s, a power-on\n"
		"            runs the sketch at once\n"
		"  -g        wait for gdb on port 1234\n"
		"  -v        more logging, repeat for more\n"
		"%s",
		name, USBIP_USB_DEFAULT_PORT,
#ifdef SIGQUIT
		"Ctrl-\\ presses the reset button while it runs.\n"
#else
		""
#endif
		);
	exit(1);
}

int main(int argc, char *argv[])
{
	char boot[1024];
	const char * sketch = NULL;
	const char * flash_file = FLASH_FILE;
	const char * addr = NULL;
	uint16_t port = 0;
	int gdb = 0, verbose = 0, button = 0;
	int opt;
	avr_t * avr;

	while ((opt = getopt(argc, argv, "a:p:F:nrgvh")) != -1) {
		switch (opt) {
			case 'a': addr = optarg; break;
			case 'p': port = strtoul(optarg, NULL, 0); break;
			case 'F': flash_file = optarg; break;
			case 'n': flash_file = NULL; break;
			case 'r': button = 1; break;
			case 'g': gdb = 1; break;
			case 'v': verbose++; break;
			default: usage(argv[0]);
		}
	}
	if (optind < argc)
		sketch = argv[optind++];
	if (optind < argc)
		usage(argv[0]);
	// the bootloader sits next to the obj-* directory of the binary
	snprintf(boot, sizeof(boot), "%s/../%s", dirname(strdup(argv[0])),
			BOOTLOADER);

	avr = avr_make_mcu_by_name(MMCU);
	if (!avr) {
		fprintf(stderr, "%s: Error creating the AVR core\n", argv[0]);
		exit(1);
	}
	// a Leonardo's BOOTRST and BOOTSZ, programmed: Caterina's 2048 words
	// boot section at 0x7000, where every reset starts
	avr->fuse[avr->boot_fuse] &= ~(AVR_FUSE_BOOTRST | AVR_FUSE_BOOTSZ);
	avr_init(avr);
	avr->frequency = FREQUENCY;
	avr->log = LOG_WARNING + verbose;

	if (flash_file)
		flash_load(avr, flash_file);
	load(avr, boot, argv[0]);
	if (sketch)
		load(avr, sketch, argv[0]);
	// the bootloader writes the application with SPM
	avr->codeend = avr->flashend;
	if (button) {
		avr_regbit_clear(avr, avr->reset_flags.porf);
		avr_regbit_set(avr, avr->reset_flags.extrf);
	}

	// even if not setup at startup, activate gdb if crashing
	avr->gdb_port = 1234;
	if (gdb) {
		avr->state = cpu_Stopped;
		avr_gdb_init(avr);
	}

	if (!usbip_usb_init(avr, addr, port))
		exit(1);
	printf("Arduino Leonardo, USB/IP server on %s port %u. To attach it:\n"
			"  Linux:   sudo modprobe vhci-hcd\n"
			"           sudo usbip attach -r 127.0.0.1 -b %s\n"
			"  Windows: usbip attach -r 127.0.0.1 -b %s (usbip-win2,"
			" from an administrator prompt)\n",
			addr ? addr : "127.0.0.1", port ? port : USBIP_USB_DEFAULT_PORT,
			USBIP_USB_BUSID, USBIP_USB_BUSID);
	fflush(stdout);

	signal(SIGINT, sig_stop);
	signal(SIGTERM, sig_stop);
#ifdef SIGQUIT
	signal(SIGQUIT, sig_reset);
#endif
	while (!stop) {
		int state = avr_run(avr);
		if (state == cpu_Done || state == cpu_Crashed)
			break;
		if (reset_button) {
			reset_button = 0;
			printf("Reset button\n");
			fflush(stdout);
			avr_regbit_clear(avr, avr->reset_flags.porf);
			avr_regbit_clear(avr, avr->reset_flags.borf);
			avr_regbit_clear(avr, avr->reset_flags.wdrf);
			avr_regbit_set(avr, avr->reset_flags.extrf);
			avr_reset(avr);
		}
	}
	if (flash_file)
		flash_save(avr, flash_file);
	avr_terminate(avr);
	return 0;
}
