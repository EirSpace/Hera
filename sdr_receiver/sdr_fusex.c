/*
 * rtl_fusex, the GPS receiver of the EirSpace's experimental rocket Hera
 * Copyright (C) 2013 by EirSpace <contact@eirspace.fr>
 *
 * Based on the rtl-sdr project available at http://sdr.osmocom.org/trac/wiki/rtl-sdr
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <sys/types.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef _WIN32
#include <unistd.h>
#else
#include <Windows.h>
#include <io.h>
#include <fcntl.h>
#include "getopt/getopt.h"
#endif

#include "rtl-sdr.h"

// Config
#define DEFAULT_BITRATE     27
#define DEBOUNCING_FILTER   10
#define BITRATE_DETECT      0
#define PARITY_CHECK        1


// Other consts
#define MAX_FILENAME_LENGTH 100
#define DEFAULT_SAMPLE_RATE 2048000
#define DEFAULT_ASYNC_BUF_NUMBER 32
#define DEFAULT_BUF_LENGTH (16 * 16384)
#define MINIMAL_BUF_LENGTH 512
#define MAXIMAL_BUF_LENGTH (256 * 16384)
#define BITRATE_DETECT_START 5
#define BITRATE_DETECT_STOP 20
#define DEBOUNCING_FILTER_BUFFER_LENGTH DEBOUNCING_FILTER + 2
#define BITRATE_TOLERENCE_RATIO 0.5
#define BIT_BUFFER_LENGTH 12
#define MAX_MESSAGE_LENGTH 100
#define STATE_BITRATE 0
#define STATE_FRAME 1
#define STATE_INTERFRAME 2


static int do_exit = 0;
static uint32_t bytes_to_read = 512;
static rtlsdr_dev_t *dev = NULL;

void usage(void)
{
    fprintf(stderr,
        "sdr_fusex, a RS232 receiver based on the RTL2932 sdr chip\n\n"
        "Usage:\t [-f frequency_to_tune_to (Hz)]\n"
        "\t[-d device_index (default: 0)]\n"
        "\t[-g gain (default: 0 for auto)]\n"
        "\t[-D print debug infos]\n"
        "\t[-R print radio]\n"
        "\tfilename (a '-' dumps samples to stdout)\n\n");
    exit(1);
}

// Signal handling
#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
    if (CTRL_C_EVENT == signum) {
        fprintf(stderr, "Signal caught, exiting!\n");
        do_exit = 1;
        rtlsdr_cancel_async(dev);
        return TRUE;
    }
    return FALSE;
}
#else
static void sighandler(int signum)
{
    fprintf(stderr, "Signal caught, exiting!\n");
    do_exit = 1;
    rtlsdr_cancel_async(dev);
}
#endif


int main(int argc, char **argv) {
#ifndef _WIN32
    struct sigaction sigact;
#endif
    char filename[MAX_FILENAME_LENGTH];
    int n_read;
    int r;
    int gain = 0;
    FILE *file;
    uint8_t *buffer;
    uint32_t dev_index = 0;
    uint32_t frequency = 869455000;
    uint32_t samp_rate = DEFAULT_SAMPLE_RATE;
    uint32_t out_block_size = 512;
    int device_count;
    char vendor[256], product[256], serial[256];
    int sum = 0;
    int n_values_per_average = 16;
    int n_averages_per_gap = 8;
    int n_gaps_per_bit = 4;
    int averages[n_averages_per_gap];
    int bit_gap_trigger = 1900;
    int n_same = 0;
    int sub_bit = 0;
    int sub_bit_old = 0;
    int bit = -1;
    int bit_buffer[BIT_BUFFER_LENGTH];
    int bit_buffer_n = 0;
    int bit_buffer_parity = 0;
    int parity_check_even = 1;
    int byte = 0;
    int state = STATE_BITRATE;
    int detect_bitrate = BITRATE_DETECT;
    int bitrate = DEFAULT_BITRATE;
    int bitrate_tolerence = 0;
    int bitrate_n = 0;
    int bitrate_vals[BITRATE_DETECT_STOP];
    int parity_check = PARITY_CHECK;
    int sub_bit_buffer[DEBOUNCING_FILTER_BUFFER_LENGTH];
    int first_interframe_detected = 0;
    int msg_n = 1;
    char msg_buffer[MAX_MESSAGE_LENGTH];
    int msg_buffer_n = 0;
    int debug = 0;
    int debug_radio = 0;
    int debug_messages = 1;

    // Args
    char opt = '\0';
    strcpy(filename, "-");
    for (int i = 1; i < argc; i++) {
        opt = argv[i][1];
        switch (opt) {
        case 'f':
            i++;
            if (i == argc) {
                usage();
                break;
            }
            frequency = (uint32_t)atof(argv[i]);
            break;

        case 'd':
            i++;
            if (i == argc) {
                usage();
                break;
            }
            dev_index = atoi(argv[i]);
            break;

        case 'g':
            i++;
            if (i == argc) {
                usage();
                break;
            }
            gain = (int)(atof(argv[i]) * 10); // Tenths of a dB
            break;

        case 'D':
            debug = 1;
            break;

        case 'R':
            debug_radio = 1;
            break;

        case 'b':
            detect_bitrate = 1 - BITRATE_DETECT;
            break;

        default:
            printf("Bad argument %s\n", argv[i]);
            usage();
            break;
        }
    }

    if (out_block_size < MINIMAL_BUF_LENGTH || out_block_size > MAXIMAL_BUF_LENGTH ){
        fprintf(stderr, "Output block size wrong value, falling back to default\n");
        fprintf(stderr, "Minimal length: %u\n", MINIMAL_BUF_LENGTH);
        fprintf(stderr, "Maximal length: %u\n", MAXIMAL_BUF_LENGTH);
        out_block_size = DEFAULT_BUF_LENGTH;
    }

    buffer = malloc(out_block_size * sizeof(uint8_t));

    // Device probe
    device_count = rtlsdr_get_device_count();
    if (!device_count) {
        fprintf(stderr, "No supported devices found.\n");
        exit(1);
    }

    if (debug) {
        fprintf(stderr, "Found %d device(s):\n", device_count);
    }
    for (int i = 0; i < device_count; i++) {
        rtlsdr_get_device_usb_strings(i, vendor, product, serial);
        if (debug) {
            fprintf(stderr, "  %d:  %s, %s, SN: %s\n", i, vendor, product, serial);
        }
    }
    if (debug) {
        fprintf(stderr, "\n");
    }

    if (debug) {
        fprintf(stderr, "Using device %d: %s\n", dev_index, rtlsdr_get_device_name(dev_index));
    }

    r = rtlsdr_open(&dev, dev_index);
    if (r < 0) {
        fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
        exit(1);
    }

#ifndef _WIN32
    sigact.sa_handler = sighandler;
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigaction(SIGINT, &sigact, NULL);
    sigaction(SIGTERM, &sigact, NULL);
    sigaction(SIGQUIT, &sigact, NULL);
    sigaction(SIGPIPE, &sigact, NULL);
#else
    SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif

    // Set the sample rate
    r = rtlsdr_set_sample_rate(dev, samp_rate);
    if (r < 0) {
        fprintf(stderr, "WARNING: Failed to set sample rate.\n");
    }

    // Set the frequency
    r = rtlsdr_set_center_freq(dev, frequency);
    if (r < 0) {
        fprintf(stderr, "WARNING: Failed to set center freq.\n");
    } else {
        if (debug) {
            fprintf(stderr, "Tuned to %u Hz.\n", frequency);
        }
    }

    if (0 == gain) {
         // Enable automatic gain
        r = rtlsdr_set_tuner_gain_mode(dev, 0);
        if (r < 0) {
            fprintf(stderr, "WARNING: Failed to enable automatic gain.\n");
        }
    } else {
        // Enable manual gain
        r = rtlsdr_set_tuner_gain_mode(dev, 1);
        if (r < 0) {
            fprintf(stderr, "WARNING: Failed to enable manual gain.\n");
        }

        // Set the tuner gain
        r = rtlsdr_set_tuner_gain(dev, gain);
        if (r < 0) {
            fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
        } else {
            fprintf(stderr, "Tuner gain set to %f dB.\n", gain/10.0);
        }
    }

    // Output file
    int out = 0;
    if(strcmp(filename, "-") == 0) { // Write samples to stdout
        file = stdout;
#ifdef _WIN32
    _setmode(_fileno(stdin), _O_BINARY);
#endif
    } else {
        file = fopen(filename, "wb");
        if (!file) {
            fprintf(stderr, "Failed to open %s\n", filename);
            out = 1;
        }
    }

    if (!out) {
        // Init buffers
        for (int i = 0; i < DEBOUNCING_FILTER_BUFFER_LENGTH; i++) {
            sub_bit_buffer[i] = 0;
        }
        for (int i = 0; i < BIT_BUFFER_LENGTH; i++) {
            bit_buffer[i] = 0;
        }
        for (int i = 0; i < MAX_MESSAGE_LENGTH; i++) {
            msg_buffer[i] = 0;
        }

        // Reset endpoint before we start reading from it (mandatory)
        r = rtlsdr_reset_buffer(dev);
        if (r < 0) {
            fprintf(stderr, "WARNING: Failed to reset buffers.\n");
        }

        if (debug) {
            fprintf(stderr, "Reading samples in sync mode...\n");
        } else {
            printf("Ready!\n");
        }
        while (!do_exit) {

            // Read bytes
            r = rtlsdr_read_sync(dev, buffer, out_block_size, &n_read);
            if (r < 0) {
                fprintf(stderr, "WARNING: sync read failed.\n");
                break;
            }
            if ((bytes_to_read > 0) && (bytes_to_read < (uint32_t)n_read)) {
                n_read = bytes_to_read;
                do_exit = 1;
            }

            // Handle data
            for (int gap = 0; gap < n_gaps_per_bit; gap++) {
                // Compute the averages
                for (int average = 0; average < n_averages_per_gap; average++) {
                    sum = 0;
                    for (int value = (gap * n_averages_per_gap + average) * n_values_per_average; value < (gap * n_averages_per_gap + (average + 1)) * n_values_per_average; value++) {
                        sum += buffer[value];
                    }
                    averages[average] = sum;
                    
                    // Show the graph
                    if (debug_radio) {
                        int i = 0;
                        for (; i < sum / 30; i++) {
                            printf(" ");
                        }
                        printf("x");
                        for(; i < 200; i++) {
                            printf(" ");
                        }
                        printf("%d\n", sum);
                    }
                }

                // Compute the gap
                sum = 0;
                for (int average = 0; average < n_averages_per_gap - 1; average++) {
                    sum += abs(averages[average + 1] - averages[average]);
                }
                if (sum < bit_gap_trigger) {
                    sub_bit = 1;
                } else {
                    sub_bit = 0;
                }
                // printf("%d\n", sum);

                // Debouncing
                for (int i = DEBOUNCING_FILTER_BUFFER_LENGTH - 1; i > 0; i--) {
                    sub_bit_buffer[i] = sub_bit_buffer[i - 1];
                }
                sub_bit_buffer[0] = sub_bit;
                if (sub_bit_buffer[0] == sub_bit_buffer[DEBOUNCING_FILTER_BUFFER_LENGTH - 1]) {
                    // Count the number of inverted sub-bits in the buffer
                    int tmp = 0;
                    for (int i = 0; i < DEBOUNCING_FILTER_BUFFER_LENGTH; i++) {
                        if (sub_bit_buffer[i] == 1 - sub_bit_buffer[0]) {
                            tmp++;
                        }
                    }
                    // Check if a bounce is detected
                    if (tmp > 0) {
                        // Erase it
                        for (int i = 0; i < DEBOUNCING_FILTER_BUFFER_LENGTH; i++) {
                            sub_bit_buffer[i] = sub_bit_buffer[0];
                        }
                    }
                }
                sub_bit = sub_bit_buffer[DEBOUNCING_FILTER_BUFFER_LENGTH - 1];

                // Bitrate detection
                if (state == STATE_BITRATE) {
                    if (!detect_bitrate) {
                        bitrate = DEFAULT_BITRATE;
                        if (debug) {
                            printf("Bitrate used : %d\n", bitrate);
                        }
                        n_same = 0;
                        bitrate_tolerence = ((float)bitrate) * BITRATE_TOLERENCE_RATIO;
                        state = STATE_FRAME;
                    } else {
                        if (sub_bit != sub_bit_old) {
                            if (debug) {
                                printf("%d %d\n", sub_bit, n_same);
                            }
                            if (bitrate_n < BITRATE_DETECT_STOP) {
                                bitrate_vals[bitrate_n] = n_same;
                                bitrate_n++;
                            } else {
                                sum = 0;
                                for (int i = BITRATE_DETECT_START; i < BITRATE_DETECT_STOP; i++) {
                                    sum += bitrate_vals[i];
                                }
                                bitrate = round(((float)sum) / (BITRATE_DETECT_STOP - BITRATE_DETECT_START));
                                bitrate_tolerence = ((float)bitrate) * BITRATE_TOLERENCE_RATIO;
                                if (debug) {
                                    printf("Bitrate detected : %d\n", bitrate);
                                    printf("Starting reception\n");
                                }
                                state = STATE_FRAME;
                            }
                            sub_bit_old = sub_bit;
                            n_same = 0;
                        }
                    }
                } else {
                    // Bit detection
                    bit = -1;
                    if (sub_bit != sub_bit_old) {
                        bit = sub_bit;
                        sub_bit_old = sub_bit;
                        n_same = 0;
                    } else if (n_same > bitrate + bitrate_tolerence) {
                        bit = sub_bit;
                        sub_bit_old = sub_bit;
                        n_same = bitrate_tolerence;
                    }

                    // Check if a bit was detected
                    if (bit >= 0) {
                        if (state == STATE_INTERFRAME) {
                            if (bit == 0) {
                                state = STATE_FRAME;
                                if (debug) {
                                    printf("Frame : ");
                                }
                            }
                            if (debug) {
                                // printf("Interframe...");
                                // fflush(stdout);
                            }
                        } else if (state == STATE_FRAME) {
                            // printf("%d", bit);
                            if (debug) {
                                printf("%d", sub_bit);
                                fflush(stdout);
                            }

                            // Add the bit to the buffer
                            bit_buffer[bit_buffer_n] = bit;
                            bit_buffer_n++;

                            // When a byte is complete, decode it
                            if (bit_buffer_n == BIT_BUFFER_LENGTH) {
                                byte = 0;
                                bit_buffer_parity = 0;
                                for (int i = 0; i < 8; i++) {
                                    byte += bit_buffer[i] << i;
                                    if (bit_buffer[i] == 1) {
                                        bit_buffer_parity = 1 - bit_buffer_parity;
                                    }
                                }
                                bit_buffer_n = 0;

                                // FF byte => interframe
                                if (byte == 255) {
                                    state = STATE_INTERFRAME;
                                    if (!first_interframe_detected) {
                                        first_interframe_detected = 1;
                                        // printf("1 ");
                                    } else {
                                        msg_buffer[msg_buffer_n] = '\0';

                                        if (debug) {
                                            printf(" [Interframe...]\n");
                                        }
                                        if (debug_messages) {
                                            // Print message
                                            printf("F%d: \"%s\"\n", msg_n, msg_buffer);
                                            if (debug) {
                                                printf("\n");
                                            }
                                        }

                                        msg_n++;
                                        msg_buffer_n = 0;
                                    }
                                } else if (first_interframe_detected) {
                                    // Print char
                                    if (debug) {
                                        printf(" (%d 0x", byte);
                                        if (byte <= 0xf) {
                                            printf("0");
                                        }
                                        printf("%x ", byte);
                                        if (byte < 32 || byte > 126) {
                                            printf("???) ");
                                        } else {
                                            printf("'%c') ", byte);
                                        }
                                    }

                                    // Parity check
                                    if (parity_check && ((parity_check_even && (bit_buffer_parity != bit_buffer[8])) || (!parity_check_even && (bit_buffer_parity != 1 - bit_buffer[8])))) {
                                        byte = '?';
                                        if (debug) {
                                            printf(" [Parity check failed] ");
                                        }
                                    }

                                    // Char check
                                    if (byte < 32 || byte > 126) {
                                        byte = '?';
                                    }

                                    // Store char into buffer
                                    msg_buffer[msg_buffer_n] = byte;
                                    msg_buffer_n++;
                                }
                            }
                        }
                    }
                }
                n_same += 1;
            }

            if ((uint32_t)n_read < out_block_size) {
                fprintf(stderr, "Short read, samples lost, exiting!\n");
                break;
            }
        }

        if (do_exit) {
            fprintf(stderr, "\nUser cancel, exiting...\n");
        } else {
            fprintf(stderr, "\nLibrary error %d, exiting...\n", r);
        }

        if (file != stdout) {
            fclose(file);
        }

        rtlsdr_close(dev);
        free(buffer);
    }
    return r >= 0 ? r : -r;
}
