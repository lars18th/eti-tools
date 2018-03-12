
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <getopt.h>
#include "ts.h"

#define DEBUG(format, ...) fprintf (stderr, "DEBUG: "format"\n", ## __VA_ARGS__)
#define  INFO(format, ...) fprintf (stderr, "INFO:  "format"\n", ## __VA_ARGS__)
#define WARN(format, ...)  fprintf (stderr, "WARN:  "format"\n", ## __VA_ARGS__)
#define ERROR(format, ...) fprintf (stderr, "ERROR: "format"\n", ## __VA_ARGS__)


/*****************************************************************************
 * Main loop
 *****************************************************************************/
static void usage(const char *psz)
{
    fprintf(stderr, "usage: %s [-p pid] [-s offset] [-i <inputfile>] [-o <outputfile>] [-v]\n", psz);
    fprintf(stderr, "       -v: encapsulated bitstream is ETI-NI (V.11), instead of default ETI-NA (G.704)");
    exit(EXIT_FAILURE);
}

int main(int i_argc, char **ppsz_argv)
{
    int c;
    FILE *inputfile=stdin;
    FILE *outputfile=stdout;
    int offset=12, pid=0x0426;
    int i_last_cc = -1;
    int eti_mode = 0;  // 0:ETI-NA(G.704); 1:ETI-NI(V.11); 2:ETI-NI(G.703)??

    static const struct option long_options[] = {
        { "pid",           required_argument, NULL, 'p' },
        { "offset",        required_argument, NULL, 's' },
        { "input",         required_argument, NULL, 'i' },
        { "output",        required_argument, NULL, 'o' },
        { "v11",           no_argument      , NULL, 'v' },
        { 0, 0, 0, 0 }
    };

    while ((c = getopt_long(i_argc, ppsz_argv, "p:s:i:o:vh", long_options, NULL)) != -1)
    {
        switch (c) {
        case 'p':
            pid=strtoul(optarg, NULL, 0);
            if(pid >= 8192) {
                ERROR("bad pid value: %d!", pid);
                exit(1);
            }
            break;

        case 's':
            offset=strtol(optarg, NULL, 0);
            if(offset >= 184 || offset <= -4) {
                ERROR("bad offset value: %d!", offset);
                exit(1);
            }
            break;

        case 'i':
            inputfile = fopen(optarg, "r");
            if(!inputfile) {
                ERROR("cant open input file!");
                exit(1);
            }
            break;

        case 'o':
            outputfile = fopen(optarg, "w");
            if(!outputfile) {
                ERROR("cant open output file!");
                exit(1);
            }
            break;

        case 'v':
            eti_mode = 1;
            break;

        case 'h':
        default:
            usage(ppsz_argv[0]);
        }
    }


    INFO("Using pid: 0x%04x (%d)", pid, pid);
    INFO("Using output format: %s", eti_mode? "ETI-NI(V.11)" : "ETI-NA(G.704)");
    unsigned long int packets=0;
    while (!feof(inputfile) && !ferror(inputfile)) {
        uint8_t p_ts[TS_SIZE];
        size_t i_ret = fread(p_ts, TS_SIZE, 1, inputfile);
        if (i_ret != 1) {
        	WARN("Can't read input ts");
        	break;
        }
        if (ts_validate(p_ts)) {
            if(offset >= 0 &&  ts_get_pid(p_ts)==pid) {
            	if(i_last_cc > 0 && (0x0f & (i_last_cc+1)) != ts_get_cc(p_ts)) {
            		WARN("TS Discontinuity");
            	}
            	i_last_cc = ts_get_cc(p_ts);
            	uint8_t *payload;
                if(ts_get_transporterror(p_ts)) {
                        WARN("TS Packed error!");
                        payload = p_ts + TS_HEADER_SIZE;  // When error is detected, pass the entire payload.
                } else {
                        payload = ts_payload(p_ts);
                }
            	if(offset) {
            		payload+=offset;
            	}

            	if(p_ts+TS_SIZE < payload) {
                	ERROR("payload is out of ts by %ld bytes", payload-p_ts+TS_SIZE);
                	break;
            	}

            	size_t o_ret = fwrite(payload, p_ts+TS_SIZE-payload, 1, outputfile);
                if (o_ret != 1) {
                	WARN("Can't write output ts");
                	break;
                }
                packets++;
            } else if(offset < 0) {
            	uint8_t *payload = &p_ts[TS_HEADER_SIZE + offset];
            	if(p_ts+TS_SIZE < payload || p_ts > payload) {
                	ERROR("payload is out of ts by %ld bytes", payload-p_ts+TS_SIZE);
                	break;
            	}

            	size_t o_ret = fwrite(payload, p_ts+TS_SIZE-payload, 1, outputfile);
                if (o_ret != 1) {
                	WARN("Can't write output ts");
                	break;
                }
                packets++;
            }
        } else {
            do {
                memmove(p_ts, &p_ts[1], TS_SIZE-1);
                size_t i_ret = fread(&p_ts[TS_SIZE-1], 1, 1, inputfile);
                if (i_ret != 1) {
                	WARN("Can't read input ts");
                	break;
                }
            } while (!ts_validate(p_ts) && !feof(inputfile) && !ferror(inputfile));
        }
    }

    if(packets){
    	INFO("Successfully read %ld ts-packets", packets);
    }


//mainErr:
    fclose(inputfile);
    fclose(outputfile);
    return 0;
}
