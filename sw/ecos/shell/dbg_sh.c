//
//
//

#include <stdio.h>
#include "LPC22xx.h"

#include <cyg/error/codes.h>
#include <cyg/io/io.h>
#include <cyg/io/ttyio.h>

// #include <redboot.h>
#include "lib_dbg_sh.h"
#include "parse.h"


CYG_HAL_TABLE_BEGIN( __RedBoot_CMD_TAB__, RedBoot_commands );
CYG_HAL_TABLE_END( __RedBoot_CMD_TAB_END__, RedBoot_commands );
extern struct cmd __RedBoot_CMD_TAB__[], __RedBoot_CMD_TAB_END__;


//--------------------------------------------------------------------------
//  
//

#include "memtest.h"

extern datum memTestDataBus(volatile datum *address);
extern datum *memTestAddressBus(volatile datum *baseAddress, unsigned long nBytes);
extern datum *memTestDevice(volatile datum *baseAddress, unsigned long nBytes);

static void do_memtest (int argc, char *argv[]);
RedBoot_cmd ("memtest", "Test Memory", "-b <location> -l <length>", do_memtest);


static void
do_memtest (int argc, char *argv[])
{
    struct option_info opts[2];
    unsigned long location, length, result;
    bool location_set, length_set;

    init_opts (&opts[0], 'b', true, OPTION_ARG_TYPE_NUM,
               &location, &location_set, "location");
    init_opts (&opts[1], 'l', true, OPTION_ARG_TYPE_NUM,
               &length, &length_set, "length");
    if (!scan_opts (argc, argv, 1, opts, 2, 0, 0, "")) {
        return;
    }
    if (!location_set) {
        printf ("memtest: what location?\n");
        return;
    }
    
    if (!length_set) {
        printf ("memtest: what length?\n");
        return;
    }
    
    result = memTestDataBus( (volatile datum *)location );
    
    if( result == 0 ) {
      printf( "memTestDataBus: passes\n" );
    } else {
      printf( "memTestDataBus: failed with %x\n", result );
      return;
    }

    result = memTestAddressBus( (volatile datum *)location, length );
    
    if( result == NULL ) {
      printf( "memTestAddressBus: passes\n" );
    } else {
      printf( "memTestAddressBus: failed at address %x\n", result );
      return;
    }

    result = memTestDevice( (volatile datum *)location, length );
    
    if( result == NULL ) {
      printf( "memTestDevice: passes\n" );
    } else {
      printf( "memTestDevice: failed at address %x\n", result );
      return;
    }
    
    printf( "memtest: done\n" );

    return;
}


//--------------------------------------------------------------------------
//
//
RedBoot_cmd("iopeek",
	    "Read I/O location",
	    "[-b <location>] [-1|2|4]",
	    do_iopeek
    );
RedBoot_cmd("iopoke",
	    "Write I/O location",
	    "[-b <location>] [-1|2|4] -v <value>",
	    do_iopoke
    );

void
do_iopoke(int argc, char *argv[])
{
    struct option_info opts[5];
    unsigned long base;
    bool base_set, value_set;
    bool set_32bit = false;
    bool set_16bit = false;
    bool set_8bit = false;
    cyg_uint32 value;
    int size = 1;

    init_opts(&opts[0], 'b', true, OPTION_ARG_TYPE_NUM, 
              &base, &base_set, "base address");
    init_opts(&opts[1], 'v', true, OPTION_ARG_TYPE_NUM, 
              &value, &value_set, "valuex");
    init_opts(&opts[2], '4', false, OPTION_ARG_TYPE_FLG,
              &set_32bit, 0, "output 32 bit units");
    init_opts(&opts[3], '2', false, OPTION_ARG_TYPE_FLG,
              &set_16bit, 0, "output 16 bit units");
    init_opts(&opts[4], '1', false, OPTION_ARG_TYPE_FLG,
              &set_8bit, 0, "output 8 bit units");
    if (!scan_opts(argc, argv, 1, opts, 5, 0, 0, "")) {
        return;
    }
    if (!base_set) {
        printf("iopoke what <location>?\n");
        return;
    }
    if (!value_set) { 
        printf("iopoke what <value>?\n");
        return;
    }
    if (set_32bit) {
        size = 4;
    } else if (set_16bit) {
        size = 2;
    } else if (set_8bit) {
        size = 1;
    }

    switch (size) {
    case 4:
        HAL_WRITE_UINT32 ( base, value );
        break;
    case 2:
        HAL_WRITE_UINT16 ( base, value );
        break;
    case 1: 
        HAL_WRITE_UINT8 ( base, value );
        break;
    }
}

void
do_iopeek(int argc, char *argv[])
{
    struct option_info opts[4];
    unsigned long base;
    bool base_set;
    bool set_32bit = false;
    bool set_16bit = false;
    bool set_8bit = false;
    int size = 1, value;

    init_opts(&opts[0], 'b', true, OPTION_ARG_TYPE_NUM, 
              &base, &base_set, "base address");
    init_opts(&opts[1], '4', false, OPTION_ARG_TYPE_FLG,
              &set_32bit, 0, "output 32 bit units");
    init_opts(&opts[2], '2', false, OPTION_ARG_TYPE_FLG,
              &set_16bit, 0, "output 16 bit units");
    init_opts(&opts[3], '1', false, OPTION_ARG_TYPE_FLG,
              &set_8bit, 0, "output 8 bit units");
    if (!scan_opts(argc, argv, 1, opts, 4, 0, 0, "")) {
        return;
    }
    if (!base_set) {
        printf("iopeek what <location>?\n");
        return;
    }
    if (set_32bit) {
      size = 4;
    } else if (set_16bit) {
        size = 2;
    } else if (set_8bit) {
        size = 1;
    }

    switch (size) {
    case 4:
        HAL_READ_UINT32 ( base, value );
        printf("0x%04lx = 0x%08x\n", base, value );
        break;
    case 2:
        HAL_READ_UINT16 ( base, value );
        printf("0x%04lx = 0x%04x\n", base, value );
        break;
    case 1: 
        HAL_READ_UINT8 ( base, value );
        printf("0x%04lx = 0x%02x\n", base, value );
        break;
    }
}


//--------------------------------------------------------------------------
//
//
RedBoot_cmd("help",
            "Help about help?",
            "[<topic>]",
            do_help
    );

void
show_help(struct cmd *cmd, struct cmd *cmd_end, char *which, char *pre)
{
    bool show;
    int len = 0;

    if (which) {
        len = strlen(which);
    }
    while (cmd != cmd_end) {
        show = true;
        if (which && (strncasecmp(which, cmd->str, len) != 0)) {
            show = false;
        }
        if (show) {
            printf("%s\n  %s %s %s\n", cmd->help, pre, cmd->str, cmd->usage);
            if ((cmd->sub_cmds != (struct cmd *)0) && (which != (char *)0)) {
                show_help(cmd->sub_cmds, cmd->sub_cmds_end, 0, cmd->str);
            }
        }
        cmd++;
    }
}

void
do_help(int argc, char *argv[])
{
    struct cmd *cmd;
    char *which = (char *)0;

    if (!scan_opts(argc, argv, 1, 0, 0, (void *)&which, OPTION_ARG_TYPE_STR, "<topic>")) {
        printf("Invalid argument\n");
        return;
    }
    cmd = __RedBoot_CMD_TAB__;
    show_help(cmd, &__RedBoot_CMD_TAB_END__, which, "");
    return;
}


void dbg_sh(void)
{
  char buffer[256];

  char *command;
  struct cmd *cmd;

  int argc;
  char *argv[16];

  while(1)
  {
    printf( "dbg_sh> " );

    gets( buffer );
    command = buffer;
    
    if( strlen(command) > 0 )
    {
      if ((cmd = parse(&command, &argc, &argv[0])) != (struct cmd *)0)
      {
          (cmd->fun)(argc, argv);
      } else
      {
          printf("** Error: Illegal command: \"%s\"\n", argv[0]);
      }
    } 
  }
}


