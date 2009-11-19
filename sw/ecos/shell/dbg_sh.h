//
//
//


// #include <pkgconf/hal.h>
// #include <cyg/hal/hal_if.h>
// #include <cyg/hal/hal_tables.h>


// // CLI support functions
// // externC bool parse_num(char *s, unsigned long *val, char **es, char *delim);
// // externC bool parse_bool(char *s, bool *val);

// typedef void cmd_fun(int argc, char *argv[]);
// struct cmd {
//     char    *str;
//     char    *help;
//     char    *usage;
//     cmd_fun *fun;
//     struct cmd *sub_cmds, *sub_cmds_end;
// } CYG_HAL_TABLE_TYPE;
// // externC struct cmd *cmd_search(struct cmd *tab, struct cmd *tabend, char *arg);
// // externC void        cmd_usage(struct cmd *tab, struct cmd *tabend, char *prefix);
// #define RedBoot_cmd(_s_,_h_,_u_,_f_) cmd_entry(_s_,_h_,_u_,_f_,0,0,RedBoot_commands)
// #define RedBoot_nested_cmd(_s_,_h_,_u_,_f_,_subs_,_sube_) cmd_entry(_s_,_h_,_u_,_f_,_subs_,_sube_,RedBoot_commands)
// #define _cmd_entry(_s_,_h_,_u_,_f_,_subs_,_sube_,_n_)                                   \
// cmd_fun _f_;                                                      \
// struct cmd _cmd_tab_##_f_ CYG_HAL_TABLE_QUALIFIED_ENTRY(_n_,_f_) = {_s_, _h_, _u_, _f_, _subs_, _sube_};
// #define cmd_entry(_s_,_h_,_u_,_f_,_subs_,_sube_,_n_)                                   \
// extern _cmd_entry(_s_,_h_,_u_,_f_,_subs_,_sube_,_n_)
// #define local_cmd_entry(_s_,_h_,_u_,_f_,_n_)                             \
// static _cmd_entry(_s_,_h_,_u_,_f_,0,0,_n_)

// #define CYGBLD_REDBOOT_MAX_MEM_SEGMENTS 1
// #define CYGNUM_REDBOOT_CMD_LINE_EDITING 16

// #define MAX_ARGV 16

// // Option processing support

// struct option_info {
//     char flag;
//     bool takes_arg;
//     int  arg_type;
//     void *arg;
//     bool *arg_set;
//     char *name;
// };

// #define NUM_ELEMS(s) (sizeof(s)/sizeof(s[0]))

// #define OPTION_ARG_TYPE_NUM 0    // Numeric data
// #define OPTION_ARG_TYPE_STR 1    // Generic string
// #define OPTION_ARG_TYPE_FLG 2    // Flag only


//-----------------------------------------------------------------------------
// String functions. Some of these are duplicates of the same functions in
// the I18N package.

// Validate a hex character
__inline__ static bool
_is_hex(char c)
{
    return (((c >= '0') && (c <= '9')) ||
            ((c >= 'A') && (c <= 'F')) ||            
            ((c >= 'a') && (c <= 'f')));
}

// Convert a single hex nibble
__inline__ static int
_from_hex(char c) 
{
    int ret = 0;

    if ((c >= '0') && (c <= '9')) {
        ret = (c - '0');
    } else if ((c >= 'a') && (c <= 'f')) {
        ret = (c - 'a' + 0x0a);
    } else if ((c >= 'A') && (c <= 'F')) {
        ret = (c - 'A' + 0x0A);
    }
    return ret;
}

// Convert a character to lower case
__inline__ static char
_tolower(char c)
{
    if ((c >= 'A') && (c <= 'Z')) {
        c = (c - 'A') + 'a';
    }
    return c;
}

// Validate alpha
__inline__ static bool
isalpha(int c)
{
    return (((c >= 'a') && (c <= 'z')) || 
            ((c >= 'A') && (c <= 'Z')));
}

// Validate digit
__inline__ static bool
isdigit(int c)
{
    return ((c >= '0') && (c <= '9'));
}

// Validate alphanum
__inline__ static bool
isalnum(int c)
{
    return (isalpha(c) || isdigit(c));
}

