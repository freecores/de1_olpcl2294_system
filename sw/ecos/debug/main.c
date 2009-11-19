//
//
//

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include <cyg/kernel/kapi.h>

#include "LPC22xx.h"
#include "lib_dbg_sh.h"

extern void dbg_sh(void);


/* now declare (and allocate space for) some kernel objects,
   like the two threads we will use */
cyg_thread thread_s[2];		/* space for two thread objects */

char stack[2][4096];		/* space for two 4K stacks */

/* now the handles for the threads */
cyg_handle_t simple_threadA, simple_threadB;

/* and now variables for the procedure which is the thread */
cyg_thread_entry_t simple_program;

/* and now a mutex to protect calls to the C library */
cyg_mutex_t cliblock;

/* we install our own startup routine which sets up threads */
void cyg_user_start(void)
{
//   printf("Entering twothreads' cyg_user_start() function\n");
  
  // enable cs3
  PINSEL2 = 0x0f814924;
  
  // configure BCFG3
  *((unsigned int *)0xFFE0000C) = 0x20007de7;

  
  cyg_mutex_init(&cliblock);

  cyg_thread_create(4, simple_program, (cyg_addrword_t) 0,
		    "Thread A", (void *) stack[0], 4096,
		    &simple_threadA, &thread_s[0]);
//   cyg_thread_create(4, simple_program, (cyg_addrword_t) 1,
// 		    "Thread B", (void *) stack[1], 4096,
// 		    &simple_threadB, &thread_s[1]);

  cyg_thread_resume(simple_threadA);
//   cyg_thread_resume(simple_threadB);
}

/* this is a simple program which runs in a thread */
void simple_program(cyg_addrword_t data)
{
  int message = (int) data;
  int delay;

  printf("Beginning execution; thread data is %d\n", message);
  
  dbg_sh();

}


int main(void)
{
  
  

  return( -1 );
}

