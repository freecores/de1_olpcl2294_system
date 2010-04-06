//
//
//

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include <cyg/kernel/kapi.h>

#include "LPC22xx.h"
#include "lib_dbg_sh.h"
#include "oc_gpio.h"

extern void dbg_sh(void);

static cyg_interrupt int1;
static cyg_handle_t int1_handle;


//
// Interrupt service routine for interrupt 1.
//
cyg_uint32 interrupt_1_isr(
            cyg_vector_t vector,
            cyg_addrword_t data)
{ 
  // Block this interrupt from occurring until
  // the DSR completes.
  cyg_interrupt_mask( vector );
  
  // disable and clear gpio b intr
  OC_GPIO_B_RGPIO_INTE &= 0x7fffffff;
  OC_GPIO_B_RGPIO_INTS &= 0x7fffffff;


  // Tell the processor that we have received
  // the interrupt.
  cyg_interrupt_acknowledge( vector );

  // Tell the kernel that chained interrupt processing
  // is done and the DSR needs to be executed next.
  return( CYG_ISR_HANDLED | CYG_ISR_CALL_DSR );
}

// 
// Deferred service routine for interrupt 1.
// 
void interrupt_1_dsr(
       cyg_vector_t vector,
       cyg_ucount32 count,
       cyg_addrword_t data)
{
  
  hex_led_command( DE1_HEX_LED_INCREMENT, 0);
  
  OC_GPIO_B_RGPIO_INTE |=  0x80000000;
  
  // Allow this interrupt to occur again.
  cyg_interrupt_unmask( vector );
}


/* now declare (and allocate space for) some kernel objects,
   like the two threads we will use */
cyg_thread thread_s[2];		/* space for two thread objects */

char stack[2][4096];		/* space for two 4K stacks */

/* now the handles for the threads */
cyg_handle_t dbg_shell_thread, simple_threadB;

/* and now variables for the procedure which is the thread */
cyg_thread_entry_t dbg_shell;
cyg_thread_entry_t simple_program;

/* we install our own startup routine which sets up threads */
void cyg_user_start(void)
{  
  // enable cs3
  PINSEL2 = 0x0f814924;
  
  // configure BCFG3
  *((unsigned int *)0xFFE0000C) = 0x20007de7;
  
  // reset FPGA
  *((unsigned int *)0x83300000) = 0x00000001;
  cyg_thread_delay(10);

  // configure gpio
  fled_init(0x00000003);
  hex_led_init(0x00);

  cyg_thread_create(4, dbg_shell, (cyg_addrword_t) 0,
		    "DBG Shell", (void *) stack[0], 4096,
		    &dbg_shell_thread, &thread_s[0]);
		    
  cyg_thread_create(4, simple_program, (cyg_addrword_t) 1,
		    "Thread B", (void *) stack[1], 4096,
		    &simple_threadB, &thread_s[1]);

  cyg_thread_resume(dbg_shell_thread);
  cyg_thread_resume(simple_threadB);
  
  
  cyg_vector_t int1_vector = CYGNUM_HAL_INTERRUPT_EINT3;
//   cyg_priority_t int1_priority = CYGNUM_HAL_PRI_HIGH;
  cyg_priority_t int1_priority = 0;

  //
  // Create interrupt 1.
  //
  cyg_interrupt_create(
     int1_vector,
     int1_priority,
     0,
     &interrupt_1_isr,
     &interrupt_1_dsr,
     &int1_handle,
     &int1);

  // Attach the interrupt created to the vector.
  cyg_interrupt_attach( int1_handle );
  
  // configure gpio b
  OC_GPIO_B_RGPIO_INTS &= 0x7fffffff;
  OC_GPIO_B_RGPIO_INTE =  0x80000000;
  OC_GPIO_B_RGPIO_CTRL =  0x00000001;
  
  // configure eint3
  *((unsigned int *)0xE002C004) |= 0x20000000;

  
  // Unmask the interrupt we just configured.
  cyg_interrupt_unmask( int1_vector );
  
}

/* this is a simple program which runs in a thread */
void dbg_shell(cyg_addrword_t data)
{
  int message = (int) data;

  printf("Beginning execution; thread data is %d\n", message);
  
  dbg_sh();

}

/* this is a simple program which runs in a thread */
void simple_program(cyg_addrword_t data)
{

  for (;;) {

    OC_GPIO_B_RGPIO_OUT ^= 0x00000001;
//     hex_led_command( DE1_HEX_LED_INCREMENT, 0);
    
    cyg_thread_delay(100);
  }
  
}


