@/**************************************************************************/
@/*                                                                        */
@/*            Copyright (c) 1996-2011 by Express Logic Inc.               */
@/*                                                                        */
@/*  This software is copyrighted by and is the sole property of Express   */
@/*  Logic, Inc.  All rights, title, ownership, or other interests         */
@/*  in the software remain the property of Express Logic, Inc.  This      */
@/*  software may only be used in accordance with the corresponding        */
@/*  license agreement.  Any unauthorized use, duplication, transmission,  */
@/*  distribution, or disclosure of this software is expressly forbidden.  */
@/*                                                                        */
@/*  This Copyright notice may not be removed or modified without prior    */
@/*  written consent of Express Logic, Inc.                                */
@/*                                                                        */
@/*  Express Logic, Inc. reserves the right to modify this software        */
@/*  without notice.                                                       */
@/*                                                                        */
@/*  Express Logic, Inc.                     info@expresslogic.com         */
@/*  11423 West Bernardo Court               http://www.expresslogic.com   */
@/*  San Diego, CA  92127                                                  */
@/*                                                                        */
@/**************************************************************************/
@
@
@
@ Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 @ Cypress Semiconductor Corporation. All Rights Reserved.
 @ This software, including source code, documentation and related
 @ materials ("Software"), is owned by Cypress Semiconductor Corporation
 @ or one of its subsidiaries ("Cypress") and is protected by and subject to
 @ worldwide patent protection (United States and foreign),
 @ United States copyright laws and international treaty provisions.
 @ Therefore, you may use this Software only as provided in the license
 @ agreement accompanying the software package from which you
 @ obtained this Software ("EULA").
 @ If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 @ non-transferable license to copy, modify, and compile the Software
 @ source code solely for use in connection with Cypress's
 @ integrated circuit products. Any reproduction, modification, translation,
 @ compilation, or representation of this Software except as specified
 @ above is prohibited without the express written permission of Cypress.
 @ 
 @ Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 @ EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 @ WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 @ reserves the right to make changes to the Software without notice. Cypress
 @ does not assume any liability arising out of the application or use of the
 @ Software or any product or circuit described in the Software. Cypress does
 @ not authorize its products for use in any products where a malfunction or
 @ failure of the Cypress product may reasonably be expected to result in
 @ significant property damage, injury or death ("High Risk Product"). By
 @ including Cypress's product in a High Risk Product, the manufacturer
 @ of such system or application assumes all risk of such use and in doing
 @ so agrees to indemnify Cypress against all liability.
@

    .arm

SVC_MODE        =       0xD3                    @ Disable IRQ/FIQ SVC mode
IRQ_MODE        =       0xD2                    @ Disable IRQ/FIQ IRQ mode
FIQ_MODE        =       0xD1                    @ Disable IRQ/FIQ FIQ mode
SYS_MODE        =       0xDF                    @ Disable IRQ/FIQ SYS mode
@FIQ_STACK_SIZE  =       512                     @ FIQ stack size
@IRQ_STACK_SIZE  =       1024                    @ IRQ stack size
@SYS_STACK_SIZE  =       1024                    @ System stack size
@
@
    .global     _tx_thread_system_stack_ptr
    .global     _tx_initialize_unused_memory
    .global     _tx_thread_context_save
    .global     _tx_thread_context_restore
    .global     _tx_timer_interrupt
    .global     __RAM_segment_used_end__
    .global     link_stack_isr_location
    .global     link_stack_isr_end

@
@
@/* Define the 16-bit Thumb mode veneer for _tx_initialize_low_level for
@   applications calling this function from to 16-bit Thumb mode.  */
@
    .text
    .align 2
    .thumb
    .global $_tx_initialize_low_level
    .type   $_tx_initialize_low_level,function
$_tx_initialize_low_level:
     BX        pc                               @ Switch to 32-bit mode
     NOP                                        @
    .arm
     STMFD     sp!, {lr}                        @ Save return address
     BL        _tx_initialize_low_level         @ Call _tx_initialize_low_level function
     LDMFD     sp!, {lr}                        @ Recover saved return address
     BX        lr                               @ Return to 16-bit caller
@
@
    .text
    .align 2

@/**************************************************************************/
@/*                                                                        */
@/*  FUNCTION                                               RELEASE        */
@/*                                                                        */
@/*    _tx_initialize_low_level                          Cortex-R4/GNU     */
@/*                                                           5.0          */
@/*  AUTHOR                                                                */
@/*                                                                        */
@/*    William E. Lamie, Express Logic, Inc.                               */
@/*                                                                        */
@/*  DESCRIPTION                                                           */
@/*                                                                        */
@/*    This function is responsible for any low-level processor            */
@/*    initialization, including setting up interrupt vectors, setting     */
@/*    up a periodic timer interrupt source, saving the system stack       */
@/*    pointer for use in ISR processing later, and finding the first      */
@/*    available RAM memory address for tx_application_define.             */
@/*                                                                        */
@/*  INPUT                                                                 */
@/*                                                                        */
@/*    None                                                                */
@/*                                                                        */
@/*  OUTPUT                                                                */
@/*                                                                        */
@/*    None                                                                */
@/*                                                                        */
@/*  CALLS                                                                 */
@/*                                                                        */
@/*    None                                                                */
@/*                                                                        */
@/*  CALLED BY                                                             */
@/*                                                                        */
@/*    _tx_initialize_kernel_enter           ThreadX entry function        */
@/*                                                                        */
@/*  RELEASE HISTORY                                                       */
@/*                                                                        */
@/*    DATE              NAME                      DESCRIPTION             */
@/*                                                                        */
@/*  01-01-2011     William E. Lamie         Initial Version 5.0           */
@/*                                                                        */
@/**************************************************************************/
@VOID   _tx_initialize_low_level(VOID)
@{
    .global _tx_initialize_low_level
    .type   _tx_initialize_low_level,function
_tx_initialize_low_level:
    PUSH    {lr}
@
@    /* We must be in SVC mode at this point!  */
@
@    /* Setup various stack pointers.  */
@
    LDR     r1, =link_stack_isr_end             @ Get pointer to stack area

#ifdef TX_ENABLE_IRQ_NESTING
@
@    /* Setup the system mode stack for nested interrupt support  */
@
    LDR     r2, =SYS_STACK_SIZE                 @ Pickup stack size
    MOV     r3, #SYS_MODE                       @ Build SYS mode CPSR
    MSR     CPSR_c, r3                          @ Enter SYS mode
    SUB     r1, r1, #1                          @ Backup 1 byte
    BIC     r1, r1, #7                          @ Ensure 8-byte alignment
    MOV     sp, r1                              @ Setup SYS stack pointer
    SUB     r1, r1, r2                          @ Calculate start of next stack
#endif

    LDR     r2, =FIQ_STACK_SIZE                 @ Pickup stack size
    MOV     r0, #FIQ_MODE                       @ Build FIQ mode CPSR
    MSR     CPSR, r0                            @ Enter FIQ mode
    SUB     r1, r1, #1                          @ Backup 1 byte
    BIC     r1, r1, #7                          @ Ensure 8-byte alignment
    MOV     sp, r1                              @ Setup FIQ stack pointer
    SUB     r1, r1, r2                          @ Calculate start of next stack
    LDR     r2, =IRQ_STACK_SIZE                 @ Pickup IRQ stack size
    MOV     r0, #IRQ_MODE                       @ Build IRQ mode CPSR
    MSR     CPSR, r0                            @ Enter IRQ mode
    SUB     r1, r1, #1                          @ Backup 1 byte
    BIC     r1, r1, #7                          @ Ensure 8-byte alignment
    MOV     sp, r1                              @ Setup IRQ stack pointer
    SUB     r3, r1, r2                          @ Calculate end of IRQ stack
    MOV     r0, #SVC_MODE                       @ Build SVC mode CPSR
    MSR     CPSR, r0                            @ Enter SVC mode
    LDR     r2, =link_stack_isr_location        @ Pickup stack bottom
    CMP     r3, r2                              @ Compare the current stack end with the bottom
_stack_error_loop:
    BLT     _stack_error_loop                   @ If the IRQ stack exceeds the stack bottom, just sit here!
@
@    /* Save the system stack pointer.  */
@    _tx_thread_system_stack_ptr = (VOID_PTR) (sp);
@
    LDR     r2, =_tx_thread_system_stack_ptr    @ Pickup stack pointer
    STR     r1, [r2]                            @ Save the system stack
@
@    /* Save the first available memory address.  */
@    _tx_initialize_unused_memory =  (VOID_PTR) _end;
@
    LDR     r1, =__RAM_segment_used_end__       @ Get end of non-initialized RAM area
    LDR     r2, =_tx_initialize_unused_memory   @ Pickup unused memory ptr address
    ADD     r1, r1, #8                          @ Increment to next free word
    STR     r1, [r2]                            @ Save first free memory address
@
@    /* Setup Timer for periodic interrupts.  */
    BL      platform_tick_start
@
@    /* Done, return to caller.  */
@
    POP     {lr}
#ifdef __THUMB_INTERWORK
    BX      lr                                  @ Return to caller
#else
    MOV     pc, lr                              @ Return to caller
#endif
@}

@
@/* Interrupt vector  */
@
    .global irq_vector_external_interrupt
    .global __tx_irq_processing_return
irq_vector_external_interrupt:
@
@    /* Jump to context save to save system context.  */
    B       _tx_thread_context_save
__tx_irq_processing_return:
@
@    /* At this point execution is still in the IRQ mode.  The CPSR, point of
@       interrupt, and all C scratch registers are available for use.  In
@       addition, IRQ interrupts may be re-enabled - with certain restrictions -
@       if nested IRQ interrupts are desired.  Interrupts may be re-enabled over
@       small code sequences where lr is saved before enabling interrupts and
@       restored after interrupts are again disabled.  */
@
@    /* Interrupt nesting is allowed after calling _tx_thread_irq_nesting_start
@       from IRQ mode with interrupts disabled.  This routine switches to the
@       system mode and returns with IRQ interrupts enabled.
@
@       NOTE:  It is very important to ensure all IRQ interrupts are cleared
@       prior to enabling nested IRQ interrupts.  */
#ifdef TX_ENABLE_IRQ_NESTING
    BL      _tx_thread_irq_nesting_start
#endif
    BLX     _tx_platform_irq_demuxer
@
@
@    /* If interrupt nesting was started earlier, the end of interrupt nesting
@       service must be called before returning to _tx_thread_context_restore.
@       This routine returns in processing in IRQ mode with interrupts disabled.  */
#ifdef TX_ENABLE_IRQ_NESTING
    BL      _tx_thread_irq_nesting_end
#endif
@
@    /* Jump to context restore to restore system context.  */
    B       _tx_thread_context_restore

