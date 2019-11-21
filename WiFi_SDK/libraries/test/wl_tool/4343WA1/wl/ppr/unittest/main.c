/*
 * Basic unit test for ppr module
 *
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *
 * $Id: test_ppr.c xxxxxx 2013-10-30 06:00:44Z emanuell,shaib $
 */


/* ******************************************************************************************************************
************* Definitions for module components to be tested with Check  tool ***************************************
********************************************************************************************************************/



#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "test_ppr_size_routine.h"
#include "test_ppr_clean.h"

/* ***************************************************************************************************************
   ************************************* Start of Test Section ***************************************************
   ***************************************************************************************************************/

#include <check.h> /* Includes Check framework */

/*
 * Main flow:
 * 1. Define SRunner object which will aggregate all suites.
 * 2. Adding all suites to SRunner which enables consecutive suite(s) running.
 * 3. Running all suites contained in SRunner.
 */

int main(void){
	int number_failed; /* Count number of failed tests*/
	//Adding suit to SRunner.
	SRunner *sr = srunner_create(ppr_size_routine());
	//Adding another suit to SRunner.
	srunner_add_suite (sr,ppr_clean());
	srunner_run_all (sr, CK_VERBOSE); /* Prints the summary one message per test (passed or failed) */
	number_failed = (int) srunner_ntests_failed(sr); /* count all the failed tests */
	srunner_free(sr);
	return (number_failed == 0) ? EXIT_SUCCESS: EXIT_FAILURE;
}
