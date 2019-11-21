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

#include <typedefs.h>
#include <bcmendian.h>
#include <bcmwifi_channels.h>
#include <wlc_ppr.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "test_ppr_size_routine.h"

/* ***************************************************************************************************************
   ************************************* Start of Test Section ***************************************************
   ***************************************************************************************************************/

#include <check.h> /* Includes Check framework */

/*
 * In order to run unit tests with Check, we must create some test cases,
 * aggregate them into a suite, and run them with a suite runner.

 * The pattern of every unit test is as following

 * START_TEST(name_of_test){
 *
 *     perform tests;
 *	       ...assert results
 * }
 * END_TEST

 * Test Case is a set of at least 1 unit test
 * Test Suite is a collection of Test Cases
 * Check Framework can run multiple Test Suites.
 * More details will be on Twiki
 */

/* ------------- Global Definitoions ------------------------- */

/*
 * Global variables definitions, for setup and teardown function.
 */
static ppr_t* pprptr;
static wl_tx_bw_t bw;
static osl_t* osh;


/* ------------- Startup and Teardown - Fixtures ---------------
 * Setting up objects for each unit test,
 * it may be convenient to add some setup that is constant across all the tests in a test case
 * rather than setting up objects for each unit test.
 * Before each unit test in a test case, the setup() function is run, if defined.
 */
void
setup(void)
{
	// Create ppr pointer
	bw = WL_TX_BW_20;
	pprptr = ppr_create(osh,bw);
}

/*
 * Tear down objects for each unit test,
 * it may be convenient to add teardown that is constant across all the tests in a test case
 * rather than tearing down objects for each unit test.
 * After each unit test in a test case, the setup() function is run, if defined.
 * Note: checked teardown() fixture will not run if the unit test fails.
*/
void
teardown(void)
{
	// Delete ppr pointer
	ppr_delete(osh,pprptr);
}

/*
 * The START_TEST/END_TEST pair are macros that setup basic structures to permit testing.
 */

// Assertion of size routine for user alloc/dealloc - bw20
START_TEST(test_ppr_bw20_size){
	const int ppr_bw20_size = 340;
	bw = WL_TX_BW_20;
	ck_assert_int_eq ((int)ppr_size(bw), ppr_bw20_size);
  }
END_TEST

// Assertion of size routine for user alloc/dealloc - bw40
START_TEST(test_ppr_bw40_size){
	const int ppr_bw40_size = 676;
	bw = WL_TX_BW_40;
	ck_assert_int_eq ((int)ppr_size(bw), ppr_bw40_size);
  }
END_TEST

// Assertion of size routine for user alloc/dealloc - bw80
START_TEST(test_ppr_bw80_size){
	const int ppr_bw80_size = 1012;
	bw = WL_TX_BW_80;
	ck_assert_int_eq ((int)ppr_size(bw), ppr_bw80_size);
  }
END_TEST

/*
 * Suite of test cases which Asserts the size routine for user alloc/dealloc
 * for bw20, bw40 and bw80 sizes.
 */

Suite * ppr_size_routine(void)
{
	// Suit creation
	Suite *s = suite_create("PPR - Size routine for user alloc/dealloc");
	// Test case creation
	TCase *tc_size = tcase_create("Test Case - SIZE");
	// Adding unit tests to test case.
	tcase_add_test(tc_size,test_ppr_bw20_size);
	tcase_add_test(tc_size,test_ppr_bw40_size);
	tcase_add_test(tc_size,test_ppr_bw80_size);
	// Adding 'tc_ser_size' test case to a Suite
	suite_add_tcase (s, tc_size);
    return s;
}
