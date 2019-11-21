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
#include "test_ppr_clean.h"

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
setup2(void)
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
teardown2(void)
{
	// Delete ppr pointer
	ppr_delete(osh,pprptr);
}

/*
 * The START_TEST/END_TEST pair are macros that setup basic structures to permit testing.
 */

START_TEST(test_ppr_bw20_delete){
	ppr_t* pprptr2;
	wl_tx_bw_t bw2 = WL_TX_BW_40;
	osl_t* osh2;
	pprptr2 = ppr_create(osh2,bw2);

	int ppr_size20=ppr_size(bw2);

	ppr_delete(osh2,pprptr2);
  }
END_TEST

START_TEST(test_ppr_bw20_clear){
	ppr_t* pprptr2;
	wl_tx_bw_t bw2 = WL_TX_BW_20;
	osl_t* osh2;
	pprptr2 = ppr_create(osh2,bw2);
	int ppr_size20 = 182;

	memset( ((uchar*)pprptr2) + 4, (int8)0x30, ppr_size(bw2)-4);

	int i;
	//ppr_delete(osh2,pprptr2);
	ppr_clear(pprptr2);

	for (i = 0; i < ppr_size20 - 4; i++)
		ck_assert_int_eq ( 0xFF & *(4 + i + ((uchar*)pprptr2)), 0xFF & WL_RATE_DISABLED);
  }
END_TEST

START_TEST(test_ppr_bw40_clear){

	ppr_t* pprptr2;
	wl_tx_bw_t bw2 = WL_TX_BW_40;
	osl_t* osh2;
	pprptr2 = ppr_create(osh2,bw2);
	//int ppr_size40=(uint)ppr_size(bw2);
	int ppr_size40 = 364;

	memset( ((uchar*)pprptr2) + 4, (int8)0x50, ppr_size(bw2)-4);

	int i;
	ppr_clear(pprptr2);

		for (i = 0; i < ppr_size40 - 4; i++)
			ck_assert_int_eq ( 0xFF & *(4 + i + ((uchar*)pprptr2)), 0xFF & WL_RATE_DISABLED);
  }
END_TEST

START_TEST(test_ppr_bw80_clear){
	ppr_t* pprptr2;
	wl_tx_bw_t bw2 = WL_TX_BW_80;
	osl_t* osh2;
	pprptr2 = ppr_create(osh2,bw2);
	int ppr_size80 = 546;
	memset( ((uchar*)pprptr2) + 4, (int8)0x70, ppr_size(bw2)-4);

	int i;
	ppr_clear(pprptr2);

	for (i = 0; i < ppr_size80 - 4; i++)
		ck_assert_int_eq ( 0xFF & *(4 + i + ((uchar*)pprptr2)), 0xFF & WL_RATE_DISABLED);
  }
END_TEST

/*
 * Suite of test cases which Asserts the size routine for user serialization
 * allocations.
 *
 */
Suite * ppr_clean(void)
{
	// Suite definition - aggregates test cases into a suite, and run them with a suite runner.
	Suite *s2 = suite_create("PPR - Size routine for user serialization allocations");
	// Test case definition
	TCase *tc_ser_size = tcase_create("Test Case - SER SIZE");
	// Checked fixture to current test case
	tcase_add_checked_fixture(tc_ser_size, setup2, teardown2);
	// Adding unit tests to test case.
	tcase_add_test(tc_ser_size,test_ppr_bw20_delete);
	tcase_add_test(tc_ser_size,test_ppr_bw20_clear);
	tcase_add_test(tc_ser_size,test_ppr_bw40_clear);
	tcase_add_test(tc_ser_size,test_ppr_bw80_clear);
	// Adding 'tc_ser_size' test case to a suite
	suite_add_tcase(s2, tc_ser_size);
	return s2;
}
