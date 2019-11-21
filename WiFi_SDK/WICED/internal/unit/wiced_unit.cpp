
/*
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
 */

/** @file
 *  Unit Tester for Wiced layer
 *
 */

#include "gtest/gtest.h"
#include "wiced.h"
#include "mock.h"

#define RETCHK_START( retcheck_var_in )  \
        { \
            int count = 0; \
            int* retcheck_var = &(retcheck_var_in); \
            do \
            { \
                *retcheck_var = count; \
                count++;

#define RETCHK_ACTIVATED()  (*retcheck_var == -1)

#define RETCHK_EXPECT_EQ( expected, val )  if ( RETCHK_ACTIVATED( ) ) { EXPECT_EQ( (expected), (val) ); }
#define RETCHK_EXPECT_NE( expected, val )  if ( RETCHK_ACTIVATED( ) ) { EXPECT_NE( (expected), (val) ); }
#define RETCHK_EXPECT_GT( expected, val )  if ( RETCHK_ACTIVATED( ) ) { EXPECT_GT( (expected), (val) ); }

#define RETCHK_COUNT( ) count

#define RETCHK_END( ) \
            } while ( *retcheck_var == -1 ); \
            *retcheck_var = -1; \
        }


static int bad_malloc_countdown = -1;
static const wiced_ip_setting_t ip_settings  = { .ip_address = { WICED_IPV4, { .v4 = MAKE_IPV4_ADDRESS(192, 168, 1, 105) } },
                                                 .gateway    = { WICED_IPV4, { .v4 = MAKE_IPV4_ADDRESS(192, 168, 1,   1) } },
                                                 .netmask    = { WICED_IPV4, { .v4 = MAKE_IPV4_ADDRESS(192, 168, 1, 255) } },
};

static void* wiced_unit_malloc( size_t size );
static void * wiced_unit_malloc_named( const char* name, size_t size );
extern "C" void * __real_malloc_named( const char* name, size_t size );
extern "C" void * malloc_debug_malloc( size_t size );

class unit_test_wiced_init :  public ::testing::Test
{

    protected:

    virtual void SetUp()
    {
        malloc_leak_set_ignored( LEAK_CHECK_GLOBAL );
    }
    virtual void TearDown()
    {
        malloc_leak_check( NULL, LEAK_CHECK_GLOBAL );
    }
};

/* Initialise WicedFS normally */
TEST_F(unit_test_wiced_init, normal)
{
    EXPECT_EQ( WICED_SUCCESS, wiced_init( ) );
    EXPECT_EQ( WICED_SUCCESS, wiced_deinit( ) );
}

TEST_F(unit_test_wiced_init, double_init)
{
    EXPECT_EQ( WICED_SUCCESS, wiced_init( ) );
    EXPECT_EQ( WICED_SUCCESS, wiced_init( ) );
    EXPECT_EQ( WICED_SUCCESS, wiced_deinit( ) );
}

TEST_F(unit_test_wiced_init, double_deinit)
{
    EXPECT_EQ( WICED_SUCCESS, wiced_init( ) );
    EXPECT_EQ( WICED_SUCCESS, wiced_deinit( ) );
    EXPECT_EQ( WICED_SUCCESS, wiced_deinit( ) );
}


TEST_F(unit_test_wiced_init, bad_mallocs_init)
{
    set_mock_function( malloc, wiced_unit_malloc );
    set_mock_function( malloc_named, wiced_unit_malloc_named );

//    malloc(1);

    RETCHK_START( bad_malloc_countdown )
    {
        RETCHK_EXPECT_NE( WICED_SUCCESS, wiced_init( ) );
        malloc_leak_check( NULL, LEAK_CHECK_GLOBAL );
    }
    RETCHK_END( )

    reset_mock_function( malloc );
    reset_mock_function( malloc_named );
}



TEST_F(unit_test_wiced_init, bad_mallocs_up)
{
    EXPECT_EQ( WICED_SUCCESS, wiced_init( ) );
    malloc_leak_set_ignored( LEAK_CHECK_GLOBAL );

    set_mock_function( malloc, wiced_unit_malloc );
    set_mock_function( malloc_named, wiced_unit_malloc_named );

//    malloc(1);

    RETCHK_START( bad_malloc_countdown )
    {
        RETCHK_EXPECT_NE( WICED_SUCCESS, wiced_network_up(WICED_STA_INTERFACE, WICED_USE_STATIC_IP, &ip_settings) );
        malloc_leak_check( NULL, LEAK_CHECK_GLOBAL );
    }
    RETCHK_END( )

    reset_mock_function( malloc );
    reset_mock_function( malloc_named );

    EXPECT_EQ( WICED_SUCCESS, wiced_deinit( ) );

}



TEST_F(unit_test_wiced_init, net_up_normal)
{
    EXPECT_EQ( WICED_SUCCESS, wiced_init( ) );
    EXPECT_EQ( WICED_SUCCESS, wiced_network_up(WICED_STA_INTERFACE, WICED_USE_STATIC_IP, &ip_settings) );
    EXPECT_EQ( WICED_SUCCESS, wiced_network_down( WICED_STA_INTERFACE ) );
    EXPECT_EQ( WICED_SUCCESS, wiced_deinit( ) );
}

TEST_F(unit_test_wiced_init, net_up_twice)
{
    EXPECT_EQ( WICED_SUCCESS, wiced_init( ) );
    EXPECT_EQ( WICED_SUCCESS, wiced_network_up(WICED_STA_INTERFACE, WICED_USE_STATIC_IP, &ip_settings) );
    EXPECT_EQ( WICED_SUCCESS, wiced_network_up(WICED_STA_INTERFACE, WICED_USE_STATIC_IP, &ip_settings) );
    EXPECT_EQ( WICED_SUCCESS, wiced_network_down( WICED_STA_INTERFACE ) );
    EXPECT_EQ( WICED_SUCCESS, wiced_deinit( ) );
}


TEST_F(unit_test_wiced_init, net_down_twice)
{
    EXPECT_EQ( WICED_SUCCESS, wiced_init( ) );
    EXPECT_EQ( WICED_SUCCESS, wiced_network_up(WICED_STA_INTERFACE, WICED_USE_STATIC_IP, &ip_settings) );
    EXPECT_EQ( WICED_SUCCESS, wiced_network_down( WICED_STA_INTERFACE ) );
    EXPECT_EQ( WICED_SUCCESS, wiced_network_down( WICED_STA_INTERFACE ) );
    EXPECT_EQ( WICED_SUCCESS, wiced_deinit( ) );
}






static void * wiced_unit_malloc_named( const char* name, size_t size )
{
    if ( bad_malloc_countdown == 0 )
    {
        bad_malloc_countdown--;
        return NULL;
    }
    if ( bad_malloc_countdown != -1 )
    {
        bad_malloc_countdown--;
    }

    return __real_malloc_named( name, size );
}


static void* wiced_unit_malloc( size_t size )
{
    if ( bad_malloc_countdown == 0 )
    {
        bad_malloc_countdown--;
        return NULL;
    }
    if ( bad_malloc_countdown != -1 )
    {
        bad_malloc_countdown--;
    }

    return malloc_debug_malloc( size );
}




