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

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                     Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define FS_COMMANDS \
    { (char*) "mount",          mount,                  1, NULL, NULL, (char*) "<device name>"}, \
    { (char*) "unmount",        unmount,                1, NULL, NULL, (char*) "<device id, 0:Ramdisk, 1:Usbdisk>"}, \
    { (char*) "pwd",            get_cwd,                0, NULL, NULL, (char*) ""}, \
    { (char*) "mkdir",          mk_dir,                 1, NULL, NULL, (char*) "<directory name>"}, \
    { (char*) "rmdir",          rm_dir,                 1, NULL, NULL, (char*) "<directory name>"}, \
    { (char*) "cd",             change_dir,             1, NULL, NULL, (char*) "<directory name>"}, \
    { (char*) "cat",            cat_file,               1, NULL, NULL, (char*) "<file name>"}, \
    { (char*) "cp",             cp_file,                2, NULL, NULL, (char*) "<source file name> <target file name>"}, \
    { (char*) "rm",             rm_file,                1, NULL, NULL, (char*) "<file name>"}, \
    { (char*) "ls",             ls_dir,                 0, NULL, NULL, (char*) ""}, \
    { (char*) "mkfile",         mk_file,                2, NULL, NULL, (char*) "<file name> <file size>"}, \


/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
/* Console commands */
int mount (int argc, char* argv[]);
int unmount (int argc, char* argv[]);
int get_cwd (int argc, char* argv[]);
int mk_dir (int argc, char* argv[]);
int rm_dir (int argc, char* argv[]);
int change_dir (int argc, char* argv[]);
int cat_file (int argc, char* argv[]);
int cp_file (int argc, char* argv[]);
int rm_file (int argc, char* argv[]);
int ls_dir (int argc, char* argv[]);
int mk_file (int argc, char* argv[]);

/* This function read a given file and create/write to another new
 * file, then do file compare.
 */
int file_rw_sha1sum_test (int argc, char* argv[]);

/* This function read and write to the given file with several different
 * burst lengths to do throughput test.
 */
int file_rw_tput_test (int argc, char* argv[]);

/* This function read all of the files under /read directory,
 * and write to /write directory. Also calculate read/write file sha1sum
 * and compare with /sha1_expect_list.txt for validation.
 */
int files_rw_sha1sum_test (int argc, char* argv[]);

int sw_calc_sha1 (int argc, char* argv[]);


#ifdef __cplusplus
} /*extern "C" */
#endif
