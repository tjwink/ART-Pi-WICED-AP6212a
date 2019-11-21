#ifndef ITHREAD_H
#define ITHREAD_H

/*******************************************************************************
 *
 * Copyright (c) 2000-2003 Intel Corporation 
 * All rights reserved. 
 * Copyright (c) 2012 France Telecom All rights reserved. 
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 *
 * * Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer. 
 * * Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution. 
 * * Neither name of Intel Corporation nor the names of its contributors 
 * may be used to endorse or promote products derived from this software 
 * without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL INTEL OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY 
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

/*!
 * \file
 */

#include <sys/param.h>
#include "UpnpGlobal.h" /* For UPNP_INLINE, EXPORT_SPEC */
#include "UpnpUniStd.h" /* for close() */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include "wiced_defaults.h"
#include "wiced_utilities.h"
#include "wiced_rtos.h"
#include "wiced_crypto.h"
#include "wiced_time.h"
#include "linked_list.h"

#ifdef DEBUG_THREADUTIL
#define ITHREAD_PRINT(x...)       do { fprintf(stdout, x); } while(0)
#define ITHREAD_PRINT_ERROR(x...) do { fprintf(stderr, x); } while(0)
#else /* !DEBUG_THREADUTIL */
#define ITHREAD_PRINT(x...)       do { } while(0)
#define ITHREAD_PRINT_ERROR(x...) do { } while(0)
#endif /* DEBUG_THREADUTIL */

#define EINVAL    WICED_BADARG
#define EAGAIN    WICED_WOULD_BLOCK
#define ETIMEDOUT WICED_TIMEOUT

#ifdef DEBUG
#define ITHREAD_STACK_MIN (6000)
#else /* !DEBUG */
#define ITHREAD_STACK_MIN (4096)
#endif /* DEBUG */

#define ITHREAD_SIGNAL_EVENT (0x1)

linked_list_t *get_wiced_thread_list(void);
wiced_bool_t   linked_list_compare_cbf(linked_list_node_t* node, void* user_data);

/***************************************************************************
 * Name: ithread_t
 *
 *  Description:
 *      Thread handle.
 *      typedef to pthread_t.
 *      Internal Use Only.
 ***************************************************************************/
typedef wiced_thread_t* ithread_t;


/****************************************************************************
 * Name: start_routine
 *
 *  Description:
 *      Thread start routine 
 *      Internal Use Only.
 ***************************************************************************/
typedef void *(*start_routine)(void *arg);

  
/****************************************************************************
 * Name: ithread_cond_t
 *
 *  Description:
 *      condition variable.
 *      typedef to pthread_cond_t
 *      Internal Use Only.
 ***************************************************************************/
typedef wiced_event_flags_t ithread_cond_t;


/****************************************************************************
 * Name: ithread_mutex_t
 *
 *  Description:
 *      Mutex.
 *      typedef to pthread_mutex_t
 *      Internal Use Only.
 ***************************************************************************/
typedef wiced_mutex_t ithread_mutex_t;


/****************************************************************************
 * Name: ithread_rwlock_t
 *
 *  Description:
 *      Condition attribute.
 *      typedef to pthread_rwlock_t
 *      Internal Use Only
 ***************************************************************************/
/* Read-write locks aren't available: use mutex instead. */
typedef ithread_mutex_t ithread_rwlock_t;


/****************************************************************************
 * Function: ithread_initialize_library
 *
 *  Description:
 *      Initializes the library. Does nothing in all implementations, except
 *      when statically linked for WIN32.
 *  Parameters:
 *      none.
 *  Returns:
 *      0 on success, Nonzero on failure.
 ***************************************************************************/
static UPNP_INLINE int ithread_initialize_library(void) {
	int ret = 0;

	linked_list_init(get_wiced_thread_list());

	return ret;
}


/****************************************************************************
 * Function: ithread_cleanup_library
 *
 *  Description:
 *      Clean up library resources. Does nothing in all implementations, except
 *      when statically linked for WIN32.
 *  Parameters:
 *      none.
 *  Returns:
 *      0 on success, Nonzero on failure.
 ***************************************************************************/
static UPNP_INLINE int ithread_cleanup_library(void) {
	int ret = 0;

	linked_list_node_t *node = NULL;

	while ( linked_list_remove_node_from_rear(get_wiced_thread_list(), &node) == WICED_SUCCESS )
	{
	    wiced_rtos_thread_force_awake((wiced_thread_t *)node->data);
	    wiced_rtos_thread_join((wiced_thread_t *)node->data);
	    wiced_rtos_delete_thread((wiced_thread_t *)node->data);
	    free(node->data);
	    free(node);
	}
	linked_list_deinit(get_wiced_thread_list());

	return ret;
}


/****************************************************************************
 * Function: ithread_initialize_thread
 *
 *  Description:
 *      Initializes the thread. Does nothing in all implementations, except
 *      when statically linked for WIN32.
 *  Parameters:
 *      none.
 *  Returns:
 *      0 on success, Nonzero on failure.
 ***************************************************************************/
static UPNP_INLINE int ithread_initialize_thread(void) {
	int ret = 0;

	return ret;
}


/****************************************************************************
 * Function: ithread_cleanup_thread
 *
 *  Description:
 *      Clean up thread resources. Does nothing in all implementations, except
 *      when statically linked for WIN32.
 *  Parameters:
 *      none.
 *  Returns:
 *      0 on success, Nonzero on failure.
 ***************************************************************************/
static UPNP_INLINE int ithread_cleanup_thread(void) {
	int ret = 0;

	return ret;
}


/****************************************************************************
 * Function: ithread_mutex_init
 *
 *  Description:
 *      Initializes mutex.
 *      Must be called before use.
 *      
 *  Parameters:
 *      ithread_mutex_t * mutex (must be valid non NULL pointer to pthread_mutex_t)
 *      const ithread_mutexattr_t * mutex_attr 
 *  Returns:
 *      0 on success, Nonzero on failure.
 *      Always returns 0.
 *      See man page for pthread_mutex_init
 *****************************************************************************/
#define ithread_mutex_init(mutex, attr) wiced_rtos_init_mutex(mutex)


/****************************************************************************
 * Function: ithread_mutex_lock
 *
 *  Description:
 *      Locks mutex.
 *  Parameters:
 *      ithread_mutex_t * mutex (must be valid non NULL pointer to pthread_mutex_t)
 *      mutex must be initialized.
 *      
 *  Returns:
 *      0 on success, Nonzero on failure.
 *      Always returns 0.
 *      See man page for pthread_mutex_lock
 *****************************************************************************/
#define ithread_mutex_lock wiced_rtos_lock_mutex
  

/****************************************************************************
 * Function: ithread_mutex_unlock
 *
 *  Description:
 *      Unlocks mutex.
 *
 *  Parameters:
 *      ithread_mutex_t * mutex (must be valid non NULL pointer to pthread_mutex_t)
 *      mutex must be initialized.
 *      
 *  Returns:
 *      0 on success, Nonzero on failure.
 *      Always returns 0.
 *      See man page for pthread_mutex_unlock
 *****************************************************************************/
#define ithread_mutex_unlock wiced_rtos_unlock_mutex


/****************************************************************************
 * Function: ithread_mutex_destroy
 *
 *  Description:
 *      Releases any resources held by the mutex. 
 *		Mutex can no longer be used after this call.
 *		Mutex is only destroyed when there are no longer any threads waiting on it. 
 *		Mutex cannot be destroyed if it is locked.
 *  Parameters:
 *      ithread_mutex_t * mutex (must be valid non NULL pointer to pthread_mutex_t)
 *      mutex must be initialized.
 *  Returns:
 *      0 on success. Nonzero on failure.
 *      Always returns 0.
 *      See man page for pthread_mutex_destroy
 *****************************************************************************/
#define ithread_mutex_destroy wiced_rtos_deinit_mutex


/****************************************************************************
 * Function: ithread_rwlock_init
 *
 *  Description:
 *      Initializes rwlock.
 *      Must be called before use.
 *      
 *  Parameters:
 *      ithread_rwlock_t *rwlock (must be valid non NULL pointer to pthread_rwlock_t)
 *      const ithread_rwlockattr_t *rwlock_attr 
 *  Returns:
 *      0 on success, Nonzero on failure.
 *      Always returns 0.
 *      See man page for pthread_rwlock_init
 *****************************************************************************/
/* Read-write locks aren't available: use mutex instead. */
#define ithread_rwlock_init ithread_mutex_init


/****************************************************************************
 * Function: ithread_rwlock_rdlock
 *
 *  Description:
 *      Locks rwlock for reading.
 *  Parameters:
 *      ithread_rwlock_t *rwlock (must be valid non NULL pointer to pthread_rwlock_t)
 *      rwlock must be initialized.
 *      
 *  Returns:
 *      0 on success, Nonzero on failure.
 *      Always returns 0.
 *      See man page for pthread_rwlock_rdlock
 *****************************************************************************/
/* Read-write locks aren't available: use mutex instead. */
#define ithread_rwlock_rdlock ithread_mutex_lock


/****************************************************************************
 * Function: ithread_rwlock_wrlock
 *
 *  Description:
 *      Locks rwlock for writting.
 *  Parameters:
 *      ithread_rwlock_t *rwlock (must be valid non NULL pointer to pthread_rwlock_t)
 *      rwlock must be initialized.
 *      
 *  Returns:
 *      0 on success, Nonzero on failure.
 *      Always returns 0.
 *      See man page for pthread_rwlock_wrlock
 *****************************************************************************/
/* Read-write locks aren't available: use mutex instead. */
#define ithread_rwlock_wrlock ithread_mutex_lock


/****************************************************************************
 * Function: ithread_rwlock_unlock
 *
 *  Description:
 *      Unlocks rwlock.
 *
 *  Parameters:
 *      ithread_rwlock_t *rwlock (must be valid non NULL pointer to pthread_rwlock_t)
 *      rwlock must be initialized.
 *      
 *  Returns:
 *      0 on success, Nonzero on failure.
 *      Always returns 0.
 *      See man page for pthread_rwlock_unlock
 *****************************************************************************/
/* Read-write locks aren't available: use mutex instead. */
#define ithread_rwlock_unlock ithread_mutex_unlock


/****************************************************************************
 * Function: ithread_rwlock_destroy
 *
 *  Description:
 *      Releases any resources held by the rwlock. 
 *		rwlock can no longer be used after this call.
 *		rwlock is only destroyed when there are no longer any threads waiting on it. 
 *		rwlock cannot be destroyed if it is locked.
 *  Parameters:
 *      ithread_rwlock_t *rwlock (must be valid non NULL pointer to pthread_rwlock_t)
 *      rwlock must be initialized.
 *  Returns:
 *      0 on success. Nonzero on failure.
 *      Always returns 0.
 *      See man page for pthread_rwlock_destroy
 *****************************************************************************/
/* Read-write locks aren't available: use mutex instead. */
#define ithread_rwlock_destroy ithread_mutex_destroy


/****************************************************************************
 * Function: ithread_cond_init
 *
 *  Description:
 *      Initializes condition variable.
 *      Must be called before use.
 *  Parameters:
 *      ithread_cond_t *cond (must be valid non NULL pointer to pthread_cond_t)
 *      const ithread_condattr_t *cond_attr (ignored)
 *  Returns:
 *      0 on success, Nonzero on failure.
 *      See man page for pthread_cond_init
 *****************************************************************************/
#define ithread_cond_init(cond, attr) wiced_rtos_init_event_flags(cond)


/****************************************************************************
 * Function: ithread_cond_signal
 *
 *  Description:
 *      Wakes up exactly one thread waiting on condition.
 *      Associated mutex MUST be locked by thread before entering this call.
 *  Parameters:
 *      ithread_cond_t *cond (must be valid non NULL pointer to 
 *      ithread_cond_t)
 *      cond must be initialized
 *  Returns:
 *      0 on success, Nonzero on failure.
 *      See man page for pthread_cond_signal
 *****************************************************************************/
static UPNP_INLINE int ithread_cond_signal(ithread_cond_t *cond)
{
    int rc = 0;
    wiced_result_t result;

    result = wiced_rtos_set_event_flags(cond, ITHREAD_SIGNAL_EVENT);
    if ( result != WICED_SUCCESS )
    {
        rc = EINVAL;
    }

    return rc;
}


/****************************************************************************
 * Function: ithread_cond_broadcast
 *
 *  Description:
 *      Wakes up all threads waiting on condition.
 *      Associated mutex MUST be locked by thread before entering this call.
 *  Parameters:
 *      ithread_cond_t *cond (must be valid non NULL pointer to 
 *      ithread_cond_t)
 *      cond must be initialized
 *  Returns:
 *      0 on success, Nonzero on failure.
 *      See man page for pthread_cond_broadcast
 *****************************************************************************/
static UPNP_INLINE int ithread_cond_broadcast(ithread_cond_t *cond)
{
    return ithread_cond_signal(cond);
}


/****************************************************************************
 * Function: ithread_cond_wait
 *
 *  Description:
 *      Atomically releases mutex and waits on condition.
 *      Associated mutex MUST be locked by thread before entering this call.
 *      Mutex is reacquired when call returns.
 *  Parameters:
 *      ithread_cond_t *cond (must be valid non NULL pointer to 
 *      ithread_cond_t)
 *      cond must be initialized
 *      ithread_mutex_t *mutex (must be valid non NULL pointer to 
 *      ithread_mutex_t)
 *      Mutex must be locked.
 *  Returns:
 *      0 on success, Nonzero on failure.
 *      See man page for pthread_cond_wait
 *****************************************************************************/
static UPNP_INLINE int ithread_cond_wait(ithread_cond_t *cond, ithread_mutex_t *mutex)
{
    int rc = 0;
    wiced_result_t result;
    uint32_t events;

    UNUSED_PARAMETER(events);

    wiced_rtos_wait_for_event_flags(cond, ITHREAD_SIGNAL_EVENT, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_NO_WAIT);
    result = wiced_rtos_unlock_mutex(mutex);
    wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, rc = EINVAL );
    result = wiced_rtos_wait_for_event_flags(cond, ITHREAD_SIGNAL_EVENT, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);
    if ( result != WICED_SUCCESS )
    {
        rc = EINVAL;
    }
    result = wiced_rtos_lock_mutex(mutex);
    wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, rc = EINVAL );

_exit:
    return rc;
}


  /****************************************************************************
   * Function: pthread_cond_timedwait
   *
   *	Description:      
   *		Atomically releases the associated mutex and waits on the
   *	condition.
   *		If the condition is not signaled in the specified time than the
   *	call times out and returns.
   *		Associated mutex MUST be locked by thread before entering this call.
   *		Mutex is reacquired when call returns.
   *  Parameters:
   *      ithread_cond_t *cond (must be valid non NULL pointer to ithread_cond_t)
   *      	cond must be initialized
   *      ithread_mutex_t *mutex (must be valid non NULL pointer to ithread_mutex_t)
   *      	Mutex must be locked.
   *      const struct timespec *abstime (absolute time, measured from Jan 1, 1970)
   *  Returns:
   *      0 on success. ETIMEDOUT on timeout. Nonzero on failure.
   *      See man page for pthread_cond_timedwait
   ***************************************************************************/
 
static UPNP_INLINE int ithread_cond_timedwait(ithread_cond_t *cond, ithread_mutex_t *mutex, const struct timespec *abstime, time_t reltime)
{
    int            rc             = 0;
    wiced_result_t result;
    wiced_time_t   now_time_msec  = 0;
    wiced_time_t   then_time_msec;
    uint32_t       events;

    UNUSED_PARAMETER(events);

    if ( abstime != NULL )
    {
        result = wiced_time_get_time(&now_time_msec);
        wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, rc = EINVAL );
        then_time_msec = (abstime->tv_sec * 1000) + (abstime->tv_nsec / 1000000UL);
        if ( then_time_msec > now_time_msec )
        {
            then_time_msec -= now_time_msec;
        }
        else
        {
            then_time_msec = 0;
        }
    }
    else
    {
        then_time_msec = (wiced_time_t)reltime;
    }

    wiced_rtos_wait_for_event_flags(cond, ITHREAD_SIGNAL_EVENT, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_NO_WAIT);
    result = wiced_rtos_unlock_mutex(mutex);
    wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, rc = EINVAL );
    ITHREAD_PRINT("%s(%p)[%d]: about to wait %d msecs...\n", __FUNCTION__, cond, (int)now_time_msec, (int)then_time_msec);
    result = wiced_rtos_wait_for_event_flags(cond, ITHREAD_SIGNAL_EVENT, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, then_time_msec);
    if ( result != WICED_SUCCESS )
    {
        if ( result == WICED_TIMEOUT )
        {
            rc = ETIMEDOUT;
        }
        else
        {
            rc = EINVAL;
        }
        wiced_time_get_time(&now_time_msec);
        ITHREAD_PRINT("%s(%p)[%d]: failed with %s(%d)\n", __FUNCTION__, cond, (int)now_time_msec, ((result == WICED_TIMEOUT) ? "TIMEOUT" : "ERROR"), (int)result);
    }
    result = wiced_rtos_lock_mutex(mutex);
    wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, rc = EINVAL );

_exit:
    return rc;
}
  

  /****************************************************************************
   * Function: ithread_cond_destroy
   *
   *  Description:
   *      Releases any resources held by the condition variable. 
   *		Condition variable can no longer be used after this call.	
   *  Parameters:
   *      ithread_cond_t *cond (must be valid non NULL pointer to 
   *      ithread_cond_t)
   *      cond must be initialized.
   *  Returns:
   *      0 on success. Nonzero on failure.
   *      See man page for pthread_cond_destroy
   ***************************************************************************/
#define ithread_cond_destroy wiced_rtos_deinit_event_flags


  /****************************************************************************
   * Function: ithread_create
   *
   *  Description:
   *		Creates a thread with the given start routine
   *      and argument.
   *  Parameters:
   *      ithread_t * thread (must be valid non NULL pointer to pthread_t)
   *      ithread_attr_t *attr
   *      void * (start_routine) (void *arg) (start routine)
   *      void * arg - argument.
   *  Returns:
   *      0 on success. Nonzero on failure.
   *	    Returns EAGAIN if a new thread can not be created.
   *      Returns EINVAL if there is a problem with the arguments.
   *      See man page fore pthread_create
   ***************************************************************************/
static UPNP_INLINE int ithread_create(ithread_t *thread, uint32_t stack_size, start_routine func, void *arg)
{
    wiced_result_t result;
    int            rc        = EAGAIN;
    linked_list_node_t *node = NULL;

    *thread = NULL;
    *thread = malloc(sizeof(wiced_thread_t));
    if ( *thread == NULL )
    {
        ITHREAD_PRINT_ERROR("malloc(sizeof(wiced_thread_t)) failed !\n");
        goto _exit;
    }

    node = malloc(sizeof(linked_list_node_t));
    if ( node == NULL )
    {
        ITHREAD_PRINT_ERROR("malloc(sizeof(linked_list_node_t)) failed !\n");
        goto _exit;
    }

    if ( stack_size < ITHREAD_STACK_MIN )
    {
        stack_size = ITHREAD_STACK_MIN;
    }
    result = wiced_rtos_create_thread( *thread, WICED_DEFAULT_LIBRARY_PRIORITY, "ithread_upnp", (wiced_thread_function_t)func, stack_size, arg );
    if (result == WICED_SUCCESS)
    {
        rc = 0;
    }
    else
    {
        ITHREAD_PRINT_ERROR("wiced_rtos_create_thread(thread=%p,func=%p,stack_size=%d) failed with %d\n", *thread, func, (int)stack_size, (int)result);
    }

 _exit:
    if ( rc == 0 )
    {
        node->data = *thread;
        linked_list_insert_node_at_rear(get_wiced_thread_list(), node);
    }
    else
    {
        if ( *thread != NULL )
        {
            free(*thread);
            *thread = NULL;
        }
        if ( node != NULL )
        {
            free(node);
        }
    }
    return rc;
}


  /****************************************************************************
   * Function: ithread_cancel
   *
   *  Description:
   *		Cancels a thread.
   *  Parameters:
   *      ithread_t * thread (must be valid non NULL pointer to ithread_t)
   *  Returns:
   *      0 on success. Nonzero on failure.
   *      See man page for pthread_cancel
   ***************************************************************************/
static UPNP_INLINE int ithread_cancel(ithread_t *thread)
{
    wwd_result_t result;
    int            rc   = EAGAIN;

    result  = wiced_rtos_thread_force_awake(*thread);
    result |= host_rtos_finish_thread((host_thread_type_t *)(*thread));
    if (result == WWD_SUCCESS)
    {
        rc = 0;
    }
    else
    {
        ITHREAD_PRINT_ERROR("host_rtos_finish_thread(thread=%p) failed with %d\n", *thread, (int)result);
    }

    return rc;
}


  /****************************************************************************
   * Function: ithread_join
   *
   *  Description:
   *		Suspends the currently running thread until the 
   * specified thread
   *      has finished. 
   *      Returns the return code of the thread, or ITHREAD_CANCELED 
   *      if the thread has been canceled.
   *  Parameters:
   *      ithread_t *thread (valid non null thread identifier)
   *      void ** return (space for return code) 
   *  Returns:
   *		0 on success, Nonzero on failure.
   *     See man page for pthread_join
   ***************************************************************************/
static UPNP_INLINE int ithread_join(ithread_t *thread, void **rc_code)
{
    wiced_result_t result;
    int            rc     = EAGAIN;

    result = wiced_rtos_thread_join(*thread);
    if (result == WICED_SUCCESS)
    {
        linked_list_node_t *node = NULL;

        rc = 0;
        if ( linked_list_find_node(get_wiced_thread_list(), linked_list_compare_cbf, *thread, &node) == WICED_SUCCESS )
        {
            wiced_rtos_delete_thread(*thread);
            linked_list_remove_node(get_wiced_thread_list(), node);
            free(*thread);
            *thread = NULL;
            free(node);
        }
    }
    else
    {
        ITHREAD_PRINT_ERROR("wiced_rtos_thread_join(thread=%p) failed with %d\n", *thread, (int)result);
    }

    return rc;
}
  

/****************************************************************************
 * Function: isleep
 *
 *  Description:
 *		Suspends the currently running thread for the specified number 
 *      of seconds
 *      Always returns 0.
 *  Parameters:
 *      unsigned int seconds - number of seconds to sleep.
 *  Returns:
 *		0 on success, Nonzero on failure.
 *              See man page for sleep (man 3 sleep)
 *****************************************************************************/
#define isleep(x) wiced_rtos_delay_milliseconds((x)*1000)


/****************************************************************************
 * Function: isleep
 *
 *  Description:
 *		Suspends the currently running thread for the specified number 
 *      of milliseconds
 *      Always returns 0.
 *  Parameters:
 *      unsigned int milliseconds - number of milliseconds to sleep.
 *  Returns:
 *		0 on success, Nonzero on failure.
 *              See man page for sleep (man 3 sleep)
 *****************************************************************************/
#define imillisleep(x) wiced_rtos_delay_milliseconds(x)


#ifdef __cplusplus
}
#endif


#endif /* ITHREAD_H */

