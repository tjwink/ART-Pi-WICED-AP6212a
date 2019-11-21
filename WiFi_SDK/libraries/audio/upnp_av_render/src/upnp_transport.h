/* upnp_transport.h - UPnP AVTransport definitions
 *
 * Copyright (C) 2005   Ivo Clarysse
 *
 * This file is part of GMediaRender.
 *
 * GMediaRender is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * GMediaRender is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Library General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GMediaRender; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, 
 * MA 02110-1301, USA.
 *
 */

#ifndef _UPNP_TRANSPORT_H
#define _UPNP_TRANSPORT_H

#include "variable-container.h"

typedef enum {
	TRANSPORT_STOPPED,
	TRANSPORT_PLAYING,
	TRANSPORT_TRANSITIONING,	/* optional */
	TRANSPORT_PAUSED_PLAYBACK,	/* optional */
	TRANSPORT_PAUSED_RECORDING,	/* optional */
	TRANSPORT_RECORDING,	/* optional */
	TRANSPORT_NO_MEDIA_PRESENT	/* optional */
}transport_state_t;

struct action_event;
struct service;
struct upnp_device;

struct service *upnp_transport_get_service(void);
void upnp_transport_init(struct upnp_device *);
void upnp_transport_deinit(void);

int stop(struct action_event *event);
int play(struct action_event *event);
int pause_stream(struct action_event *event);
transport_state_t get_transport_state(void);

// Register a callback to get informed when variables change. This should
// return quickly.
void upnp_transport_register_variable_listener(variable_change_listener_t cb,
						       void *userdata);

#endif /* _UPNP_TRANSPORT_H */
