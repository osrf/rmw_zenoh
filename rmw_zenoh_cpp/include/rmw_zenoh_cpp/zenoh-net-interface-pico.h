/*------------------ Zenoh-pico operations ------------------*/
/**
 * Read from the network. This function should be called manually called when
 * the read loop has not been started, e.g., when running in a single thread.
 *
 * Parameters:
 *     session: The zenoh-net session.
 * Returns:
 *     ``0`` in case of success, ``-1`` in case of failure.
 */
int znp_read(zn_session_t * z);

/**
 * Start a separate task to read from the network and process the messages
 * as soon as they are received. Note that the task can be implemented in
 * form of thread, process, etc. and its implementation is platform-dependent.
 *
 * Parameters:
 *     session: The zenoh-net session.
 * Returns:
 *     ``0`` in case of success, ``-1`` in case of failure.
 */
int znp_start_read_task(zn_session_t * z);

/**
 * Stop the read task. This may result in stopping a thread or a process depending
 * on the target platform.
 *
 * Parameters:
 *     session: The zenoh-net session.
 * Returns:
 *     ``0`` in case of success, ``-1`` in case of failure.
 */
int znp_stop_read_task(zn_session_t * z);

/**
 * Send a KeepAlive message.
 *
 * Parameters:
 *     session: The zenoh-net session.
 * Returns:
 *     ``0`` in case of success, ``-1`` in case of failure.
 */
int znp_send_keep_alive(zn_session_t * z);

/**
 * Start a separate task to handle the session lease. This task will send ``KeepAlive``
 * messages when needed and will close the session when the lease is expired. Note that
 * the task can be implemented in form of thread, process, etc. and its implementation
 * is platform-dependent.
 *
 * Parameters:
 *     session: The zenoh-net session.
 * Returns:
 *     ``0`` in case of success, ``-1`` in case of failure.
 */
int znp_start_lease_task(zn_session_t * z);

/**
 * Stop the lease task. This may result in stopping a thread or a process depending
 * on the target platform.
 *
 * Parameters:
 *     session: The zenoh-net session.
 * Returns:
 *     ``0`` in case of success, ``-1`` in case of failure.
 */
int znp_stop_lease_task(zn_session_t * z);
