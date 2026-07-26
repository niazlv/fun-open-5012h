/*
 * Application launcher: the root screen of the UI stack.
 * Applications are described by a const table and run as screens on top.
 */

#ifndef _LAUNCHER_H_
#define _LAUNCHER_H_

/*- Prototypes --------------------------------------------------------------*/
void launcher_start(void);

// Exit the running application back to the launcher (callable from apps)
void launcher_exit_app(void);

#endif // _LAUNCHER_H_
