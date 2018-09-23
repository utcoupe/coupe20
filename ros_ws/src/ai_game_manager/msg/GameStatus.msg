uint8 STATUS_INIT   = 0 # Initializing, initialized of waiting for scheduler start command.
uint8 STATUS_INGAME = 1 # Scheduler was activated and set the game to start. Independent from timer.
uint8 STATUS_HALT   = 2 # A node asked for a general HALT or gae ended (timer finished).
uint8 game_status

uint8 INIT_INITIALIZING = 0 # All nodes didn't respond and we didn't reach the init timeout yet.
uint8 INIT_INITIALIZED  = 1 # All nodes responded successfully and are initialized.
uint8 INIT_FAILED       = 2 # Nodes responded false or didn't respond after before the init timeout.
uint8 init_status