uint8 EVENT_HMI_INITIALIZED = 0 # Sent to tell ai/game_manager HMI is alive.
uint8 EVENT_START           = 1 # Sent to tell ai/scheduler to start.
uint8 EVENT_JACK_FIRED      = 2 # Sent when the user activates the jack (needs to be armed beforehand).
uint8 EVENT_GAME_CANCEL     = 3 # Sent when the user clicks clicks on 'previous' when in game.
uint8 event

uint8 chosen_strategy_id # Used for event
uint8 chosen_team_id     # start only.