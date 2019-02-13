uint8 LOAD_WAITING     = 0  # waiting for AX12 to seize the atom (bottom)
uint8 LOAD_FINISH      = 1  # the atom has been brought up (top)
uint8 LOAD_IN_PROGRESS = 2  # the atom is being brought up (middle)
uint8 load_status

# Number of atom loaded   !!! When loading tower impossible to know the number of atom loaded (can only guess)
bool is_active
float32 nb_atom_in 