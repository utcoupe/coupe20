class Status():
    STATUS_INIT   = 0 # All nodes initializing, didn't respond yet.
    STATUS_INGAME = 1 # Scheduler started, doing its job.
    STATUS_HALT   = 2 # Robot stopped (game end, critical HALT requested by a node...)

    INIT_INITIALIZING = 0 # All nodes didn't respond and we didn't reach the init timeout yet.
    INIT_INITIALIZED  = 1 # All nodes responded successfully and are initialized.
    INIT_FAILED       = 2 # Nodes responded false or didn't respond after before the init timeout.

    INIT_CHECKLIST = {  # Please comment the lines instead of deleting them.
        "/ai/scheduler": None,

        "/memory/map": None,
        "/memory/definitions": None,

        "/navigation/navigator": None,
        "/navigation/pathfinder": None,
        "/navigation/collisions": None,

        "/movement/actuators": None,

        "/recognition/localizer": None,
        "/recognition/enemy_tracker": None,
        # "/recognition/cube_finder": None,
        # "/recognition/cp_recognizer": None,
        "/recognition/objects_classifier": None,

        "/processing/belt_interpreter": None,
        # "/processing/lidar_objects": None,

        "/drivers/ard_asserv": None,
        #"/drivers/ard_hmi": None,
        #"/drivers/ard_others": None,
        "/drivers/port_finder": None,
        "/drivers/ax12": None,
        "/drivers/teraranger": None,
    }
