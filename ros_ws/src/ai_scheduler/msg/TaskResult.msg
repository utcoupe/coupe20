# Message module that can be included in services. Serves as a universal 
# way of responding if a service was successfully executed.
uint8 RESULT_SUCCESS = 0 # Execution successful.
uint8 RESULT_PAUSE   = 1 # Can be retried later. E.g. navigator couldn't reach 
                         # a goal because of the enemy blocking the way.
uint8 RESULT_FAIL    = 2 # Execution didn't succeed.
uint8 result

string verbose_reason # if RESULT_FAIL, this string will be logged by ai/scheduler.