#!/usr/bin/env python

from asserv import set_pos_test
from asserv import pwm_test
from asserv import spd_test
from asserv import goto_test

def main():
    set_pos_test.main()
    pwm_test.main()
    spd_test.main()
    goto_test.main()    

if __name__ == "__main__":
    main()
