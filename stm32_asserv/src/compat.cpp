#include "compat.h"
#include "parameters.h"

extern "C" void serial_print_int(int i) {
	// SERIAL_MAIN.print(i);
}

extern "C" void serial_print_float(float f) {
	// SERIAL_MAIN.print(f);
}

extern "C" void serial_print(const char *str) {
	// SERIAL_MAIN.println(str);
}

extern "C" void serial_send(char data) { //Envoi d'un octet en serial, dépend de la plateforme
	// SERIAL_MAIN.write(data);
}

extern "C" char generic_serial_read(){
	// return SERIAL_MAIN.read();
	return 0;
}

