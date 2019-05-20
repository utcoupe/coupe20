#ifndef POLOLU_A4983_H
#define POLOLU_A4983_H

#define ENABLE LOW
#define DISABLE HIGH
//#define MIN_DELAY 4000//5000//2000 //us //350
#define STEP_PER_REVOLUTION 200

/** Includes **/
/**************/

#include "Arduino.h"

/** Defines **/
/*************/


/** Class Descritpion **/

class PololuA4983
{
	private:
		int16_t m_dir_pin;
		int16_t m_step_pin;
		int16_t m_en_pin;
		uint32_t m_last_step_time;
		int16_t m_remaining_steps;
		uint32_t elapsedTime();
		uint16_t m_min_delay;


	public:
		PololuA4983(int step_pin, int dir_pin, int en_pin, uint16_t min_delay);
		PololuA4983(int step_pin, int dir_pin, uint16_t min_delay);
		~PololuA4983();
		void moveStep(uint16_t nb_steps, bool dir);
		void moveRevolution(uint16_t nb_rev, bool dir);
		
		void enable();
		void disable();
		void update();
		void stop();
		int16_t getRemainingStep();
		int16_t m_position_step;

		

};


#endif //POLOLU_A4983_H