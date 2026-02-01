/*
 * keypad.h - 4x4 Matrix Keypad Driver
 */

#ifndef KEYPAD_H
#define KEYPAD_H

/**
 * Initialize keypad GPIO pins
 * Returns 0 on success, negative on error
 */
int keypad_init(void);

/**
 * Scan keypad for key press
 * Returns character of pressed key, or 0 if no key pressed
 * Handles debouncing automatically
 */
char keypad_scan(void);

#endif /* KEYPAD_H */