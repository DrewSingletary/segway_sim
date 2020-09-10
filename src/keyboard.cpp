#include "segway_sim/keyboard.hpp"

cKeyboard::cKeyboard(const char* dev)
{
	active = false;
	keyboard_fd = 0;
	keyboard_ev = new input_event();
	keyboard_st = new keyboard_state();
	keyboard_fd = open(dev, O_RDONLY);
	if (keyboard_fd > 0) {
		ioctl(keyboard_fd, EVIOCGNAME(256), name);
		active = true;
		pthread_create(&thread, 0, &cKeyboard::loop, this);
	}
}

cKeyboard::~cKeyboard()
{
	if (keyboard_fd > 0) {
		active = false;
		pthread_cancel(thread);
		close(keyboard_fd);
	}
	delete keyboard_st;
	delete keyboard_ev;
	keyboard_fd = 0;
}

void* cKeyboard::loop(void *obj)
{
	while (reinterpret_cast<cKeyboard *>(obj)->active) reinterpret_cast<cKeyboard *>(obj)->readEv();
}

void cKeyboard::readEv()
{
	int bytes = read(keyboard_fd, keyboard_ev, sizeof(*keyboard_ev));
	if (bytes > 0) {
		if (keyboard_ev->type & EV_KEY) {
			keyboard_st->keys[keyboard_ev->code] = keyboard_ev->value;
		}
	}
}

short cKeyboard::getKeyState(short key)
{
	return keyboard_st->keys[key];
}