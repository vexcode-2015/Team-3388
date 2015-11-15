#ifndef Utils.c
#define Utils.c

int threshold(int in, int deadzone){
	if(abs(in) < deadzone){
		return 0;
	}
	return in;
}
#endif
