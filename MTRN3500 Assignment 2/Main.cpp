#include "TMM.h"

int main(void) {
	ThreadManagement^ mainThread = gcnew ThreadManagement();
	mainThread->SetupSharedMemory();
	mainThread->threadFunction();
	return 0;
}