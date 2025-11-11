#ifndef COUNTSTEP_H_
#define COUNTSTEP_H_

#if COMPILE_COUNTSTEP

#define COUNTSTEP_NAME STR_COUNTSTEP

void StepTask(void *pvParameters);
void KeyTask(void *pvParameters);
void DisplayTask(void *pvParameters);

#endif

#endif /* COUNTSTEP_H_ */