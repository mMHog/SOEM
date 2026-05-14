//
// Created by ralph on 5/28/25.
//

#ifndef CIA402_H
#define CIA402_H

#include "common.h"

int servo_init(uint16* control_word,uint16* status_word)
{
    *control_word = 128;
    osal_usleep(100000);
    printf("Control word: 128 Status word: %u\n", *status_word);
    *control_word = 0;
    osal_usleep(100000);
    printf("Control word: 0 Status word: %u\n", *status_word);
    *control_word = 6;
    osal_usleep(100000);
    printf("Control word: 6 Status word: %u\n", *status_word);
    *control_word = 7;
    osal_usleep(100000);
    printf("Control word: 7 Status word: %u\n", *status_word);
    *control_word = 15;
    osal_usleep(100000);
    printf("Control word: 15 Status word:  %u\n", *status_word);
    if (*status_word != 5943)
    {
        printf("SERVO ENABLE ERROR\n");
    }
    // if (feedback[i]->status_word != 34615 && feedback[i]->status_word != 49975)
    // {
    //     printf("Fail to enable joint %d\n", i + 1);
    //     //exit(0);
    // }

    osal_usleep(100000);
    return 0;
}

int servo_pause(uint16* control_word,uint16* status_word)
{
    *control_word = 7;
    printf("Control word: 7 Status word: %u\n", *status_word);
    return 0;
}

int servo_continue(uint16* control_word,uint16* status_word)
{
    *control_word = 15;
    printf("Control word: 15 Status word: %u\n", *status_word);
    return 0;
}

int servo_stop(uint16* control_word,uint16* status_word)
{
    *control_word = 128;
    printf("Control word: 128 Status word: %u\n", *status_word);
    return 0;
}
#endif //CIA402_H
