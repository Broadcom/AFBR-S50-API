/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 SDK example application.
 *
 * @copyright
 *
 * Copyright (c) 2023, Broadcom Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "examples/examples.h"
#include "platform/argus_print.h" // declaration of print()
#include "board/board.h"

/*!***************************************************************************
 * @brief   Application entry point.
 *
 * @details The main function of the program, called after startup code
 *          This function should never be exited.
 *
 * @return  Never returns anything...
 *****************************************************************************/
int main(void)
{
    /* Initialize the platform hardware including the required peripherals
     * for the API. */
    status_t status = Board_Init();
    HandleError(status, true, "Board initialization failed!");

    /* Pass control to the example code. */
    ExampleMain();

    /* The examples should never return... */
    for (;;);
}

void HandleError(status_t status, bool stop, char const * msg)
{
    /* Check for status < 0 and print message and halt the program execution. */
    if (status < STATUS_OK)
    {
        print("ERROR (%d): %s\n", status, msg);
        if (stop)
        {
            print(" --> Stopped execution due to a critical issue!\n"
                  "     Check the hardware end reset the board!\n");
            while (1) __asm("nop"); // stop!
        }
    }
}

