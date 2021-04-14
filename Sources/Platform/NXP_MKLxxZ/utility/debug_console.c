/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file is a modified version of the file fsl_debug_console.c
 * 				which was distributed by Freescale as part of the Kinetis
 * 				Software Development Kit v2.0.
 * 
 * @copyright
 * 
 * Copyright (c) 2021, Broadcom Inc
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
 *
 ******************************************************************************
 * Original file header:
 *
 *
 * This is a modified version of the file printf.c, which was distributed
 * by Motorola as part of the M5407C3BOOT.zip package used to initialize
 * the M5407C3 evaluation board.
 *
 * Copyright:
 *      1999-2000 MOTOROLA, INC. All Rights Reserved.
 *  You are hereby granted a copyright license to use, modify, and
 *  distribute the SOFTWARE so long as this entire notice is
 *  retained without alteration in any modified and/or redistributed
 *  versions, and that such modified versions are clearly identified
 *  as such. No licenses are granted by implication, estoppel or
 *  otherwise under any patents or trademarks of Motorola, Inc. This
 *  software is provided on an "AS IS" basis and without warranty.
 *
 *  To the maximum extent permitted by applicable law, MOTOROLA
 *  DISCLAIMS ALL WARRANTIES WHETHER EXPRESS OR IMPLIED, INCLUDING
 *  IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR
 *  PURPOSE AND ANY WARRANTY AGAINST INFRINGEMENT WITH REGARD TO THE
 *  SOFTWARE (INCLUDING ANY MODIFIED VERSIONS THEREOF) AND ANY
 *  ACCOMPANYING WRITTEN MATERIALS.
 *
 *  To the maximum extent permitted by applicable law, IN NO EVENT
 *  SHALL MOTOROLA BE LIABLE FOR ANY DAMAGES WHATSOEVER (INCLUDING
 *  WITHOUT LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS
 *  INTERRUPTION, LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY
 *  LOSS) ARISING OF THE USE OR INABILITY TO USE THE SOFTWARE.
 *
 *  Motorola assumes no responsibility for the maintenance and support
 *  of this software

 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "debug_console.h"

#include <stdarg.h>
#include <stdlib.h>
#if defined(__CC_ARM)
#include <stdio.h>
#endif
#include <math.h>

/*! @brief Keil: suppress ellipsis warning in va_arg usage below. */
#if defined(__CC_ARM)
#pragma diag_suppress 1256
#endif /* __CC_ARM */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief This definition is maximum line that debugconsole can scanf each time.*/
#define IO_MAXLINE 20U

/*! @brief The overflow value.*/
#ifndef HUGE_VAL
#define HUGE_VAL (99.e99)
#endif /* HUGE_VAL */


#if PRINTF_ADVANCED_ENABLE
/*! @brief Specification modifier flags for printf. */
enum _debugconsole_printf_flag
{
    kPRINTF_Minus = 0x01U,              /*!< Minus FLag. */
    kPRINTF_Plus = 0x02U,               /*!< Plus Flag. */
    kPRINTF_Space = 0x04U,              /*!< Space Flag. */
    kPRINTF_Zero = 0x08U,               /*!< Zero Flag. */
    kPRINTF_Pound = 0x10U,              /*!< Pound Flag. */
    kPRINTF_LengthChar = 0x20U,         /*!< Length: Char Flag. */
    kPRINTF_LengthShortInt = 0x40U,     /*!< Length: Short Int Flag. */
    kPRINTF_LengthLongInt = 0x80U,      /*!< Length: Long Int Flag. */
    kPRINTF_LengthLongLongInt = 0x100U, /*!< Length: Long Long Int Flag. */
};
#endif /* PRINTF_ADVANCED_ENABLE */

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/


/*!
 * @brief This function puts padding character.
 *
 * @param[in] c         Padding character.
 * @param[in] curlen    Length of current formatted string .
 * @param[in] width     Width of expected formatted string.
 * @param[in] count     Number of characters.
 * @param[in] func_ptr  Function to put character out.
 */
static void PrintfPaddingCharacter(
    char c, int32_t curlen, int32_t width, int32_t *count, PUTCHAR_FUNC func_ptr, void * buf)
{
    int32_t i;

    for (i = curlen; i < width; i++)
    {
        func_ptr(buf, c);
        (*count)++;
    }
}

/*!
 * @brief Converts a radix number to a string and return its length.
 *
 * @param[in] numstr    Converted string of the number.
 * @param[in] nump      Pointer to the number.
 * @param[in] neg       Polarity of the number.
 * @param[in] radix     The radix to be converted to.
 * @param[in] use_caps  Used to identify %x/X output format.

 * @return Length of the converted string.
 */
static int32_t ConvertRadixNumToString(char *numstr, void *nump, int32_t neg, int32_t radix, bool use_caps)
{
#if PRINTF_ADVANCED_ENABLE
    int64_t a;
    int64_t b;
    int64_t c;

    uint64_t ua;
    uint64_t ub;
    uint64_t uc;
#else
    int32_t a;
    int32_t b;
    int32_t c;

    uint32_t ua;
    uint32_t ub;
    uint32_t uc;
#endif /* PRINTF_ADVANCED_ENABLE */

    int32_t nlen;
    char *nstrp;

    nlen = 0;
    nstrp = numstr;
    *nstrp++ = '\0';

    if (neg)
    {
#if PRINTF_ADVANCED_ENABLE
        a = *(int64_t *)nump;
#else
        a = *(int32_t *)nump;
#endif /* PRINTF_ADVANCED_ENABLE */
        if (a == 0)
        {
            *nstrp = '0';
            ++nlen;
            return nlen;
        }
        while (a != 0)
        {
#if PRINTF_ADVANCED_ENABLE
            b = (int64_t)a / (int64_t)radix;
            c = (int64_t)a - ((int64_t)b * (int64_t)radix);
            if (c < 0)
            {
                uc = (uint64_t)c;
                c = (int64_t)(~uc) + 1 + '0';
            }
#else
            b = a / radix;
            c = a - (b * radix);
            if (c < 0)
            {
                uc = (uint32_t)c;
                c = (int32_t)((uint32_t)(~uc) + 1 + '0');
            }
#endif /* PRINTF_ADVANCED_ENABLE */
            else
            {
                c = c + '0';
            }
            a = b;
            *nstrp++ = (char)c;
            ++nlen;
        }
    }
    else
    {
#if PRINTF_ADVANCED_ENABLE
        ua = *(uint64_t *)nump;
#else
        ua = *(uint32_t *)nump;
#endif /* PRINTF_ADVANCED_ENABLE */
        if (ua == 0)
        {
            *nstrp = '0';
            ++nlen;
            return nlen;
        }
        while (ua != 0)
        {
#if PRINTF_ADVANCED_ENABLE
            ub = (uint64_t)ua / (uint64_t)radix;
            uc = (uint64_t)ua - ((uint64_t)ub * (uint64_t)radix);
#else
            ub = ua / (uint32_t)radix;
            uc = ua - (ub * (uint32_t)radix);
#endif /* PRINTF_ADVANCED_ENABLE */

            if (uc < 10)
            {
                uc = uc + '0';
            }
            else
            {
                uc = uc - 10 + (use_caps ? 'A' : 'a');
            }
            ua = ub;
            *nstrp++ = (char)uc;
            ++nlen;
        }
    }
    return nlen;
}



/*!
 * @brief	This function outputs its parameters according to a formatted string.
 *
 * @note	I/O is performed by calling given function pointer using following
 * 			(*func_ptr)(c);
 *
 * @param	func_ptr Function to put character out.
 * @param	buf Buffer to put characters in.
 * @param	fmt Format string for printf.
 * @param	ap Arguments to printf.
 *
 * @return	Number of characters
 */
int PrintfFormattedData(PUTCHAR_FUNC func_ptr, void * buf, const char *fmt, va_list * ap)
{
    /* va_list ap; */
    char *p;
    int32_t c;

    char vstr[33];
    char *vstrp = NULL;
    int32_t vlen = 0;

    int32_t done;
    int32_t count = 0;

    int32_t field_width;
    uint32_t precision_width;
    char *sval;
    int32_t cval;
    bool use_caps;
    uint8_t radix = 0;

#if PRINTF_ADVANCED_ENABLE
    uint32_t flags_used;
    int32_t schar, dschar;
    int64_t ival;
    uint64_t uval = 0;
#else
    int32_t ival;
    uint32_t uval = 0;
#endif /* PRINTF_ADVANCED_ENABLE */


    /* Start parsing apart the format string and display appropriate formats and data. */
    for (p = (char *)fmt; (c = *p) != 0; p++)
    {
        /*
         * All formats begin with a '%' marker.  Special chars like
         * '\n' or '\t' are normally converted to the appropriate
         * character by the __compiler__.  Thus, no need for this
         * routine to account for the '\' character.
         */
        if (c != '%')
        {
            func_ptr(buf, c);
            count++;
            /* By using 'continue', the next iteration of the loop is used, skipping the code that follows. */
            continue;
        }

        use_caps = true;

#if PRINTF_ADVANCED_ENABLE
        /* First check for specification modifier flags. */
        flags_used = 0;
        done = false;
        while (!done)
        {
            switch (*++p)
            {
                case '-':
                    flags_used |= kPRINTF_Minus;
                    break;
                case '+':
                    flags_used |= kPRINTF_Plus;
                    break;
                case ' ':
                    flags_used |= kPRINTF_Space;
                    break;
                case '0':
                    flags_used |= kPRINTF_Zero;
                    break;
                case '#':
                    flags_used |= kPRINTF_Pound;
                    break;
                default:
                    /* We've gone one char too far. */
                    --p;
                    done = true;
                    break;
            }
        }
#endif /* PRINTF_ADVANCED_ENABLE */

        /* Next check for minimum field width. */
        field_width = 0;
        done = false;
        while (!done)
        {
            c = *++p;
            if ((c >= '0') && (c <= '9'))
            {
            	field_width *= 10;
                field_width += (c - '0');
            }
            else
            {
                /* We've gone one char too far. */
                --p;
                done = true;
            }
        }
        /* Next check for the width and precision field separator. */
        precision_width = 6;
        if (*++p == '.')
        {
            /* Must get precision field width, if present. */
            precision_width = 0;
            done = false;
            while (!done)
            {
                c = *++p;
                if ((c >= '0') && (c <= '9'))
                {
                	precision_width *= 10U;
                    precision_width += (uint32_t)(c - '0');
                }
                else
                {
                    /* We've gone one char too far. */
                    --p;
                    done = true;
                }
            }
        }
        else
        {
            /* We've gone one char too far. */
            --p;
        }
#if PRINTF_ADVANCED_ENABLE
        /*
         * Check for the length modifier.
         */
        switch (/* c = */ *++p)
        {
            case 'h':
                if (*++p != 'h')
                {
                    flags_used |= kPRINTF_LengthShortInt;
                    --p;
                }
                else
                {
                    flags_used |= kPRINTF_LengthChar;
                }
                break;
            case 'l':
                if (*++p != 'l')
                {
                    flags_used |= kPRINTF_LengthLongInt;
                    --p;
                }
                else
                {
                    flags_used |= kPRINTF_LengthLongLongInt;
                }
                break;
            default:
                /* we've gone one char too far */
                --p;
                break;
        }
#endif /* PRINTF_ADVANCED_ENABLE */
        /* Now we're ready to examine the format. */
        c = *++p;
        {
            if ((c == 'd') || (c == 'i') || (c == 'f') || (c == 'F') || (c == 'x') || (c == 'X') || (c == 'o') ||
                (c == 'b') || (c == 'p') || (c == 'u'))
            {
                if ((c == 'd') || (c == 'i'))
                {
#if PRINTF_ADVANCED_ENABLE
                    if (flags_used & kPRINTF_LengthLongLongInt)
                    {
                        ival = (int64_t)va_arg(*ap, int64_t);
                    }
                    else
#endif /* PRINTF_ADVANCED_ENABLE */
                    {
                        ival = (int32_t)va_arg(*ap, int32_t);
                    }
                    vlen = ConvertRadixNumToString(vstr, &ival, true, 10, use_caps);
                    vstrp = &vstr[vlen];
#if PRINTF_ADVANCED_ENABLE
                    if (ival < 0)
                    {
                        schar = '-';
                        ++vlen;
                    }
                    else
                    {
                        if (flags_used & kPRINTF_Plus)
                        {
                            schar = '+';
                            ++vlen;
                        }
                        else
                        {
                            if (flags_used & kPRINTF_Space)
                            {
                                schar = ' ';
                                ++vlen;
                            }
                            else
                            {
                                schar = 0;
                            }
                        }
                    }
                    dschar = false;
                    /* Do the ZERO pad. */
                    if (flags_used & kPRINTF_Zero)
                    {
                        if (schar)
                        {
                            func_ptr(buf, schar);
                            count++;
                        }
                        dschar = true;

                        PrintfPaddingCharacter('0', vlen, field_width, &count, func_ptr, buf);
                        vlen = field_width;
                    }
                    else
                    {
                        if (!(flags_used & kPRINTF_Minus))
                        {
                            PrintfPaddingCharacter(' ', vlen, field_width, &count, func_ptr, buf);
                            if (schar)
                            {
                                func_ptr(buf, schar);
                                count++;
                            }
                            dschar = true;
                        }
                    }
                    /* The string was built in reverse order, now display in correct order. */
                    if ((!dschar) && schar)
                    {
                        func_ptr(buf, schar);
                        count++;
                    }
#endif /* PRINTF_ADVANCED_ENABLE */
                }

                if ((c == 'X') || (c == 'x'))
                {
                    if (c == 'x')
                    {
                        use_caps = false;
                    }
#if PRINTF_ADVANCED_ENABLE
                    if (flags_used & kPRINTF_LengthLongLongInt)
                    {
                        uval = (uint64_t)va_arg(*ap, uint64_t);
                    }
                    else
#endif /* PRINTF_ADVANCED_ENABLE */
                    {
                        uval = (uint32_t)va_arg(*ap, uint32_t);
                    }
                    vlen = ConvertRadixNumToString(vstr, &uval, false, 16, use_caps);
                    vstrp = &vstr[vlen];

#if PRINTF_ADVANCED_ENABLE
                    dschar = false;
                    if (flags_used & kPRINTF_Zero)
                    {
                        if (flags_used & kPRINTF_Pound)
                        {
                            func_ptr(buf, '0');
                            func_ptr(buf, (use_caps ? 'X' : 'x'));
                            count += 2;
                            /*vlen += 2;*/
                            dschar = true;
                        }
                        PrintfPaddingCharacter('0', vlen, field_width, &count, func_ptr, buf);
                        vlen = field_width;
                    }
                    else
                    {
                        if (!(flags_used & kPRINTF_Minus))
                        {
                            if (flags_used & kPRINTF_Pound)
                            {
                                vlen += 2;
                            }
                            PrintfPaddingCharacter(' ', vlen, field_width, &count, func_ptr, buf);
                            if (flags_used & kPRINTF_Pound)
                            {
                                func_ptr(buf, '0');
                                func_ptr(buf, use_caps ? 'X' : 'x');
                                count += 2;

                                dschar = true;
                            }
                        }
                    }

                    if ((flags_used & kPRINTF_Pound) && (!dschar))
                    {
                        func_ptr(buf, '0');
                        func_ptr(buf, use_caps ? 'X' : 'x');
                        count += 2;
                        vlen += 2;
                    }
#endif /* PRINTF_ADVANCED_ENABLE */
                }
                if (c == 'o')
                {
                    uval = (uint32_t)va_arg(*ap, uint32_t);
                    radix = 8;
                }
                if (c == 'b')
                {
                    uval = (uint32_t)va_arg(*ap, uint32_t);
                    radix = 2;
                    vstrp = &vstr[vlen];
                }
                if (c == 'p')
                {
                    uval = (uint32_t)va_arg(*ap, void *);
                    radix = 16;
                    vstrp = &vstr[vlen];
                }
                if (c == 'u')
                {
                    uval = (uint32_t)va_arg(*ap, uint32_t);
                    radix = 10;
                    vstrp = &vstr[vlen];
                }
                if ((c == 'o') || (c == 'b') || (c == 'p') || (c == 'u'))
                {
                    vlen = ConvertRadixNumToString(vstr, &uval, false, radix, use_caps);
                    vstrp = &vstr[vlen];
#if PRINTF_ADVANCED_ENABLE
                    if (flags_used & kPRINTF_Zero)
                    {
                        PrintfPaddingCharacter('0', vlen, field_width, &count, func_ptr, buf);
                        vlen = field_width;
                    }
                    else
                    {
                        if (!(flags_used & kPRINTF_Minus))
                        {
                            PrintfPaddingCharacter(' ', vlen, field_width, &count, func_ptr, buf);
                        }
                    }
#endif /* PRINTF_ADVANCED_ENABLE */
                }
#if !PRINTF_ADVANCED_ENABLE
                PrintfPaddingCharacter(' ', vlen, field_width, &count, func_ptr, buf);
#endif /* !PRINTF_ADVANCED_ENABLE */
                if (vstrp != NULL)
                {
                    while (*vstrp)
                    {
                        func_ptr(buf, *vstrp--);
                        count++;
                    }
                }
#if PRINTF_ADVANCED_ENABLE
                if (flags_used & kPRINTF_Minus)
                {
                    PrintfPaddingCharacter(' ', vlen, field_width, &count, func_ptr, buf);
                }
#endif /* PRINTF_ADVANCED_ENABLE */
            }
            else if (c == 'c')
            {
                cval = (char)va_arg(*ap, uint32_t);
                func_ptr(buf, cval);
                count++;
            }
            else if (c == 's')
            {
                sval = (char *)va_arg(*ap, char *);
                if (sval)
                {
                    vlen = (int32_t)strlen(sval);
#if PRINTF_ADVANCED_ENABLE
                    if (!(flags_used & kPRINTF_Minus))
#endif /* PRINTF_ADVANCED_ENABLE */
                    {
                        PrintfPaddingCharacter(' ', vlen, field_width, &count, func_ptr, buf);
                    }
                    while (*sval)
                    {
                        func_ptr(buf, *sval++);
                        count++;
                    }
#if PRINTF_ADVANCED_ENABLE
                    if (flags_used & kPRINTF_Minus)
                    {
                        PrintfPaddingCharacter(' ', vlen, field_width, &count, func_ptr, buf);
                    }
#endif /* PRINTF_ADVANCED_ENABLE */
                }
            }
            else
            {
                func_ptr(buf, c);
                count++;
            }
        }
    }
    return count;
}

